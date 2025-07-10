#pragma once

#include <FreeRTOS.h>
#include <semphr.h>

#include <array>
#include <atomic>
#include <concepts>
#include <span>
#include <type_traits>

namespace freertos {
enum struct CALL_FROM { ISR, NON_ISR };

// TODO: variable atomic length support
template <typename T>
concept Elem = std::same_as<T, uint8_t> || std::same_as<T, char>;

template <typename C>
concept ElemContainer = requires(C t) {
  requires Elem<std::decay_t<decltype(t[0])>>;
  t.data();
  t.size();
};

template <size_t N>
struct tsink {
  struct mtx_guard {
    explicit mtx_guard(SemaphoreHandle_t write_itx) : mtx{write_itx} {
      configASSERT(xSemaphoreTake(mtx, portMAX_DELAY));
    }
    ~mtx_guard() { configASSERT(xSemaphoreGive(mtx)); }

    SemaphoreHandle_t mtx;
  };

  using consume_fn_ptr = void (*)(const uint8_t*, size_t);

  using size_type = size_t;

  tsink(consume_fn_ptr f, uint32_t priority) : consume_fn{f} {
    auto task_impl = [](void* arg) {
      auto consume_and_wait = [](std::span<const uint8_t> ringbuf, size_t pos,
                                 size_t size, consume_fn_ptr consume_fn) {
        if (!size) return;
        consume_fn(ringbuf.data() + pos, size);
        ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
      };

      auto* sink = reinterpret_cast<tsink<N>*>(arg);
      while (true) {
        if (size_t sz = sink->size(); sz) {
          auto idx = sink->normalize(sink->read_idx);
          auto wrap_around =
              ((idx + sz) / sink->ringbuf.size()) *  // either 1 or 0
              ((idx + sz) % sink->ringbuf.size());   // actual amount
          auto immediate = sz - wrap_around;
          consume_and_wait(sink->ringbuf, idx, immediate, sink->consume_fn);
          consume_and_wait(sink->ringbuf, 0, wrap_around, sink->consume_fn);
          sink->read_idx += sz;
        } else {
          vTaskDelay(pdMS_TO_TICKS(1));
        }
      }
    };

    static StaticSemaphore_t write_mtx_buffer;
    configASSERT((write_mtx = xSemaphoreCreateMutexStatic(&write_mtx_buffer)));

    constexpr size_t STACK_SIZE = 512;  // TODO: reduce the size amap
    static StackType_t task_stack[STACK_SIZE];
    static StaticTask_t task_buffer;
    configASSERT((task_hdl = xTaskCreateStatic(task_impl, "tsink", STACK_SIZE,
                                               this, priority, task_stack,
                                               &task_buffer)) != NULL)
  }

  ~tsink() {
    vTaskDelete(task_hdl);
    vSemaphoreDelete(write_mtx);
  }

  tsink(tsink&& rhs) {
    using std::swap;
    swap(ringbuf, rhs.ringbuf);
    swap(read_idx, rhs.read_idx);
    swap(write_idx, rhs.write_idx);
    swap(ticket_matcher, rhs.ticket_matcher);
    swap(task_hdl, rhs.task_hdl);
    swap(write_mtx, rhs.write_mtx);
    swap(consume_fn, rhs.consume_fn);
  }

  constexpr size_type size() const { return write_idx - read_idx; }
  constexpr size_type space() const { return ringbuf.size() - size(); }
  void reset_ticket() { ticket_matcher = 0; }

  bool write_or_fail(Elem auto elem) {
    auto expected = write_idx.load();
    if (expected - read_idx == ringbuf.size()) return false;
    if (write_idx.compare_exchange_strong(expected, expected + 1)) {
      ringbuf[normalize(expected)] = elem;
      return true;
    }
    return false;
  }

  // write `len` from `ptr` buffer into the sink
  void write_blocking(const Elem auto* ptr, size_t len) {
    while (true) {
      if (volatile auto _ = mtx_guard{write_mtx}; space() >= len) {
        for (size_t i = 0; i < len; ++i) configASSERT(write_or_fail(ptr[i]));
        return;
      }
      vTaskDelay(pdMS_TO_TICKS(1));
    }
  }

  void write_blocking(const ElemContainer auto& t) {
    write_blocking(t.data(), t.size());
  }

  // performance at the mercy of the scheduler
  inline void write_ordered(const Elem auto* ptr, size_t len, size_t ticket) {
    while (true) {
      if (ticket == ticket_matcher) {
        while (space() < len) vTaskDelay(pdMS_TO_TICKS(1));
        for (size_t i = 0; i < len; ++i) configASSERT(write_or_fail(ptr[i]));
        ticket_matcher += 1;
        return;
      }
    }
  }
  //
  // callback upon consume completion to signal the sink task
  template <CALL_FROM callsite>
  void consume_complete() {
    if constexpr (callsite == CALL_FROM::ISR) {
      static BaseType_t xHigherPriorityTaskWoken;
      vTaskNotifyGiveFromISR(task_hdl, &xHigherPriorityTaskWoken);
      portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    } else {
      xTaskNotifyGive(task_hdl);
    }
  }

 private:
  std::array<uint8_t, N> ringbuf{};
  volatile size_type read_idx = 0;
  std::atomic<size_type> write_idx = 0;
  size_type ticket_matcher = 0;
  TaskHandle_t task_hdl = nullptr;
  SemaphoreHandle_t write_mtx = nullptr;
  consume_fn_ptr consume_fn = nullptr;

  constexpr size_type normalize(size_type idx) const {
    return idx % ringbuf.size();
  }
};

}  // namespace freertos
