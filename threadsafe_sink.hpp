#pragma once

#include <FreeRTOS.h>
#include <semphr.h>

#include <array>
#include <atomic>
#include <span>
#include <type_traits>

namespace freertos {
namespace detail {
template <typename T>
concept AtomicType = sizeof(T) == 1 || sizeof(T) == 2 || sizeof(T) == 4;

template <typename C>
concept AtomicTypeContainer = requires(C t) {
  requires AtomicType<std::decay_t<decltype(t[0])>>;
  t.data();
  t.size();
};
}  // namespace detail

enum struct CALL_FROM : uint8_t { ISR, NON_ISR };

using size_type = size_t;

template <detail::AtomicType T, size_type N>
struct tsink {
  using consume_fn_ptr = void (*)(const uint8_t*, size_type);

  tsink(consume_fn_ptr f, uint32_t priority) : consume_fn{f} {
    auto task_impl = [](void* arg) {
      auto consume_and_wait = [](std::span<const T> ringbuf, size_type pos,
                                 size_type size, consume_fn_ptr consume_fn) {
        if (!size) return;
        consume_fn(reinterpret_cast<const uint8_t*>(ringbuf.data() + pos),
                   size * sizeof(T));
        ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
      };

      auto* sink = reinterpret_cast<tsink<T, N>*>(arg);
      while (true) {
        if (size_type sz = sink->size(); sz) {
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

    constexpr size_type STACK_SIZE = 256;
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
    write_idx.store(rhs.write_idx.exchange(write_idx.load()));
    swap(ticket_matcher, rhs.ticket_matcher);
    swap(task_hdl, rhs.task_hdl);
    swap(write_mtx, rhs.write_mtx);
    swap(consume_fn, rhs.consume_fn);
  }

  constexpr size_type size() const { return write_idx - read_idx; }
  constexpr size_type space() const { return ringbuf.size() - size(); }
  void reset_ticket() { ticket_matcher = 0; }

  bool write_or_fail(detail::AtomicType auto elem) {
    auto expected = write_idx.load();
    if (expected - read_idx == ringbuf.size()) return false;
    if (write_idx.compare_exchange_strong(expected, expected + 1)) {
      ringbuf[normalize(expected)] = elem;
      return true;
    }
    return false;
  }

  // write `len` from `ptr` buffer into the sink
  void write_blocking(const detail::AtomicType auto* ptr, size_type len) {
    while (true) {
      if (volatile auto _ = mtx_guard{write_mtx};
          space() >= len * sizeof(*ptr)) {
        for (size_type i = 0; i < len; ++i) configASSERT(write_or_fail(ptr[i]));
        return;
      }
      vTaskDelay(pdMS_TO_TICKS(1));
    }
  }

  void write_blocking(const detail::AtomicTypeContainer auto& t) {
    write_blocking(t.data(), t.size());
  }

  // performance at the mercy of the scheduler
  inline void write_ordered(const detail::AtomicType auto* ptr, size_type len,
                            size_type ticket) {
    while (true) {
      if (ticket == ticket_matcher) {
        while (space() < len) vTaskDelay(pdMS_TO_TICKS(1));
        for (size_type i = 0; i < len; ++i) configASSERT(write_or_fail(ptr[i]));
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
  std::array<T, N> ringbuf{};
  volatile size_type read_idx = 0;
  std::atomic<size_type> write_idx = 0;
  size_type ticket_matcher = 0;
  TaskHandle_t task_hdl = nullptr;
  SemaphoreHandle_t write_mtx = nullptr;
  consume_fn_ptr consume_fn = nullptr;

  struct mtx_guard {
    explicit mtx_guard(SemaphoreHandle_t write_itx) : mtx{write_itx} {
      configASSERT(xSemaphoreTake(mtx, portMAX_DELAY));
    }
    ~mtx_guard() { configASSERT(xSemaphoreGive(mtx)); }

    SemaphoreHandle_t mtx;
  };

  constexpr size_type normalize(size_type idx) const {
    return idx % ringbuf.size();
  }
};

}  // namespace freertos
