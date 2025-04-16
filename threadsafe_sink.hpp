#pragma once

#include <FreeRTOS.h>
#include <semphr.h>

#include <atomic>
#include <concepts>
#include <type_traits>

namespace freertos {
enum struct TSINK_CALL_FROM { ISR, NON_ISR };

using tsink_consume_fn = void (*)(const uint8_t*, size_t);

namespace tsink_detail {
// TODO: support for half-word & word length
template <typename T>
concept Elem = std::same_as<T, uint8_t> || std::same_as<T, char>;

template <typename T>
concept ElemContainer = requires(T t) {
  requires Elem<std::decay_t<decltype(t[0])>>;
  t.data();
  t.size();
};

#ifndef TSINK_CAPACITY
inline constexpr size_t TSINK_CAPACITY = 2048;
#endif

inline TaskHandle_t task_hdl;
inline SemaphoreHandle_t write_mtx;

inline uint8_t sink[TSINK_CAPACITY]{};
inline volatile size_t read_idx = 0;
inline std::atomic<size_t> write_idx = 0;

inline volatile size_t ticket_matcher = 0;

inline tsink_consume_fn consume_fn;

inline size_t tsink_size() { return write_idx - read_idx; }

inline size_t tsink_space() { return TSINK_CAPACITY - tsink_size(); }

inline size_t normalize(size_t idx) { return idx % TSINK_CAPACITY; }

struct mtx_guard {
  mtx_guard() { configASSERT(xSemaphoreTake(write_mtx, portMAX_DELAY)); }
  ~mtx_guard() { configASSERT(xSemaphoreGive(write_mtx)); }
};

inline void task_impl(void*) {
  auto consume_and_wait = [](size_t pos, size_t size) static {
    if (!size) return;
    consume_fn(sink + pos, size);
    ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
  };

  while (true) {
    if (size_t size = tsink_size(); size) {
      auto wrap_around = ((normalize(read_idx) + size) / TSINK_CAPACITY) *
                         normalize(read_idx + size);
      auto immediate = size - wrap_around;
      consume_and_wait(normalize(read_idx), immediate);
      consume_and_wait(0, wrap_around);
      read_idx += size;
    } else {
      vTaskDelay(pdMS_TO_TICKS(1));
    }
  }
}
}  // namespace tsink_detail

using tsink_detail::tsink_size;
using tsink_detail::tsink_space;

inline void tsink_reset_ticket() { tsink_detail::ticket_matcher = 0; }

inline bool tsink_write_or_fail(tsink_detail::Elem auto elem) {
  using namespace tsink_detail;

  // TODO: does the load and CAS need to be paired in certain memory order?
  auto expected = write_idx.load();
  if (expected - read_idx == TSINK_CAPACITY) return false;
  if (write_idx.compare_exchange_strong(expected, expected + 1)) {
    sink[normalize(expected)] = elem;
    return true;
  }
  return false;
}

// performance at the mercy of the scheduler
template <tsink_detail::Elem E>
inline void tsink_write_ordered(const E* ptr, size_t len, size_t ticket) {
  using namespace tsink_detail;

  while (true) {
    if (ticket == ticket_matcher) {
      while (tsink_space() < len) vTaskDelay(pdMS_TO_TICKS(1));
      for (size_t i = 0; i < len; ++i)
        configASSERT(tsink_write_or_fail(ptr[i]));
      ticket_matcher += 1;
      return;
    }
  }
}

// write `len` from `ptr` buffer into the sink
template <tsink_detail::Elem E>
inline void tsink_write_blocking(const E* ptr, size_t len) {
  using namespace tsink_detail;

  while (true) {
    if (volatile auto _ = mtx_guard{}; tsink_space() >= len) {
      for (size_t i = 0; i < len; ++i)
        configASSERT(tsink_write_or_fail(ptr[i]));
      return;
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

inline void tsink_write_blocking(const tsink_detail::ElemContainer auto& t) {
  tsink_write_blocking(t.data(), t.size());
}

// callback upon consume completion to signal the sink task
template <TSINK_CALL_FROM callsite>
void tsink_consume_complete() {
  using namespace tsink_detail;
  if constexpr (callsite == TSINK_CALL_FROM::ISR) {
    static BaseType_t xHigherPriorityTaskWoken;
    vTaskNotifyGiveFromISR(task_hdl, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  } else {
    xTaskNotifyGive(task_hdl);
  }
}

// init function taking a function pointer to consume the bytes in the sink
inline void tsink_init(tsink_consume_fn f, uint32_t priority) {
  using namespace tsink_detail;
  consume_fn = f;

  static StaticSemaphore_t write_mtx_buffer;
  configASSERT((write_mtx = xSemaphoreCreateMutexStatic(&write_mtx_buffer)));

  constexpr size_t STACK_SIZE = 512;
  static StackType_t task_stack[STACK_SIZE];
  static StaticTask_t task_buffer;
  configASSERT((task_hdl = xTaskCreateStatic(task_impl, "tsink", STACK_SIZE,
                                             NULL, priority, task_stack,
                                             &task_buffer)) != NULL)
}
}  // namespace freertos
