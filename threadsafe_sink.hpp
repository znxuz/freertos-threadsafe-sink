#pragma once

#include <FreeRTOS.h>
#include <semphr.h>

#include <cstring>
#include <span>
#include <string_view>

namespace freertos {
enum struct TSINK_CALL_FROM { ISR, NON_ISR };

using tsink_consume_f = void (*)(const uint8_t* buf, size_t size);

namespace tsink_detail {
#ifndef TSINK_CAPACITY
inline constexpr size_t TSINK_CAPACITY = 2048;
#endif

inline SemaphoreHandle_t write_mtx;
inline TaskHandle_t task_hdl;

inline uint8_t sink[TSINK_CAPACITY]{};
inline volatile size_t write_idx = 0;
inline volatile size_t read_idx = 0;

inline tsink_consume_f consume;

inline size_t tsink_size() { return write_idx - read_idx; }

inline size_t tsink_space() { return TSINK_CAPACITY - tsink_size(); }

struct mtx_guard {
  mtx_guard() { configASSERT(xSemaphoreTake(write_mtx, portMAX_DELAY)); }
  ~mtx_guard() { configASSERT(xSemaphoreGive(write_mtx)); }
};

inline void task_impl(void*) {
  auto consume_and_wait = [](size_t pos, size_t size) static {
    if (!size) return;
    consume(sink + pos, size);
    ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
  };

  size_t size{};
  while (true) {
    if (!(size = tsink_size())) {
      vTaskDelay(pdMS_TO_TICKS(1));
      continue;
    }

    auto wrap_around = ((read_idx % TSINK_CAPACITY + size) / TSINK_CAPACITY) *
                       ((read_idx + size) % TSINK_CAPACITY);
    auto immediate = size - wrap_around;
    consume_and_wait(read_idx % TSINK_CAPACITY, immediate);
    consume_and_wait(0, wrap_around);
    read_idx += size;
  }
}
}  // namespace tsink_detail

inline void tsink_write_ordered(const char* ptr, size_t len, size_t ticket) {
  using namespace tsink_detail;

  static size_t ticket_matcher;
  while (true) {
    if (ticket == ticket_matcher) {
      while (tsink_space() < len) vTaskDelay(pdMS_TO_TICKS(1));
      for (size_t i = 0; i < len; ++i) {
        sink[write_idx % TSINK_CAPACITY] = ptr[i];
        write_idx += 1;
      }
      ticket_matcher += 1;
      return;
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

inline bool tsink_write_or_fail(const char* ptr, size_t len) {
  using namespace tsink_detail;

  volatile auto _ = mtx_guard{};
  if (tsink_space() < len) return false;
  for (size_t i = 0; i < len; ++i) {
    sink[write_idx % TSINK_CAPACITY] = ptr[i];
    write_idx += 1;
  }
  return true;
}

// write `len` from `ptr` buffer into the sink
inline void tsink_write_blocking(const char* ptr, size_t len) {
  using namespace tsink_detail;
  while (!tsink_write_or_fail(ptr, len)) vTaskDelay(pdMS_TO_TICKS(1));
}

inline void tsink_write_str(std::string_view s) {
  tsink_write_blocking(s.data(), s.size());
}

template <typename T>
concept ByteType = std::same_as<T, uint8_t> || std::same_as<T, char>;

template <ByteType T>
inline void tsink_write_sp(std::span<T> sp) {
  tsink_write_blocking(sp.data(), sp.size());
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
inline void tsink_init(tsink_consume_f f, uint32_t priority) {
  using namespace tsink_detail;
  consume = f;

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
