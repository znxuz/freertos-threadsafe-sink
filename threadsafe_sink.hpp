#pragma once

#include <FreeRTOS.h>
#include <semphr.h>
#include <task.h>

#include <cstring>

namespace freertos {
enum struct TSINK_CALL_FROM { ISR, NON_ISR };

using tsink_consume_f = void (*)(const uint8_t* buf, size_t size);

namespace tsink_detail {
template <TSINK_CALL_FROM callsite>
struct mtx_guard {
  mtx_guard(SemaphoreHandle_t mtx) : mtx{mtx} {
    if constexpr (callsite == TSINK_CALL_FROM::ISR)
      xSemaphoreTakeFromISR(mtx, &pxHigherPriorityTaskWoken);
    else
      xSemaphoreTake(mtx, portMAX_DELAY);
  }
  ~mtx_guard() {
    if constexpr (callsite == TSINK_CALL_FROM::ISR)
      portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
    xSemaphoreGive(mtx);
  }

  SemaphoreHandle_t mtx;
  static BaseType_t pxHigherPriorityTaskWoken;
};

#ifndef TSINK_CAPACITY
inline constexpr size_t TSINK_CAPACITY = 2048;
#endif

inline SemaphoreHandle_t write_mtx;
inline TaskHandle_t task_hdl;

inline uint8_t sink[TSINK_CAPACITY]{};
inline bool consumable[TSINK_CAPACITY]{};
inline volatile size_t write_idx = 0;
inline volatile size_t read_idx = 0;

inline tsink_consume_f consume;

inline void consume_and_wait(size_t pos, size_t size) {
  auto update_for_writer = [](size_t pos, size_t size) static {
    for (size_t i = 0; i < size; ++i) consumable[pos + i] = false;
  };

  consume(sink + pos, size);
  ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
  update_for_writer(pos, size);
}

inline void task_impl(void*) {
  while (true) {
    auto end = write_idx;
    if (read_idx == end && !consumable[read_idx]) {
      vTaskDelay(1);
      continue;
    }

    auto size = (read_idx < end ? end : TSINK_CAPACITY) - read_idx;
    consume_and_wait(read_idx, size);
    if (read_idx >= end && end) consume_and_wait(0, end);

    read_idx = end;
  }
}
}  // namespace tsink_detail

// write `len` from `ptr` buffer into the sink
template <TSINK_CALL_FROM callsite>
inline void tsink_write_blocking(const char* ptr, size_t len) {
  using namespace tsink_detail;
  volatile auto _ = mtx_guard<callsite>{write_mtx};
  for (size_t i = 0; i < len; ++i) {
    while (consumable[write_idx]) vTaskDelay(1);
    sink[write_idx] = ptr[i];
    taskENTER_CRITICAL();
    consumable[write_idx] = true;
    write_idx = (write_idx + 1) % TSINK_CAPACITY;
    taskEXIT_CRITICAL();
  }
}

template <TSINK_CALL_FROM callsite>
inline bool tsink_write_or_fail(const char* ptr, size_t len) {
  using namespace tsink_detail;

  // TODO race condition:
  // t1 calls first and gets blocked; t2 calls second and passes
  if (auto space = TSINK_CAPACITY - (write_idx - read_idx); space < len)
    return false;
  tsink_write_blocking<callsite>(ptr, len);
  return true;
}
template <TSINK_CALL_FROM callsite>
inline void tsink_write_str(const char* s) {
  tsink_write_blocking<callsite>(s, strlen(s));
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
