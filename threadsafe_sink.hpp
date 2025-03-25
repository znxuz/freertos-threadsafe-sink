#pragma once

#include <FreeRTOS.h>
#include <semphr.h>
#include <task.h>

#include <cstring>

namespace freertos {
enum struct TSINK_CALL_FROM { ISR, NON_ISR };

using tsink_consume_f = void (*)(const uint8_t* buf, std::size_t size);

namespace tsink_detail {
#ifndef SINK_SIZE
inline constexpr size_t SINK_SIZE = 2048;
#endif

inline SemaphoreHandle_t write_mtx;
inline TaskHandle_t task_hdl;

inline uint8_t sink[SINK_SIZE];
inline bool consumable[SINK_SIZE];
inline volatile size_t write_idx;

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
  size_t pos = 0;
  while (true) {
    auto end = write_idx;
    if (pos == end && !consumable[pos]) {
      vTaskDelay(1);
      continue;
    }

    auto size = (pos < end ? end : SINK_SIZE) - pos;
    consume_and_wait(pos, size);
    if (pos >= end && end) consume_and_wait(0, end);

    pos = end;
  }
}
}  // namespace tsink_detail

// write `len` from `ptr` buffer into the sink
inline void tsink_write(const char* ptr, size_t len) {
  xSemaphoreTake(tsink_detail::write_mtx, portMAX_DELAY);
  for (size_t i = 0; i < len; ++i) {
    while (tsink_detail::consumable[tsink_detail::write_idx]) vTaskDelay(1);

    tsink_detail::sink[tsink_detail::write_idx] = ptr[i];
    taskENTER_CRITICAL();
    tsink_detail::consumable[tsink_detail::write_idx] = true;
    tsink_detail::write_idx =
        (tsink_detail::write_idx + 1) % tsink_detail::SINK_SIZE;
    taskEXIT_CRITICAL();
  }
  xSemaphoreGive(tsink_detail::write_mtx);
}

inline void tsink_write_str(const char* s) { tsink_write(s, strlen(s)); }

// callback upon consume completion to signal the sink task
template <TSINK_CALL_FROM callsite>
void tsink_consume_complete() {
  if constexpr (callsite == TSINK_CALL_FROM::ISR) {
    static BaseType_t xHigherPriorityTaskWoken;
    vTaskNotifyGiveFromISR(tsink_detail::task_hdl, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  } else {
    xTaskNotifyGive(tsink_detail::task_hdl);
  }
}

// init function taking a function pointer to consume the bytes in the sink
inline void tsink_init(tsink_consume_f f, uint32_t priority) {
  tsink_detail::consume = f;

  static StaticSemaphore_t write_mtx_buffer;
  configASSERT((tsink_detail::write_mtx =
                    xSemaphoreCreateMutexStatic(&write_mtx_buffer)));

  constexpr size_t STACK_SIZE = configMINIMAL_STACK_SIZE * 4;
  static StackType_t task_stack[STACK_SIZE];
  static StaticTask_t task_buffer;
  configASSERT((tsink_detail::task_hdl = xTaskCreateStatic(
                    tsink_detail::task_impl, "tsink", STACK_SIZE, NULL,
                    priority, task_stack, &task_buffer)) != NULL)
}
}  // namespace freertos
