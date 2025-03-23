#pragma once

#include <FreeRTOS.h>
#include <semphr.h>
#include <task.h>

namespace freertos {
enum struct CALLSITE { ISR, NON_ISR };

typedef void (*csink_consume_f)(const uint8_t* buf, size_t size);

namespace detail {
#ifndef SINK_SIZE
inline constexpr size_t SINK_SIZE = 2048;
#endif

inline SemaphoreHandle_t task_semphr;
inline SemaphoreHandle_t write_mtx;
inline TaskHandle_t task_hdl;

inline uint8_t sink[SINK_SIZE];
inline bool consumable[SINK_SIZE];
inline volatile size_t write_idx;

inline csink_consume_f consume;

inline void consume_and_wait(size_t pos, size_t size) {
  auto update_for_writer = [](size_t pos, size_t size) static {
    for (size_t i = 0; i < size; ++i) consumable[pos++] = false;
  };

  consume(sink + pos, size);
  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
  update_for_writer(pos, size);
}

inline void task_impl(void*) {
  size_t start = 0;
  while (true) {
    xSemaphoreTake(task_semphr, portMAX_DELAY);

    auto end = write_idx;
    if (start == end) continue;  // just for safety

    auto size = (start < end ? end : SINK_SIZE) - start;
    consume_and_wait(start, size);
    if (start > end && end) consume_and_wait(0, end);

    start = end;
  }
}
}  // namespace detail

// write `len` from `ptr` buffer into the sink
inline void csink_write(const char* ptr, size_t len) {
  xSemaphoreTake(detail::write_mtx, portMAX_DELAY);
  for (size_t i = 0; i < len; ++i) {
    while (detail::consumable[detail::write_idx]) vTaskDelay(1);

    detail::sink[detail::write_idx] = ptr[i];
    detail::consumable[detail::write_idx] = true;
    detail::write_idx = (detail::write_idx + 1) % detail::SINK_SIZE;
  }
  xSemaphoreGive(detail::task_semphr);
  xSemaphoreGive(detail::write_mtx);
}

// callback upon consume completion to signal the sink task
template <CALLSITE context>
void csink_consume_complete() {
  static BaseType_t xHigherPriorityTaskWoken;
  if constexpr (context == CALLSITE::ISR) {
    vTaskNotifyGiveFromISR(detail::task_hdl, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  } else {
    xTaskNotifyGive(detail::task_hdl);
  }
}

// init function taking a function pointer to consume the bytes in the sink
inline void csink_init(csink_consume_f f, uint32_t priority) {
  detail::consume = f;

  static StaticSemaphore_t write_mtx_buffer, task_semphr_buffer;
  configASSERT(
      (detail::write_mtx = xSemaphoreCreateMutexStatic(&write_mtx_buffer)));
  configASSERT((detail::task_semphr =
                    xSemaphoreCreateBinaryStatic(&task_semphr_buffer)));

  constexpr size_t STACK_SIZE = configMINIMAL_STACK_SIZE * 4;
  static StackType_t task_stack[STACK_SIZE];
  static StaticTask_t task_buffer;
  configASSERT((detail::task_hdl = xTaskCreateStatic(
                    detail::task_impl, "cs", STACK_SIZE, NULL, priority,
                    task_stack, &task_buffer)) != NULL)
}
}  // namespace freertos
