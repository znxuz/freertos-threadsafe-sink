# freertos-threadsafe-sink

`freertos-threadsafe-sink` is a thread-safe, header-only, multi-producer sink
based on a circular buffer for FreeRTOS.

## Prerequisites

- Architecture with atomic byte and word access i.e. ARM
- FreeRTOS Kernel above V10.2.1 (depends on Task Notification)
- C++23 (depends on `constexpr if`, `static operator()` for lambdas)

## Usage

### 1. Initialization

Initialize the Sink by calling `tsink_init()`.

Pass a consume function (`tsink_consume_f`) and a priority level for the sink
task to consume the data.

```cpp
using tsink_consume_f = void (*)(const uint8_t* buf, size_t size);

inline void tsink_init(tsink_consume_f f, uint32_t priority);
```

Optionally pass a custom size as compile macro `-DTSINK_CAPACITY` for the
internal circular buffer. A size of a power of two is **strongly** recommended
for 1-cycle bitwise AND-operation, which is done on upon every access to the
read/write buffer pointers.

### 2. Write Data

Call `tsink_write_<variant>()` to write data into the sink. Thread-safety is
guaranteed by synchronizing the calls internally using a mutex. Strict FIFO is
guaranteed only with `tsink_write_ordered()` by passing a atomically incremented
counter starting from 0 as a unique "ticket".

```cpp
inline void tsink_write_ordered(const char* ptr, size_t len, size_t ticket);
inline void tsink_write_blocking(const char* ptr, size_t len);
inline void tsink_write_or_fail(const char* ptr, size_t len);
inline void tsink_write_str(std::string_view s);
```

### 3. Signal Consumption Completion

Upon completion of data transfer, call `tsink_consume_complete()` to signal
the sink task.

This allows the buffer to then be able overwrite the consumed data if needed.
This callback can be invoked in an ISR context.

```cpp
enum struct TSINK_CALL_FROM { ISR, NON_ISR };

template <TSINK_CALL_FROM callsite>
void tsink_consume_complete();
```

For example on a platform configured with STM32-HAL with cache-enabled DMA and
an ISR triggered upon DMA transfer completion:

```cpp
void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart) {
  if (huart->Instance != huart3.Instance) return;
  tsink_consume_complete<TSINK_CALL_FROM::ISR>();
}

void main() {
  auto tsink_consume_dma = [](const uint8_t* buf, size_t size) static {
    auto flush_cache_aligned = [](uintptr_t addr, size_t size) static {
      constexpr auto align_addr = [](uintptr_t addr) { return addr & ~0x1F; };
      constexpr auto align_size = [](uintptr_t addr, size_t size) {
        return size + ((addr) & 0x1F);
      };

      SCB_CleanDCache_by_Addr(reinterpret_cast<uint32_t*>(align_addr(addr)),
                              align_size(addr, size));
    };

    flush_cache_aligned(reinterpret_cast<uintptr_t>(buf), size);
    HAL_UART_Transmit_DMA(&huart3, buf, size);
  };

  tsink_init(tsink_consume_dma, osPriorityAboveNormal);
```

Or using blocking-IO:

```cpp
  auto tsink_consume = [](const uint8_t* buf, size_t size) static {
    HAL_UART_Transmit(&huart3, buf, size, HAL_MAX_DELAY);
    tsink_consume_complete<TSINK_CALL_FROM::NON_ISR>();
  };

  // tsink_init(tsink_consume, osPriorityAboveNormal);
```
