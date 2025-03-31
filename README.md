# freertos-threadsafe-sink

`freertos-threadsafe-sink` is a thread-safe, header-only, multi-producer sink
based on a circular buffer for FreeRTOS.

## Prerequisites

- ARM architecture with atomic byte and word access
- FreeRTOS Kernel above V10.2.1 (depends on Task Notification)
- C++23 (depends on `constexpr`, `static operator()` for lambdas)

## Usage

### 1. Initialization

Initialize the Sink by calling `tsink_init()`.

Pass a consume function (`tsink_consume_f`) and a priority level for the sink
task to consume the data.

```cpp
using tsink_consume_f = void (*)(const uint8_t* buf, size_t size);

inline void tsink_init(tsink_consume_f f, uint32_t priority);
```

### 2. Write Data

Call `tsink_write` to write data into the sink. Thread-safely is guaranteed by
synchronizing the calls internally using a mutex.

```cpp
inline void tsink_write(const char* ptr, size_t len);
inline void tsink_write_str(const char* s);
```

### 3. Signal Consumption Completion

Upon completion of data consumption, call `tsink_consume_complete` to signal the
sink task.

This allows the buffer to then overwrite the consumed data if needed. This
callback can be invoked in an ISR context, for example, when a asynchronous DMA
operation finishes reading data from the sink and triggers an ISR.

```cpp
enum struct TSINK_CALL_FROM { ISR, NON_ISR };

template <TSINK_CALL_FROM callsite>
void tsink_consume_complete();
```
