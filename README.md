# freertos-threadsafe-sink

`freertos-threadsafe-sink` is a thread-safe, header-only, multi-producer sink
based on a circular buffer for FreeRTOS.

## Prerequisites

- C++23 (depends on `constexpr`, `static operator()` for lambdas)
- FreeRTOS Kernel above V10.2.1 (depends on Task Notification)

## Usage

### 1. Initialization

Initialize the Sink by calling `csink_init()`.

Pass a consume function (`csink_consume_f`) and a priority level for the sink
task to consume the data.

```cpp
typedef void (*csink_consume_f)(const uint8_t* buf, size_t size);

inline void csink_init(csink_consume_f f, uint32_t priority);
```

### 2. Write Data

Call `csink_write` to write data into the sink. Thread-safely is guaranteed by
synchronizing the calls internally using a mutex.

```cpp
inline void csink_write(const char* ptr, size_t len);
```

### 3. Signal Consumption Completion

Upon completion of data consumption, call `csink_consume_complete` to signal the
sink task.

This allows the buffer to then overwrite the consumed data if needed. This
callback can be invoked in an ISR context, for example, when a asynchronous DMA
operation finishes reading data from the sink and triggers an ISR.

```cpp
enum struct CALLSITE { ISR, NON_ISR };

template <CALLSITE context>
void csink_consume_complete();
```
