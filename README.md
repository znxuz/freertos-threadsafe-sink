# `freertos-threadsafe-sink`

A lock-free, array-based, header-only multi-producer byte sink for FreeRTOS.

## Prerequisites

- Architecture with default atomic byte and word access, i.e. ARM
- FreeRTOS Kernel above `V10.2.1` (depends on Task Notification)
- C++23 (depends on `constexpr if`)

## Usage

Everything is under the namespace `freertos`.

### 1. Initialization

Construct a `tsink<N>`.

A size of a power of two is **strongly** recommended to get better performance
by turning division & modulo operations (used in a hot loop by the reader for
normalizing the indices) into 1-cycle instructions.

Pass a consume function and a priority level for the reader task to
automatically consume the data.

```cpp
using consume_fn_ptr = void (*)(const uint8_t*, size_t);
```

### 2. Write Data

Call `write_<variant>()` member functions to write data into the sink.

```cpp
template <typename T>
concept Elem = std::same_as<T, uint8_t> || std::same_as<T, char>;

template <typename C>
concept ElemContainer = requires(C t) {
  requires Elem<std::decay_t<decltype(t[0])>>;
  t.data();
  t.size();
};

bool write_or_fail(Elem auto elem);
void write_blocking(const Elem auto* ptr, size_t len);
void write_blocking(const ElemContainer auto& t);
void write_ordered(const Elem auto* ptr, size_t len, size_t ticket);
```

Thread-safe, lock-free byte writes are done via atomic CAS(Compare and Swap)
while leveraging the default read/write atomicity for aligned byte on ARM.

For a chunk of bytes, thread-safety is guaranteed by synchronizing the calls
internally using a statically initialized FreeRTOS mutex.

Strict FIFO order can be achieved only with `write_ordered()` by passing a
atomically incremented counter starting from 0 as a unique *ticket*. Performance
degrades exponentially with the number of threads there are on calling this
function concurrently, as the non-deterministic scheduling can't possibly
prioritize the thread with the next "correct" ticket. Also call `reset_ticket()`
when the atomic ticket starts from 0 again.

### 3. Signal Consumption Completion

Upon completion of data transfer, call `consume_complete()` to signal
the reader task. This callback can be invoked in an ISR context.

This mechanism is needed to allow the reader task to synchronize with external
IO hardware (e.g., a DMA controller) to initiate subsequent writes only after
the previous transfer has been completed.

```cpp
enum struct CALL_FROM { ISR, NON_ISR };

template <CALL_FROM callsite>
void consume_complete();
```

## examples

```cpp
auto tsink_consume = [](const uint8_t* buf, size_t size) static {
  HAL_UART_Transmit(&huart3, buf, size, HAL_MAX_DELAY);
  sink->consume_complete<CALL_FROM::NON_ISR>();
};
auto sink = tsink<2048>(tsink_consume, osPriorityAboveNormal);;

uint8_t buf[100];
char buf[100]; // alternative

auto success = sink.write_or_fail('c');
sink.write_blocking("hello world"sv);
sink.write_blocking(buf, std::strlen(buf));

auto ticket_machine = std::atomic<size_t>{};
sink.write_ordered(buf, std::strlen(buf), ticket_machine.fetch_add(1));
```

For a platform using STM32-HAL with cache-enabled DMA and an ISR triggered upon
DMA transfer completion:

```cpp
using namespace freertos;

std::shared_ptr<tsink<2048>> sink;

void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart) {
  if (huart->Instance == huart3.Instance) // UART handle for DMA-TX
  sink->consume_complete<CALL_FROM::ISR>();
}

void main() {
  auto consume_dma = [](const uint8_t* buf, size_t size) static {
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

  sink = std::make_shared<tsink<2048>>(consume_dma, osPriorityAboveNormal);
  sink->write_blocking("sink initialized\n"sv);
}
```

# TODO

- atomic write also for word length data
- API to manually get & consume the data instead automatically via task
