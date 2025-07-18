# `freertos-threadsafe-sink`

A header-only, array-based, lock-free multi-producer byte/half-word/word sink
for FreeRTOS.

## Prerequisites

- Architecture with default byte/half-word/word atomicity, i.e. ARM
- FreeRTOS Kernel above `V10.2.1` (depends on Task Notification)
- C++23 (depends on `constexpr if`)

## Usage

Everything is under the namespace `freertos`.

### 1. Initialization

Construct a `tsink<T, N>` with `T: u8, u16, u32`.

A size of a power of two is **strongly** recommended to get better performance
by turning division & modulo operations (used in a hot loop by the reader for
normalizing the indices) into 1-cycle instructions.

Pass a consume function and a priority level for the reader task to
automatically call the function and consume the data.

```cpp
using consume_fn_ptr = void (*)(const uint8_t*, size_t);
```

### 2. Write Data

Call `write_<variant>()` member functions to write data into the sink.

```cpp
template <typename T>
concept AtomicType = sizeof(T) == 1 || sizeof(T) == 2 || sizeof(T) == 4;

template <typename C>
concept AtomicTypeContainer = requires(C t) {
  requires AtomicType<std::decay_t<decltype(t[0])>>;
  t.data();
  t.size();
};

bool write_or_fail(AtomicType auto elem);
void write_blocking(const AtomicType auto* ptr, size_t len);
void write_blocking(const AtomicTypeContainer auto& t);
void write_ordered(const AtomicType auto* ptr, size_t len, size_t ticket);
```

Thread-safe, lock-free single writes (`write_or_fail`) are done via atomic
CAS(Compare and Swap) while leveraging the default read/write atomicity for
aligned data on ARM.

For a chunk of bytes greater than 32 bits, thread-safety is guaranteed by
synchronizing the calls internally using a statically initialized FreeRTOS
mutex.

Strict FIFO order *can* be achieved only via `write_ordered()` by passing a
atomically incremented counter starting from 0 as a unique *ticket*. But the
performance degrades exponentially with the number of threads there are on
calling this function concurrently, as the non-deterministic scheduling can't
possibly prioritize the thread with the next "correct" ticket. Also call
`reset_ticket()` to reset the internal counter, when the passed ticket should
start from 0 again.

### 3. Signal Consumption Completion

After the data is consumed, call `consume_complete()` to signal the reader task.
This callback can be invoked in an ISR context.

This mechanism is needed to allow the consumption function to work
asynchronously, e.g. to synchronize with external IO hardware (e.g., a DMA
controller) to initiate subsequent writes only after the previous transfer has
been completed.

```cpp
enum struct CALL_FROM { ISR, NON_ISR };

template <CALL_FROM callsite>
void consume_complete();
```

## examples

```cpp
using atomic_type = uint8_t;

auto tsink_consume = [](const atomic_type* buf, size_t size) static {
  HAL_UART_Transmit(&huart3, buf, size, HAL_MAX_DELAY);
  sink->consume_complete<CALL_FROM::NON_ISR>();
};
constexpr auto POWER_TWO_SIZE = 2048uz;
auto sink = tsink<atomic_type, POWER_TWO_SIZE>(tsink_consume,
                                                    osPriorityAboveNormal);

atomic_type buf[100]; // char buf[100];
std::array<atomic_type, 15> arr{};

auto success = sink.write_or_fail('c');
sink.write_blocking("hello world"sv);
sink.write_blocking(buf, std::strlen(buf));
sink.write_blocking(arr);

auto ticket_machine = std::atomic<size_t>{};
sink.write_ordered(buf, std::strlen(buf), ticket_machine.fetch_add(1));
```

For a platform using STM32-HAL with cache-enabled DMA and an ISR triggered upon
DMA transfer completion:

```cpp
using namespace freertos;

std::shared_ptr<tsink<uint8_t, 2048>> sink;

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

  sink = std::make_shared<tsink<uint8_t, 2048>>(consume_dma,
                                                osPriorityAboveNormal);
  sink->write_blocking("sink initialized\n"sv);
}
```

# TODO

- API to manually get & consume the data instead automatically via task
