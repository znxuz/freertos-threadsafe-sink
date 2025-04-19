# freertos-threadsafe-sink

A thread-safe, lock-free or with optional strict FIFO guarantee, array-based,
header-only multi-producer byte sink for FreeRTOS.

## Prerequisites

- Architecture with default atomic byte and word access, i.e. ARM
- FreeRTOS Kernel above `V10.2.1` (depends on Task Notification)
- C++23 (depends on `constexpr if`, `static operator()` for lambdas)

## Usage

Everything is under the namespace `freertos::tsink`.

### 1. Initialization

Initialize by calling `init()`.

Pass a consume function and a priority level for the sink task to consume the
data.

```cpp
using consume_fn = void (*)(const uint8_t*, size_t);

inline void init(consume_fn f, uint32_t priority);
```

Size for the internal circular array is configurable by passing a compile macro
`-DTSINK_CAPACITY`. By default its 2k in size. A size of a power of two is
**strongly** recommended for turning modulo operations (normalizing the indices)
into a 1-cycle bitwise AND-operation.

### 2. Write Data

Call `write_<variant>()` to write data into the sink.

```cpp
template <typename T>
concept Elem = std::same_as<T, uint8_t> || std::same_as<T, char>;

template <typename C>
concept ElemContainer = requires(C t) {
  requires Elem<std::decay_t<decltype(t[0])>>;
  t.data();
  t.size();
};

inline bool write_or_fail(Elem auto elem);
inline void write_blocking(const Elem auto* ptr, size_t len);
inline void write_blocking(const ElemContainer auto& t);
inline void write_ordered(const Elem auto* ptr, size_t len, size_t ticket);
```

Thread-safe, lock-free byte writes are done via atomic CAS. And for a chunk of
bytes, thread-safety is guaranteed by synchronizing the calls internally using a
FreeRTOS mutex.

Strict FIFO order can be achieved only with `write_ordered()` by passing a
atomically incremented counter starting from 0 as a unique *ticket*. Performance
degrades exponentially with the number of threads there are on calling this
function concurrently, as the non-deterministic scheduling can't possibly
prioritize the thread with the next "correct" ticket. Also call `reset_ticket()`
when the atomic ticket starts from 0 again.

#### examples

```cpp
uint8_t buf[100];
char buf[100]; // alternative

auto success = write_or_fail('c');
write_blocking("hello world"sv);
write_blocking(buf, std::strlen(buf));

auto ticket_machine = std::atomic<size_t>{};
write_ordered(buf, std::strlen(buf), ticket_machine.fetch_add(1));
```

### 3. Signal Consumption Completion

Upon completion of data transfer, call `consume_complete()` to signal
the sink task. This callback can be invoked in an ISR context.

This mechanism enables the sink to synchronize with external IO hardware (e.g.,
a DMA controller), allowing it to initiate subsequent writes only after previous
transfers have completed.

```cpp
enum struct CALL_FROM { ISR, NON_ISR };

template <CALL_FROM callsite>
void consume_complete();
```

For example on a platform using STM32-HAL with cache-enabled DMA and an ISR
triggered upon DMA transfer completion:

```cpp
using namespace freertos;

void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart) {
  if (huart->Instance == huart3.Instance) // UART handle for DMA-TX
    tsink::consume_complete<tsink::CALL_FROM::ISR>();
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

  tsink::init(consume_dma, osPriorityAboveNormal);
}
```

Or using blocking-IO:

```cpp
void main() {
  auto consume = [](const uint8_t* buf, size_t size) static {
    HAL_UART_Transmit(&huart3, buf, size, HAL_MAX_DELAY);
    tsink::consume_complete<tsink::CALL_FROM::NON_ISR>();
  };

  tsink::init(consume, osPriorityAboveNormal);
}
```
