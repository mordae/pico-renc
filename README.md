# pico-renc: Rotary Encoder Driver

To use these drivers with `pico-sdk`, modify your `CMakeLists.txt`:

```cmake
add_subdirectory(vendor/pico-renc)
target_link_libraries(your_target pico_renc ...)
```

If you do not have your board header file, create `include/boards/myboard.h` and use it like this:

```cmake
set(PICO_BOARD myboard)
set(PICO_BOARD_HEADER_DIRS ${CMAKE_CURRENT_LIST_DIR}/include/boards)
```

Your header file may include the original board file, but it must define following:

```c
/* Number of rotary encoders on the board. */
#define NUM_RENC 1

/* How many microseconds to wait in order to resolve possible bounce. */
#define RENC_DEBOUNCE_US 1000

/* How many events can the queue hold. */
#define RENC_QUEUE_SIZE 16

/* Below how many milliseconds fixate the direction. */
#define RENC_FIX_DIR_MS 50

/* Under how many milliseconds discard completely. */
#define RENC_DISCARD_MS 5

/* Enable pull up on rotary encoder pins. */
#define RENC_PULL_UP 1
```

See `include/renc.h` for interface.
