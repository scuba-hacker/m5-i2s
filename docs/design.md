# M5StickC Plus I2S Microphone Spectrogram — Design & Implementation

## Overview

This project runs on an M5StickC Plus and displays a real-time audio spectrogram (waterfall plot) on the device's built-in TFT screen. The PDM microphone is sampled via the ESP32's I2S peripheral, a 1024-point FFT is computed on each frame, and the resulting frequency magnitudes are written into a circular display buffer that scrolls across the screen.

## Hardware

| Component | Detail |
|---|---|
| MCU | ESP32 (via M5StickC Plus) |
| Display | ST7789, 135×240 px, operated landscape (240×135) |
| Microphone | PDM microphone, CLK on GPIO 0, DATA on GPIO 34 |
| Framework | Arduino / ESP-IDF via PlatformIO |

## Build Configuration

The project uses PlatformIO with the `espressif32 @ 6.5.0` platform and the `m5stick-c` board target. C++17 is enabled. TFT_eSPI is configured entirely via build flags — no `UserSetup.h` is needed. The two library dependencies are forked versions of M5StickC-Plus and TFT_eSPI, referenced by git URL from `platformio.ini`.

## Signal Chain

```
PDM microphone
    │  GPIO 0 (CLK/WS), GPIO 34 (DATA)
    ▼
I2S driver (PDM mode, 44100 Hz, 16-bit mono)
    │  i2s_read() → 2048 bytes = 1024 × int16 samples
    ▼
Scale to float (map int16_min..int16_max → −2000..+2000)
    │  1024 real input samples
    ▼
1024-point real FFT (split-radix, forward)
    │  output: 512 complex bins (interleaved re/im floats)
    ▼
Magnitude of bins 1–128  (√(re² + im²))
    │  128 frequency values, clamped to [0, 2000]
    ▼
Map magnitude 0..2000 → 0..255 (uint8)
    │  128 intensity values for this time column
    ▼
Write to circular display buffer fft_dis_buff[posData][0..127]
    │  posData advances 0→240, wraps to 0
    ▼
Colormap lookup (ImageData[intensity × 3 + {R,G,B}])
    │  256-entry RGB palette, black→blue→cyan gradient
    ▼
Draw pixels into TFT_eSprite (240×128), pushSprite to display
```

### Frequency Resolution

| Parameter | Value |
|---|---|
| Sample rate | 44100 Hz |
| FFT size | 1024 points |
| Bin width | ~43 Hz |
| Displayed bins | 1–128 |
| Displayed range | ~43 Hz – ~5.5 kHz |

The display is oriented so frequency increases from bottom to top (`fft_dis_buff[x][128 - bin_index]`).

## I2S Initialisation

`InitI2SMicroPhone()` in [src/main.cpp](../src/main.cpp) configures I2S peripheral 0 in PDM receive mode:

- **Mode**: `I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_PDM`
- **Sample rate**: 44100 Hz
- **Bit depth**: 16-bit (hardware decimates the PDM bitstream to 16-bit PCM internally)
- **Channel**: mono right channel (`I2S_CHANNEL_FMT_ALL_RIGHT`)
- **DMA**: 2 buffers of 128 bytes each
- **Communication format**: `I2S_COMM_FORMAT_STAND_I2S` (ESP-IDF ≥ 4.1) or `I2S_COMM_FORMAT_I2S` (older)

Pin mapping: WS/CLK → GPIO 0, DATA\_IN → GPIO 34; BCK and DATA\_OUT are left as `I2S_PIN_NO_CHANGE`.

## RTOS Task Architecture

Three FreeRTOS tasks run concurrently:

| Task | Stack | Priority | Responsibility |
|---|---|---|---|
| `MicRecordfft` | 2 KB | 5 | I2S read, FFT, write to display buffer |
| `Drawdisplay` | 2 KB | 4 | Read display buffer, render pixels, push sprite |
| Arduino `loop()` | default | 1 | Button polling, task gate control |

### Semaphore Design

Three semaphores are created as mutexes but are used as binary gate semaphores:

| Semaphore | Purpose |
|---|---|
| `xSemaphore` | Mutex protecting `fft_dis_buff` and `posData` |
| `start_fft` | Gate: taken in `setup()`, given by `DisplayMicro()` to unblock `MicRecordfft` |
| `start_dis` | Gate: taken in `setup()`, given by `DisplayMicro()` to unblock `Drawdisplay` |

`DisplayMicro()` runs a tight loop: it gives both gate semaphores, calls `M5.update()` to poll buttons, then waits to re-acquire both gates. This paces the FFT and display tasks to one iteration per `loop()` cycle and stops them when a button is pressed (though in this firmware `loop()` immediately calls `DisplayMicro()` again, so the spectrogram runs continuously).

## Display Buffer & Scrolling

`fft_dis_buff[241][128]` is a circular time-frequency intensity map:

- Dimension 0 (241 columns) is time, indexed by `posData` which increments each FFT frame and wraps at 241.
- Dimension 1 (128 rows) is frequency, stored high-frequency-first so pixel row 0 is ~5.5 kHz.

`Drawdisplay` reconstructs the visible window by reading columns `posData % 240` through `posData % 240 + 239`, wrapping at 240, to produce a continuously scrolling display without any buffer copy.

## Colormap

`ImageData[768]` in [src/Icon.c](../src/Icon.c) is a 256-entry RGB palette (3 bytes per entry). Intensity value 0 maps to black; the palette progresses through dark blues, bright cyans, and into other hues at higher intensities, giving the spectrogram a "heat" appearance where louder frequencies appear lighter/warmer.

## FFT Implementations

Both implementations are compiled unconditionally. The active one is chosen at startup by the `kUseEspDsp` constant in [src/main.cpp](../src/main.cpp), which is passed as the task function pointer when creating the `MicRecordfft` FreeRTOS task.

### Robin Scheibler split-radix (`MicRecordfft`)

[src/fft.cpp](../src/fft.cpp) is a standalone split-radix radix-2 DIT FFT written by Robin Scheibler (MIT licence, 2017). Key properties:

- Supports real and complex, forward and inverse transforms.
- Size must be a power of two; validated at `fft_init()` time.
- Twiddle factors are precomputed at init and freed by `fft_destroy()`.
- The split-radix path (`USE_SPLIT_RADIX 1`) uses base cases `fft4` and `fft8` (unrolled, `LARGE_BASE_CASE 1`) to reduce recursion overhead on the ESP32.
- For the real forward FFT (`rfft`), a two-for-the-price-of-one trick runs an n/2-point complex FFT then applies post-processing to recover the full n/2 + 1 unique complex bins.

A fresh `fft_config_t` is allocated on every frame and freed afterwards — correct but carries per-frame malloc/free overhead.

No windowing is applied, so spectral leakage causes tones to smear across adjacent bins.

### Espressif ESP-DSP (`MicRecordfftEspDsp`)

Uses Espressif's [esp-dsp](https://github.com/espressif/esp-dsp) library. Key properties:

- `dsps_fft2r_fc32` — in-place radix-2 complex FFT with Xtensa-assembly optimisation; significantly faster than generic C on the ESP32.
- Requires a one-time init call (`dsps_fft2r_init_fc32`) and explicit bit-reversal (`dsps_bit_rev_fc32`) after each transform.
- Operates on a statically allocated 2048-float global buffer (`esp_dsp_fft_buf`), so there is no per-frame heap allocation.
- A **Hann window** is precomputed at startup (`dsps_wind_hann_f32`) and applied to each frame before the FFT. This eliminates most spectral leakage, producing noticeably sharper frequency lines on the spectrogram compared to the rectangular-windowed Scheibler path.

The input is packed as 1024 complex samples (real = windowed audio, imaginary = 0), so a full 1024-point complex FFT is performed. Only bins 1–128 are read for the display, which covers the same ~43 Hz – 5.5 kHz range as the other implementation.

## TFTTerminal

[src/TFTTerminal.cpp](../src/TFTTerminal.cpp) implements a `Print`-compatible scrolling text terminal backed by a `TFT_eSprite`. It maintains an internal 60-line × 55-character ring buffer and renders to the sprite on each `write()` call. It is defined in this project but not called from `main.cpp`; it exists as a utility for serial-style debug output to the display if needed.

## File Map

| File | Role |
|---|---|
| `src/main.cpp` | Application entry point: I2S init, RTOS tasks, display loop |
| `src/fft.cpp` / `fft.h` | ESP32 FFT library (split-radix radix-2 DIT) |
| `src/Icon.c` | Colour palette (`ImageData`) and error icon bitmap (`error_48`) |
| `src/TFTTerminal.cpp` / `.h` | Scrolling TFT text terminal utility |
| `platformio.ini` | Build configuration, library dependencies, TFT_eSPI pin defines |
