# Underwater Breathing Rate Analysis

## Overview

This document covers the design and implementation approach for detecting and measuring a scuba diver's breathing rate from underwater audio, captured via an I2S MEMS microphone connected to an ESP32 or ESP32-S3.

The goal is to extract breathing rate (breaths per minute) in real time from the acoustic signature of the regulator, and to support offline characterisation of that signature using recorded audio.

---

## Acoustic Signature of Scuba Breathing

Each breath cycle produces a distinctive two-phase acoustic event:

| Phase | Duration | Frequency Content | Description |
| --- | --- | --- | --- |
| Inhale | 1–2 s | ~500 Hz – 4 kHz | Broadband rushing noise as the demand valve opens; similar to white noise with a mid-to-high frequency bias |
| Pause | ~0.5 s | Ambient only | Near-silence between phases |
| Exhale | 1–2 s | ~50 – 500 Hz | Low-pitched bubbling and rolling as air exits the exhaust valve; characteristic bubble resonance |
| Pause | ~0.5 s | Ambient only | Brief silence before next inhale |

Breathing rate at rest is 10–15 breaths/minute; under exertion up to 25–30. This gives a fundamental rate of 0.16–0.5 Hz — a very slow signal that requires a different analysis approach from the FFT-based frequency measurement used in the spectrogram display.

---

## DSP Pipeline

### Why FFT Alone Is Not Sufficient for Rate Estimation

FFT is the right tool for **feature extraction** — pulling band energies from each audio frame. It is the wrong tool for measuring breathing rate itself because resolving 0.16–0.5 Hz with adequate spectral resolution would require a multi-minute analysis window. The appropriate tools for rate estimation are envelope following, onset detection, and autocorrelation.

The pipeline has two distinct layers:

```
Audio (16000 Hz, 16-bit)
    │
    ▼ [FFT — 1024 points, Hann window]
Spectral content per frame (~15 fps at 16 kHz)
    │
    ▼ [Band energy extraction — 3 bands, reuses FFT output]
e_low (50–500 Hz), e_mid (500 Hz–4 kHz), e_broad (all bins)
    │
    ▼ [Leaky integrator — first-order IIR, ~700 ms time constant]
Smooth energy envelope at ~10 Hz
    │
    ▼ [Onset detection or autocorrelation]
Breath events / breathing period
    │
    ▼
Breathing rate in breaths/minute
```

---

## Stage 1: Band Energy Extraction

Three frequency bands cover the inhale/exhale distinction. At 16000 Hz with a 1024-point FFT, each bin is 15.6 Hz wide:

| Band | Bin range | Frequency | Detects |
| --- | --- | --- | --- |
| Low | 4–32 | ~50–500 Hz | Exhale bubbles |
| Mid | 32–256 | ~500 Hz–4 kHz | Inhale rushing |
| Broadband | 4–256 | all | Overall respiratory activity |

```cpp
float e_low = 0, e_mid = 0, e_broad = 0;
for (int i = 4;  i <= 32;  i++) e_low   += mag[i] * mag[i];
for (int i = 32; i <= 256; i++) e_mid   += mag[i] * mag[i];
for (int i = 4;  i <= 256; i++) e_broad += mag[i] * mag[i];
```

This piggybacks on the existing FFT magnitude loop with negligible additional cost.

### Phase Classification

With two band energies, each frame can be classified:

- `e_mid > threshold AND e_mid > e_low` → **inhale**
- `e_low > threshold AND e_low > e_mid` → **exhale**
- both below threshold → **silence / pause**

An **adaptive noise floor** is essential underwater. Maintain a long-running slow minimum of `e_broad` — this is the ambient noise baseline. Subtract it before applying thresholds.

---

## Stage 2: Envelope Following

A leaky integrator (first-order IIR) smooths noisy per-frame energy into a stable envelope. The time constant controls responsiveness:

```cpp
// Alpha ≈ 0.97 gives ~700 ms time constant at 10 Hz effective rate
env_low   = alpha * env_low   + (1 - alpha) * e_low;
env_mid   = alpha * env_mid   + (1 - alpha) * e_mid;
env_broad = alpha * env_broad + (1 - alpha) * e_broad;
```

Downsample to 10 Hz by acting on every Nth frame (e.g. every 6th frame at ~60 fps). A 0.5 Hz signal is adequately represented at 10 Hz.

Add hysteresis: separate attack and release thresholds to avoid rapid toggling at breath boundaries.

---

## Stage 3: Breathing Rate Estimation

### Option A — Inter-onset Interval (simple)

Measure the time between successive inhale onsets. Keep a circular buffer of the last 5–8 intervals and compute the mean. Robust to one missed or doubled detection.

### Option B — Autocorrelation (robust)

Maintain a circular buffer of `env_broad` values at 10 Hz for the last 60 seconds (600 floats, 2.4 KB). Compute the autocorrelation for lags corresponding to 2–8 seconds (7–30 breaths/min):

```cpp
// Pseudocode — compute once per second
for (int lag = 20; lag <= 80; lag++) {  // 2–8 seconds at 10 Hz
    float sum = 0;
    for (int t = 0; t < N - lag; t++)
        sum += env_buf[t] * env_buf[t + lag];
    acorr[lag] = sum;
}
int peak_lag = argmax(acorr, 20, 80);
float bpm = 60.0f / (peak_lag / 10.0f);
```

This approach does not require explicit event detection and is tolerant of irregular breathing and missed beats. At 600² multiply-adds computed once per second it is well within the ESP32 budget.

---

## Practical Challenges Underwater

| Challenge | Mitigation |
| --- | --- |
| Fin kicks / body movement | Minimum breath duration gate (>0.8 s filters most impulses) |
| Nearby diver exhalation | Amplitude threshold; their bubbles are quieter at distance |
| Ambient reef / current noise | Adaptive noise floor updated on a slow timescale |
| Depth (denser gas at depth) | Minor effect at recreational depths; calibration session handles it |
| Regulator model variation | Calibration step: record 10 breaths at the surface before diving |

---

## Hardware Options

### Option 1 — M5StickC Plus (PDM microphone)

The M5StickC Plus has an integrated PDM MEMS microphone (MSM261S4030H0). It is driven by the ESP32 I2S peripheral in PDM mode.

**Limitations for breathing analysis:**

- PDM low-frequency rolloff begins around 100 Hz, attenuating the exhale bubble band (50–500 Hz)
- 4 MB flash with 192 KB SPIFFS using `min_spiffs.csv` — very limited raw audio storage
- No SD card
- SNR ~61 dB

**I2S configuration (PDM mode):**

```cpp
i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_PDM),
    .sample_rate          = 16000,
    .bits_per_sample      = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format       = I2S_CHANNEL_FMT_ALL_RIGHT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count        = 8,
    .dma_buf_len          = 128,
};
```

**Storage options on M5StickC:**

The `min_spiffs.csv` partition table allocates 192 KB to SPIFFS, sufficient for only ~12 seconds of 16 kHz 16-bit audio. Replacing it with a custom no-OTA partition table recovers the second OTA app slot:

```csv
# mercator_record.csv
nvs,      data, nvs,     0x9000,  0x5000,
app0,     app,  factory, 0x10000, 0x100000,
spiffs,   data, spiffs,  0x110000,0x2EF000,
```

This gives ~2.93 MB SPIFFS — approximately 3 minutes at 16 kHz 16-bit, or 6 minutes at 8-bit. Sufficient for a characterisation session but not a full dive.

For a full dive, store pre-computed band energy features rather than raw audio. At 3 floats + timestamp at 10 Hz (16 bytes/s), 2.93 MB SPIFFS holds over 50 hours.

**WiFi streaming (pool use only):** Stream raw PCM via UDP to a laptop at the surface at 16 kHz 16-bit (~32 KB/s). Suitable for controlled characterisation sessions.

---

### Option 2 — LilyGo T4 ESP32-S3 with SPH0645LM4H (preferred)

The Mercator Origins oceanic GoPro enclosure runs on a LilyGo T4 ESP32-S3 board with an Adafruit I2S MEMS Microphone Breakout (SPH0645LM4H) connected as a separate I2S peripheral. This combination is significantly better suited to breathing analysis.

**SPH0645LM4H vs M5StickC PDM mic:**

| Property | M5StickC PDM | SPH0645LM4H |
| --- | --- | --- |
| Interface | PDM (CLK + DATA) | Standard I2S (BCLK + WS + DATA) |
| Bit depth | 16-bit | 24-bit output, 18-bit significant |
| Low-frequency response | Rolls off ~100 Hz | **Flat to 50 Hz** |
| Upper frequency | ~10 kHz | 15 kHz |
| SNR | ~61 dB | **65 dB** |
| Storage | SPIFFS only | **SD card** |

The flat response to 50 Hz is the most important difference for this application: exhale bubble energy peaks between 100–300 Hz and is accurately captured rather than attenuated by the PDM rolloff.

**LilyGo T4 ESP32-S3 capabilities:**

- ESP32-S3 dual-core LX7 at 240 MHz
- 16 MB flash
- **8 MB PSRAM** — essential for reliable SD writes
- **SD card slot** — removes all storage constraints
- USB-C

**I2S configuration (standard I2S, not PDM):**

```cpp
i2s_config_t cfg = {
    .mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate          = 16000,
    .bits_per_sample      = I2S_BITS_PER_SAMPLE_32BIT,  // 24-bit data in 32-bit frame
    .channel_format       = I2S_CHANNEL_FMT_ONLY_LEFT,  // SEL pin = GND on Adafruit board
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count        = 8,
    .dma_buf_len          = 256,
    .use_apll             = false,
};

i2s_pin_config_t pins = {
    .bck_io_num   = PIN_BCLK,
    .ws_io_num    = PIN_WS,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num  = PIN_DATA,
};
```

The Adafruit breakout ties SEL to GND, selecting the left channel. Two breakouts can be used for stereo by connecting one SEL to 3.3V.

**Sample extraction:** The SPH0645LM4H outputs 24-bit audio left-justified in a 32-bit I2S frame, with 18 bits significant. Reading as int32 and right-shifting by 14 yields a 16-bit sample:

```cpp
int32_t raw[256];
size_t  bytesread;
i2s_read(I2S_NUM_0, raw, sizeof(raw), &bytesread, portMAX_DELAY);

int16_t samples[256];
for (int i = 0; i < 256; i++)
    samples[i] = (int16_t)(raw[i] >> 14);
```

---

## SD Card WAV Recording (ESP32-S3 / LilyGo T4)

SD cards using FAT have unpredictable write latency — a single write can block for 50–150 ms when the card erases a sector. At 16 kHz the I2S DMA buffer fills every ~64 ms, so SD hiccups will drop samples without buffering. The 8 MB PSRAM resolves this with a double-buffer: one half fills from I2S while the other flushes to SD.

At 16 kHz 16-bit mono (32 KB/s), a 16 GB card holds ~138 hours. File size for a 45-minute dive is ~86 MB.

```cpp
static const size_t HALF_BUF = 256 * 1024;  // 256 KB per half

static int8_t *buf_a;
static int8_t *buf_b;

void writeWavHeader(File &f, uint32_t sampleRate, uint32_t numSamples) {
    uint32_t dataSize   = numSamples * 2;
    uint32_t chunkSize  = 36 + dataSize;
    uint16_t audioFmt   = 1;
    uint16_t channels   = 1;
    uint16_t bitsPerSmp = 16;
    uint32_t byteRate   = sampleRate * 2;
    uint16_t blockAlign = 2;

    f.write("RIFF", 4);  f.write((uint8_t*)&chunkSize,  4);
    f.write("WAVEfmt ", 8);
    uint32_t fmtSize = 16;
                          f.write((uint8_t*)&fmtSize,    4);
                          f.write((uint8_t*)&audioFmt,   2);
                          f.write((uint8_t*)&channels,   2);
                          f.write((uint8_t*)&sampleRate, 4);
                          f.write((uint8_t*)&byteRate,   4);
                          f.write((uint8_t*)&blockAlign, 2);
                          f.write((uint8_t*)&bitsPerSmp, 2);
    f.write("data", 4);   f.write((uint8_t*)&dataSize,   4);
}

void recordToSD(const char *path, uint32_t durationSec) {
    buf_a = (int8_t*)ps_malloc(HALF_BUF);
    buf_b = (int8_t*)ps_malloc(HALF_BUF);

    File f = SD.open(path, FILE_WRITE);
    const uint32_t sampleRate   = 16000;
    const uint32_t totalSamples = sampleRate * durationSec;
    writeWavHeader(f, sampleRate, totalSamples);

    const size_t samplesPerHalf = HALF_BUF / sizeof(int16_t);
    int32_t  raw[256];
    int16_t *active   = (int16_t*)buf_a;
    int16_t *flushing = (int16_t*)buf_b;
    size_t   pos      = 0;
    uint32_t written  = 0;

    while (written < totalSamples) {
        size_t bytesread;
        i2s_read(I2S_NUM_0, raw, sizeof(raw), &bytesread, portMAX_DELAY);
        size_t count = bytesread / sizeof(int32_t);

        for (size_t i = 0; i < count && written < totalSamples; i++) {
            active[pos++] = (int16_t)(raw[i] >> 14);
            written++;

            if (pos == samplesPerHalf) {
                f.write((uint8_t*)active, HALF_BUF);
                int16_t *tmp = active;
                active       = flushing;
                flushing     = tmp;
                pos          = 0;
            }
        }
    }

    if (pos > 0)
        f.write((uint8_t*)active, pos * sizeof(int16_t));

    f.close();
    free(buf_a);
    free(buf_b);
}
```

**Recommended recording parameters:**

| Parameter | Value | Rationale |
| --- | --- | --- |
| Sample rate | 16000 Hz | Nyquist at 8 kHz covers both breathing bands comfortably |
| Bit depth | 16-bit | Standard WAV, directly readable by Python and Audacity |
| PSRAM buffer per half | 256 KB | Absorbs SD write latency spikes up to ~8 seconds |
| Duration per session | 3–10 minutes | Sufficient for algorithm development |
| File size (10 min) | ~19 MB | Negligible on any modern SD card |

---

## Offline Characterisation Workflow

Before implementing the real-time algorithm on the device, characterise the breathing signature from recordings. This determines band boundaries, envelope time constants, and detection thresholds from real data.

### Step 1 — Record Labelled Sessions

Record sessions covering:
- Resting breathing at the surface (baseline)
- Resting at depth (5 m, 10 m)
- Swimming (elevated rate, movement noise)
- With another diver nearby (interference)

### Step 2 — Label in Audacity

1. Load the WAV in Audacity (free, cross-platform)
2. Switch to spectrogram view: Track → Spectrogram. Inhale and exhale appear as alternating bands of high and low frequency energy — visually obvious.
3. Add a Label Track: Tracks → Add New → Label Track
4. Mark start and end of each inhale and exhale phase
5. Export: File → Export → Export Labels (tab-delimited timestamps)

### Step 3 — Python Analysis

**Environment:**
```
pip install numpy scipy librosa matplotlib soundfile jupyter
```

**Wideband spectrogram — visualise before measuring:**
```python
import librosa, librosa.display, matplotlib.pyplot as plt, numpy as np

y, sr = librosa.load('dive_session.wav', sr=16000, mono=True)

D    = librosa.stft(y, n_fft=1024, hop_length=512)
S_db = librosa.amplitude_to_db(np.abs(D), ref=np.max)

librosa.display.specshow(S_db, sr=sr, hop_length=512,
                         x_axis='time', y_axis='hz', fmax=6000)
plt.colorbar(format='%+2.0f dB')
```

Overlay Audacity label timestamps as vertical lines to confirm the frequency bands match the visual events.

**Band energy timeseries — validate band choices:**
```python
hop   = 512
n_fft = 1024
S     = np.abs(librosa.stft(y, n_fft=n_fft, hop_length=hop)) ** 2
freqs = librosa.fft_frequencies(sr=sr, n_fft=n_fft)

low_mask  = (freqs >= 50)  & (freqs < 500)
mid_mask  = (freqs >= 500) & (freqs < 4000)

e_low  = S[low_mask].sum(axis=0)
e_mid  = S[mid_mask].sum(axis=0)
times  = librosa.frames_to_time(np.arange(S.shape[1]),
                                 sr=sr, hop_length=hop)

plt.plot(times, 10*np.log10(e_low  + 1e-10), label='Low 50–500 Hz (exhale)')
plt.plot(times, 10*np.log10(e_mid  + 1e-10), label='Mid 500–4k Hz (inhale)')
```

**Noise floor characterisation:**
```python
silence      = y[:sr*5]   # first 5 seconds if silent
S_noise      = np.abs(librosa.stft(silence, n_fft=1024)) ** 2
noise_low    = np.median(S_noise[low_mask].sum(axis=0))
noise_mid    = np.median(S_noise[mid_mask].sum(axis=0))
print(f'SNR low band: {10*np.log10(e_low.mean()/noise_low):.1f} dB')
print(f'SNR mid band: {10*np.log10(e_mid.mean()/noise_mid):.1f} dB')
```

Less than 6 dB SNR in either band indicates a detection problem in that condition. 10 dB or more is comfortable.

**Envelope follower simulation — tune the time constant:**
```python
def leaky(x, alpha):
    env = np.zeros_like(x)
    env[0] = x[0]
    for i in range(1, len(x)):
        env[i] = alpha * env[i-1] + (1 - alpha) * x[i]
    return env

# Downsample to ~10 Hz
stride = int(sr / hop / 10)
e_ds   = e_low[::stride]

for alpha in [0.90, 0.95, 0.97, 0.99]:
    plt.plot(leaky(e_ds, alpha), label=f'α={alpha}')
```

Plot against label timestamps. The right α shows clean transitions without frame-to-frame noise.

**Autocorrelation — verify rate is recoverable:**
```python
from scipy.signal import correlate

env   = leaky(e_low[::stride], 0.97)
env   -= env.mean()
acorr = correlate(env, env, mode='full')[len(env)-1:]

lags_sec = np.arange(len(acorr)) / 10.0
mask     = (lags_sec >= 2) & (lags_sec <= 8)
peak     = lags_sec[mask][np.argmax(acorr[mask])]
print(f'Breathing period: {peak:.1f} s ({60/peak:.1f} breaths/min)')
```

A clear autocorrelation peak confirms the approach will work. A flat or noisy autocorrelation indicates insufficient SNR for that recording condition.

**Detection threshold — ROC curve:**
```python
from sklearn.metrics import precision_recall_curve

# Construct y_true from Audacity labels, y_score from envelope
precision, recall, thresholds = precision_recall_curve(y_true, y_score)
f1 = 2 * precision * recall / (precision + recall + 1e-10)
best_threshold = thresholds[np.argmax(f1)]
print(f'Optimal threshold: {best_threshold:.4f}')
```

### Step 4 — Extract Parameters for the ESP32

| Parameter | Determined by |
| --- | --- |
| Low/mid band boundary (Hz) | Spectrogram inspection |
| Envelope IIR alpha | Leaky integrator simulation plots |
| Detection threshold | ROC curve at desired F1 operating point |
| Minimum breath duration gate | Histogram of labelled breath durations |
| Expected SNR | Noise floor characterisation |
| Autocorrelation window length | Width of peak in autocorrelation |

### Recommended Recording Sequence

1. **Pool session, laptop connected** — WiFi streaming from M5StickC or direct cable, raw audio to laptop. Controlled conditions, known breathing patterns. Develops the algorithm.
2. **Pool session, standalone** — Record to SD card (GoPro enclosure) or SPIFFS burst (M5StickC). Validates that hardware-captured audio matches the characterisation.
3. **Open water, feature logging** — Once algorithm is validated, switch to logging pre-computed band energies for full dives. The raw audio is no longer needed.

---

## Implementation Roadmap

| Stage | Description |
| --- | --- |
| 1 | Add band energy computation to existing FFT magnitude loop (free, reuses output) |
| 2 | Add leaky integrator envelope follower, downsample to 10 Hz |
| 3 | Add adaptive noise floor and hysteretic threshold |
| 4 | Implement inter-onset interval counter for initial rate estimate |
| 5 | Add 60-second autocorrelation buffer for robust rate estimate |
| 6 | Display breathing rate on screen (spare pixel rows below the 128-row spectrogram) |
| 7 | Add feature logging to SPIFFS or SD card |

Stages 1–3 add fewer than 50 lines to the existing FFT task. Stages 4–5 add another 60–80 lines. The full implementation fits within the existing RTOS task architecture without a new task.
