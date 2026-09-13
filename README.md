# MT6701 Magnetic Encoder, Motor H-Bridge Control & Kalman Filter - ESP-IDF v6

*Read in other languages: [Português](README.pt-br.md)*

This repository contains the demonstration of the integration of the **MT6701** 14-bit magnetic encoder and the **BTS7960 (IBT-2)** H-bridge motor driver with **ESP-IDF v6**, utilizing standalone, reusable ESP-IDF components and an advanced **3D Kalman Filter** to estimate the position, speed (RPM), and acceleration (RPM/s) of a rotating shaft in real time.

The project is configured to run on the **ESP32-S3** microcontroller and consumes its external dependencies through Git submodules.

> **Code walkthrough:** see the [real-time code and architecture guide](docs/arquitetura-tempo-real.md) (Portuguese). It documents startup, the 3 kHz cycle, 1 kHz control, internal data structures, instrumentation, telemetry, and every log field.

---

## 🛠️ Project Architecture

The workspace is organized as follows:
- **`components/esp-mt6701`**: Git submodule for the MT6701 14-bit magnetic encoder driver, utilizing the modern ESP-IDF master I2C driver (`driver/i2c_master.h`) and optimized to run strictly in read-only mode (software-driven offset and direction).
- **`components/esp-engine-driver`**: Git submodule for the BTS7960 H-bridge driver, owning MCPWM and optional ADC1/DMA acquisition of `R_IS` and `L_IS`.
- **`components/esp_pid`**: Reusable, instance-based PID with saturation, filtered derivative, and back-calculation anti-windup.
- **`components/kalman-filter-c`**: Git submodule pointing to the pure C Kalman Filter library.
- **`components/esp_rt_diagnostics`**: Reusable development-time component for bounded timing statistics, event counters, deadlines, and immutable diagnostic snapshots. It does not record time series.
- **`components/esp_rt_diagnostics_reporter`**: Reusable asynchronous reporter that moves diagnostics and an application payload to a low-priority task without blocking the monitored loop.
- **`main/`**: Real-time application that reads the MT6701 and updates the Kalman filter at **3 kHz**, runs a speed PID at **1 kHz**, drives the H-bridge, and publishes telemetry every 5 seconds.

### Separation of responsibilities

| Responsibility | Location | Purpose |
|---|---|---|
| Acquisition and estimation | `realtime_loop.c` and `engine_angle_kalman.c` | Reads the sensor and estimates angle, speed, and acceleration at 3 kHz |
| Control | `components/esp_pid` and `motor_controller.c` | Executes the reusable PID at 1 kHz and alternates the experiment reference every 2 s by default |
| Instrumentation | `components/esp_rt_diagnostics` | Measures jitter, duration, errors, and deadline violations; does not log |
| Reporting | `components/esp_rt_diagnostics_reporter` | Owns the overwrite queue, Core 0 task, generic diagnostic logs, and quiet/log-affected classification |
| Application telemetry | `realtime_telemetry.c` and `.h` | Defines the motor payload and formats motor/current/recorder state through a reporter callback |

**Instrumentation measures timing behavior. Telemetry transports and presents those measurements.** Neither is the control law.

---

## ⚙️ Project Configurations

### Thread Safety (Component Configs)
Both drivers include Kconfig flags to toggle FreeRTOS Mutex synchronization at compile time:
*   **`CONFIG_MT6701_THREAD_SAFE`** (Default: `y`): Synchronizes I2C master register accesses.
*   **`CONFIG_ENGINE_THREAD_SAFE`** (Default: `y`): Synchronizes H-bridge speed adjustment updates.
*   *Note:* If unchecked, all mutex instructions are compiled out to provide lock-free, zero-overhead execution for maximum performance.
*   In this application, `sdkconfig.defaults` disables both mutexes because only the Core 1 real-time task accesses the sensor and commands the motor after initialization.

### I2C GPIO Pin Configuration (Application Config)
Directly configurable in `menuconfig`. Defaults:
*   **`CONFIG_APP_I2C_SDA_PIN`** (Default: `8`)
*   **`CONFIG_APP_I2C_SCL_PIN`** (Default: `9`)
*   **`CONFIG_APP_I2C_CLOCK_HZ`** (Default: `1000000`): MT6701 clock, configurable from 100 kHz to 1 MHz.

### H-Bridge MCPWM Configuration (Component Config)
Exposes physical configuration settings for the BTS7960:
*   **`CONFIG_ENGINE_PWM_FREQ_HZ`** (Default: `20000` / 20 kHz): The PWM frequency. Under the ESP32-S3's 80 MHz MCPWM timer clock limit, this yields exactly **4000 steps of resolution**.
*   **`CONFIG_ENGINE_PIN_RPWM`** (Default: `1`): GPIO pin for Forward direction PWM.
*   **`CONFIG_ENGINE_PIN_LPWM`** (Default: `2`): GPIO pin for Reverse direction PWM.
*   **`CONFIG_ENGINE_PIN_ENABLE`** (Default: `3`): GPIO pin for both R_EN/L_EN tied together.
*   **`CONFIG_ENGINE_DIRECTION_DEAD_TIME_US`** (Default: `50`): Coast interval inserted before reversing a nonzero drive command.

### Real-Time Diagnostics (Development)
*   **`CONFIG_ESP_RT_DIAGNOSTICS_ENABLE`** (Default: `y`): Enables diagnostic collection and snapshot publication. Disable it to compile the hot-path instrumentation into no-ops.
*   **`CONFIG_ESP_RT_DIAGNOSTICS_DETAILED_TIMING`** (Default: `y`): Measures named I2C, Kalman, control, and snapshot stages plus sampling/control intervals.
*   **`CONFIG_ESP_RT_DIAGNOSTICS_WINDOW_MS`** (Default: `5000`): Sets the default snapshot window; an `esp_rt_diag_config_t` instance may override it.

These snapshots combine reusable timing diagnostics with an instantaneous,
application-owned PID state. They are development diagnostics, not control
time-series recording.

### BTS7960 Current Measurement

*   **`CONFIG_ENGINE_CURRENT_SENSE_ENABLE`** (Default: `y`): Enables continuous ADC1/DMA acquisition of both BTS7960 sense outputs.
*   **`CONFIG_ENGINE_CURRENT_SENSE_GPIO_R_IS`** (Default: `4`): Conditioned ADC input for clockwise current.
*   **`CONFIG_ENGINE_CURRENT_SENSE_GPIO_L_IS`** (Default: `5`): Independent conditioned ADC input for counter-clockwise current.
*   **`CONFIG_ENGINE_CURRENT_SENSE_SAMPLE_HZ`** (Default: `25000`): Conversion rate per channel; each 1 ms frame has 25 samples from each input and reports mean and median.
*   Board, series, and pulldown resistance plus nominal `k_ILIS` are configurable.

The measured board has 10 kΩ from each sense output to ground. Each input uses
its own 10 kΩ series resistor, 1 kΩ ADC-node pulldown, 100 nF capacitor and
external Schottky clamps to 3.3 V/GND. Do not tie `R_IS` and `L_IS` together.
Captured current follows the effective bridge state: `R_IS` for positive drive
and negated `L_IS` for negative drive. Per-input faults, `BRAKE`, `COAST`, and
unavailability use reserved codes in the same `int16_t`, so the capture keeps
five channels and 13,107 samples.
Read the component documentation before connecting either GPIO.

### Control Time-Series Capture

The reusable `esp_timeseries_recorder` component reserves one contiguous
internal-DRAM buffer and stores sample-major, interleaved `int16_t` records.
`CONFIG_ESP_TIMESERIES_RECORDER_BUFFER_KIB` selects its size (128 KiB by
default); runtime channel count determines the sample capacity.

This application records speed, current, control action, reference, and raw
sensor angle. The Octave console arms captures on demand. The rate
belongs to each `ARM` operation, must divide 1 kHz exactly,
and can also be selected with the USB command `ARM <rate_hz>`. Five channels in
128 KiB hold 13,107 samples: 26.214 s at 500 Hz or 52.428 s at 250 Hz.

The current diagnostic build enables a host-requested open-loop profile.
The motor remains in COAST until `CAL START`, then runs 40%, 55%, and 40% duty
for 8 s each before returning to COAST. A regular `ARM` starts a fresh
closed-loop 600/900 RPM experiment. These values are configurable in Kconfig.
The duration lets a 500 Hz calibration include all plateaus before the buffer
fills.

The reusable `esp_angle_lut` component corrects a cyclic native-count angle
before the Kalman estimator. Its configurable full scale supports power-of-two
sensor resolutions from 8 through 16 bits; this application uses 14 bits for
the MT6701. Its 256 signed-count entries are linearly interpolated in the 3 kHz
path and stored in two CRC-protected NVS slots. The Octave console can
run the calibration capture, fit and validate the LUT, upload it as binary,
verify a readback, and then enable it. The recorded `angle_raw` channel always
remains uncorrected so a later calibration never learns the previous LUT.

A full buffer stays immutable until `CLEAR`. Metadata, addresses, scaling,
saturation/invalid counters, and a stable payload view are exposed to the
separate USB transport component. The native USB Serial/JTAG port accepts
`PING`, `INFO`, `STATUS`, `ARM`, `DUMP`, `CLEAR`, `CONTROL ...`, `CAL ...`, and `HELP`. `DUMP` returns a
self-describing text header followed by little-endian binary samples protected
by CRC-32/IEEE. See `tools/octave/README.md` for the receiver and plotting
workflow. UART0 remains the firmware log/flash port so logs cannot enter the
binary stream. Run `ts_console()` in Octave for the guided port selection and
recorder menus. Capture is deliberately host-driven: an experiment starts only
after `ARM`, and a full buffer remains immutable until `CLEAR`.
The Octave client reports acquisition and USB-transfer times separately; native
USB CDC does not use the nominal serial baud rate.

---

## ⚡ High-Speed Optimizations & Timing Accuracy

To support high rotational speeds (such as 30,000 RPM or more) and ensure maximum estimation precision, the firmware implements the following optimized behaviors:

### 1. Software-Driven Zero & Direction Calibration
*   Instead of burning configurations to the MT6701's hardware EEPROM (which requires a 5V VDD supply and introduces write blocking delays), calibration offsets and rotation direction CCW/CW are processed mathematically in software. 
*   This makes the I2C interface **read-only** during execution, ensuring compatibility with 3.3V power rails and zero risk of settings corruption.

### 2. High-Speed 2-Byte I2C Burst Reads
*   On each 333.333 µs estimator cycle, the application performs one synchronous **2-byte I2C burst read** of registers `0x03` and `0x04`. The transaction completes before the Kalman update, so the estimator always uses the newest sample without a pipeline projection.
*   The combined transaction requires approximately 45 SCL pulses, corresponding to a theoretical minimum of **~45 µs** at a 1 MHz clock.
*   The datasheet specifies a 1 µs minimum SCL period, allowing 1 MHz, provided SDA/SCL rise and fall times remain below 150 ns. Suitable external pull-ups and short connections are recommended; 400 kHz remains available as a conservative `menuconfig` fallback.

### 3. Dynamic Time Delta (`dt`) Measurement
*   The MT6701 driver timestamps each completed synchronous acquisition. The Kalman filter receives the actual interval between those timestamps rather than an ideal fixed period.
*   The PID uses the current Kalman state directly. No future projection is applied to angle, speed, or acceleration.

### 4. Dual-Rate Scheduling
*   A **1 MHz GPTimer** uses a repeating **333/333/334 us** sequence of absolute alarms. The three intervals total exactly 1 ms, yielding an exact average rate of 3 kHz. The ISR rearms the next alarm and directly notifies the real-time task; no I2C transaction or Kalman operation runs inside the interrupt.
*   The MT6701 and the complete Kalman state (position, velocity, and acceleration) are updated at **3 kHz**.
*   Every three samples, the PID calculates and applies a signed command limited to **-100% through 100%**, resulting in an exact **1 kHz** control rate. Positive commands drive forward, negative commands apply reverse torque, and an exact zero selects dynamic braking. For tuning experiments, the reference alternates between **600 and 900 RPM** every 2 seconds by default. Before `ARM`, Octave can read or replace volatile `Kp`, `Ki`, `Kd`, and step-period values; reset restores the compiled defaults.
*   A low-priority task on Core 0 receives telemetry every **5 seconds**. It preserves the window affected by the previous serial output and reports it beside the following quiet window, labeling them `log-affected` and `quiet`. The test configuration emits one compact diagnostic line and one compact state line per window. Reports appear every **10 seconds** without hiding the instrumentation's own impact. No log formatting or output runs on Core 1 after GPTimer starts.

### 5. Real-Time Loop Isolation
*   The complete acquisition, estimation, and control task is created with `xTaskCreatePinnedToCore()` on **Core 1** at priority `configMAX_PRIORITIES - 1`.
*   The I2C bus, MT6701, and GPTimer are initialized inside that task. Their peripheral interrupts are therefore allocated from Core 1, avoiding task migration and cross-core traffic in the critical path.
*   `app_main`, `esp_timer` services, and the telemetry task remain on **Core 0**.
*   The ESP32-S3 runs at **240 MHz**, the firmware uses performance optimization, and driver mutexes are compiled out because each peripheral has a single owner.
*   The FreeRTOS tick remains at **1 kHz**: the 3 kHz timing comes from GPTimer and does not require raising the global scheduler tick rate.

### 6. Windowed Timing Diagnostics
*   Telemetry crosses to Core 0 only once every 5 seconds instead of updating a queue on every control cycle. Quiet and log-affected windows are displayed separately.
*   Each window reports effective rates, missed notifications, I2C errors, 334 µs deadline overruns, and minimum/maximum `dt` values. Failures include both window and lifetime totals.
*   Stage averages, maxima, and configured-budget violation counts reset every window. A separate lifetime processing maximum remains available only as a reference.

---

## 📈 Kalman Filter (3D)

The project integrates the pure C Kalman Filter library from [kalman-filter-c](https://github.com/smartsensingme/kalman-filter-c.git) to estimate position ($\theta$), velocity ($\omega$), and acceleration ($\alpha$).

### Angular Transition Correction (Wrap-around)
Due to the circular behavior of the encoder ($0^\circ \to 360^\circ$), the [engine_angle_kalman.c](main/engine_angle_kalman.c) module implements the specialized function `engine_angle_kalman_3d_update` to normalize the measurement error (innovation) to the range of $[-180^\circ, 180^\circ]$ to prevent false spikes when transitioning the physical boundary.

### High-Precision MT6701 Tuning
The current diagnostic measurement covariance is **`0.002f`** (standard
deviation about $0.0447^\circ$). It was increased from `0.0004f` to test
additional velocity smoothing; captures showed only a small reduction in the
rotation-synchronous ripple.

---

## 🚀 How to Compile and Run

1.  **Clone the project and its dependencies:**
    This repository uses Git submodules. Clone it recursively:
    ```bash
    git clone --recursive https://github.com/smartsensingme/esp-mt6701-example.git
    ```
    If you have already cloned the project without submodules, fetch the dependencies by running:
    ```bash
    git submodule update --init --recursive
    ```

2.  **Configure ESP-IDF Environment:**
    Activate ESP-IDF (adjust path to your installation):
    ```bash
    . ~/.espressif/v6.0/esp-idf/export.sh
    ```

3.  **Configure Target and Pinout:**
    ```bash
    idf.py set-target esp32s3
    idf.py menuconfig
    ```
    *(Modify I2C pins, H-bridge MCPWM pins, frequency, and toggle thread-safety options as desired).*

4.  **Compile the Project:**
    ```bash
    idf.py build
    ```

5.  **Flash and Monitor:**
    ```bash
    idf.py flash monitor
    ```

---

## 📦 How to Reuse these Drivers in Another Project

Since the drivers were developed as clean, decoupled ESP-IDF components, you can add them directly to another project:
1. Add the MT6701 driver:
   ```bash
   git submodule add https://github.com/smartsensingme/esp-mt6701.git components/esp-mt6701
   ```
2. Add the H-Bridge engine driver:
   ```bash
   git submodule add https://github.com/smartsensingme/esp-engine-driver-.git components/esp-engine-driver
   ```
3. Include them in your application code:
   ```c
   #include "mt6701.h"
   #include "engine_driver.h"
   ```
4. All configurations (pins, frequency, thread-safety) will automatically show up in your project's `menuconfig`!

---
![SmartSensing.me Logo](https://smartsensing.me/ssme-logo.png)

## 📝 Description

This project is part of the **SmartSensing.me** ecosystem. We apply real fundamentals of instrumentation engineering and high-performance embedded systems.

Unlike superficial, clickbait content, this repository delivers:
- **Originality:** Unique implementations based on nearly 30 years of academic experience.
- **Technical Depth:** Professional usage of the ESP-IDF framework and FreeRTOS.
- **Pedagogy:** Documented and structured code for those seeking genuine technical growth.

> "We transform signals from the physical world into digital intelligence, with no shortcuts."

---

## 👤 About the Author

**José Alexandre de França** *Associate Professor at the Department of Electrical Engineering of UEL*

Electrical Engineer with nearly three decades of experience in undergraduate and postgraduate teaching. PhD in Electrical Engineering, researcher in electronic instrumentation, and embedded systems developer. SmartSensing.me is my commitment to raising the bar of technology education in Brazil.

- 🌐 **Website:** [smartsensing.me](https://smartsensing.me)
- 📧 **E-mail:** [info@smartsensing.me](mailto:info@smartsensing.me)
- 📺 **YouTube:** [@smartsensingme](https://youtube.com/@smartsensingme)
- 📸 **Instagram:** [@smartsensing.me](https://instagram.com/smartsensing.me)

---

## 📄 License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.
