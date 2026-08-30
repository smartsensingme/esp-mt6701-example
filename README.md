# MT6701 Magnetic Encoder, Motor H-Bridge Control & Kalman Filter - ESP-IDF v6

*Read in other languages: [Português](README.pt-br.md)*

This repository contains the demonstration of the integration of the **MT6701** 14-bit magnetic encoder and the **BTS7960 (IBT-2)** H-bridge motor driver with **ESP-IDF v6**, utilizing standalone, reusable ESP-IDF components and an advanced **3D Kalman Filter** to estimate the position, speed (RPM), and acceleration (RPM/s) of a rotating shaft in real time.

The project is configured to run on the **ESP32-S3** microcontroller and consumes its external dependencies through Git submodules.

---

## 🛠️ Project Architecture

The workspace is organized as follows:
- **`components/esp-mt6701`**: Git submodule for the MT6701 14-bit magnetic encoder driver, utilizing the modern ESP-IDF master I2C driver (`driver/i2c_master.h`) and optimized to run strictly in read-only mode (software-driven offset and direction).
- **`components/esp-engine-driver`**: Git submodule for the BTS7960 dual PWM H-bridge motor driver, utilizing the ESP32-S3's native high-performance **MCPWM** peripheral.
- **`components/kalman-filter-c`**: Git submodule pointing to the pure C Kalman Filter library.
- **`main/`**: Real-time application that reads the MT6701 and updates the Kalman filter at **4 kHz**, runs a temporary controller at **1 kHz**, drives the H-bridge, and publishes telemetry every 5 seconds.

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

---

## ⚡ High-Speed Optimizations & Timing Accuracy

To support high rotational speeds (such as 30,000 RPM or more) and ensure maximum estimation precision, the firmware implements the following optimized behaviors:

### 1. Software-Driven Zero & Direction Calibration
*   Instead of burning configurations to the MT6701's hardware EEPROM (which requires a 5V VDD supply and introduces write blocking delays), calibration offsets and rotation direction CCW/CW are processed mathematically in software. 
*   This makes the I2C interface **read-only** during execution, ensuring compatibility with 3.3V power rails and zero risk of settings corruption.

### 2. High-Speed 2-Byte I2C Burst Reads
*   To minimize bus transaction time, the 4 kHz estimator task performs a single, continuous **2-byte I2C read** of registers `0x03` and `0x04` to retrieve the full 14-bit angle.
*   The combined transaction requires approximately 45 SCL pulses, corresponding to a theoretical minimum of **~45 µs** at a 1 MHz clock.
*   The datasheet specifies a 1 µs minimum SCL period, allowing 1 MHz, provided SDA/SCL rise and fall times remain below 150 ns. Suitable external pull-ups and short connections are recommended; 400 kHz remains available as a conservative `menuconfig` fallback.

### 3. Dynamic Time Delta (`dt`) Measurement
*   Instead of assuming an ideal `0.00025s` (250 µs) period, the task measures the actual time between samples using **`esp_timer_get_time()`**.
*   This actual time delta (`dt`) is passed directly to the Kalman Filter.
*   This reduces errors caused by scheduler jitter, task preemption, and I2C transaction latency.

### 4. Dual-Rate Scheduling
*   A **GPTimer** generates an interrupt every 250 µs. The ISR only sends a direct notification to the real-time task; no I2C transaction or Kalman operation runs inside the interrupt.
*   The MT6701 and the complete Kalman state (position, velocity, and acceleration) are updated at **4 kHz**.
*   Every four samples, the temporary controller applies a fixed **50%** command to the H-bridge, resulting in a **1 kHz** control rate. Its API already receives speed and `dt` for the future PID implementation.
*   A low-priority task on Core 0 receives telemetry every **5 seconds**. To measure the serial output's own interference, it prints one silent window and silently discards the following window contaminated by that output. This produces one trustworthy report every **10 seconds**. No log formatting or output runs on Core 1 after GPTimer starts.

### 5. Real-Time Loop Isolation
*   The complete acquisition, estimation, and control task is created with `xTaskCreatePinnedToCore()` on **Core 1** at priority `configMAX_PRIORITIES - 1`.
*   The I2C bus, MT6701, and GPTimer are initialized inside that task. Their peripheral interrupts are therefore allocated from Core 1, avoiding task migration and cross-core traffic in the critical path.
*   `app_main`, `esp_timer` services, and the telemetry task remain on **Core 0**.
*   The ESP32-S3 runs at **240 MHz**, the firmware uses performance optimization, and driver mutexes are compiled out because each peripheral has a single owner.
*   The FreeRTOS tick remains at **1 kHz**: the 4 kHz timing comes from GPTimer and does not require raising the global scheduler tick rate.

### 6. Windowed Timing Diagnostics
*   Telemetry crosses to Core 0 only once every 5 seconds instead of updating a queue on every control cycle. Only windows containing no log output are displayed.
*   Each window reports effective rates, missed notifications, I2C errors, 250 µs deadline overruns, and minimum/maximum `dt` values.
*   Maximum wake latency, I2C transaction, Kalman, control, and total processing times reset every window. A separate lifetime maximum remains available only as a reference.

---

## 📈 Kalman Filter (3D)

The project integrates the pure C Kalman Filter library from [kalman-filter-c](https://github.com/smartsensingme/kalman-filter-c.git) to estimate position ($\theta$), velocity ($\omega$), and acceleration ($\alpha$).

### Angular Transition Correction (Wrap-around)
Due to the circular behavior of the encoder ($0^\circ \to 360^\circ$), the [engine_angle_kalman.c](main/engine_angle_kalman.c) module implements the specialized function `engine_angle_kalman_3d_update` to normalize the measurement error (innovation) to the range of $[-180^\circ, 180^\circ]$ to prevent false spikes when transitioning the physical boundary.

### High-Precision MT6701 Tuning
The measurement noise covariance `.r` parameter in the Kalman Filter config is tuned to **`0.0004f`** (equivalent to a standard deviation of $0.02^\circ$), matching the low transition noise ($0.01^\circ$ RMS typical) of the MT6701. This allows the filter to trust sensor measurements significantly more than it would with an AS5600, minimizing lagging issues while providing smooth velocity outputs.

---

## 🚀 How to Compile and Run

1.  **Clone the project and its dependencies:**
    This repository uses Git submodules. Clone it recursively:
    ```bash
    git clone --recursive git@github-ssme:smartsensingme/esp-mt6701-example-esp-idf.git
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
   git submodule add git@github-ssme:smartsensingme/esp-engine-driver-.git components/esp-engine-driver
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
