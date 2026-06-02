# AS5600 Magnetic Encoder & Kalman Filter - ESP-IDF v6

*Read in other languages: [Português](README.pt-br.md)*

This repository contains the ported demonstration of the integration of the **AS5600** magnetic encoder with **ESP-IDF v6**, utilizing a standalone, reusable ESP-IDF sensor component and an advanced **3D Kalman Filter** to estimate the position, speed (RPM), and acceleration (RPM/s) of a rotating shaft in real time.

The project is configured to run on the **ESP32-S3** microcontroller and consumes its external dependencies through Git submodules.

---

## 🛠️ Project Architecture

The workspace is organized as follows:
- **`components/as5600`**: Standalone driver component for the AS5600 sensor, utilizing the modern ESP-IDF master I2C driver (`driver/i2c_master.h`).
- **`components/kalman-filter-c`**: Git submodule pointing to the pure C Kalman Filter library.
- **`main/`**: Application code that initializes the I2C master bus, configures the AS5600 sensor, and runs the 1 kHz estimation loop.

---

## ⚙️ Project Configurations

### Thread Safety (Component Config)
The `as5600` component includes a `Kconfig` file that exposes a compilation flag:
*   **`CONFIG_AS5600_THREAD_SAFE`** (Default: `y`):
    *   *Checked:* Uses a FreeRTOS Mutex to synchronize communication when calling the driver from multiple tasks.
    *   *Unchecked:* Removes all mutex locks from compilation to provide maximum, lock-free, zero-overhead I2C transaction speeds.

### I2C GPIO Pin Configuration (Application Config)
You can configure the GPIO pins for the SCL and SDA lines directly using `menuconfig`. The defaults configured in `main/Kconfig.projbuild` are:
*   **`CONFIG_APP_I2C_SDA_PIN`** (Default: `8`)
*   **`CONFIG_APP_I2C_SCL_PIN`** (Default: `9`)

---

## 📈 Kalman Filter (3D)

The project integrates the pure C Kalman Filter library from [kalman-filter-c](https://github.com/smartsensingme/kalman-filter-c.git) to estimate position ($\theta$), velocity ($\omega$), and acceleration ($\alpha$).

### Angular Transition Correction (Wrap-around)
Due to the circular behavior of the encoder ($0^\circ \to 360^\circ$), the main loop ([main.c](main/main.c)) implements the specialized function `engine_angle_kalman_3d_update` to normalize the measurement error (innovation) to the range of $[-180^\circ, 180^\circ]$ to prevent false spikes when transitioning the physical boundary.

---

## 🚀 How to Compile and Run

1.  **Clone the project and its dependencies:**
    This repository uses Git submodules. Clone it recursively:
    ```bash
    git clone --recursive git@github-ssme:smartsensingme/as5600-sensor-esp-idf.git
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
    *(Go to `Application I2C Configurations` to modify I2C pins, or `AS5600 Driver Configuration` to toggle thread safety).*

4.  **Compile the Project:**
    ```bash
    idf.py build
    ```

5.  **Flash and Monitor:**
    ```bash
    idf.py flash monitor
    ```

---

## 📦 How to Reuse this Driver in Another Project

Since the driver was developed as a clean, decoupled ESP-IDF component, you can add it directly to another project:
1. Add it as a Git submodule in your project's `components/as5600` directory:
   ```bash
   git submodule add git@github-ssme:smartsensingme/as5600-esp-idf-component.git components/as5600
   ```
2. In your application code, include it:
   ```c
   #include "as5600.h"
   ```
3. The component's configurations (like thread safety) will automatically show up in your project's `menuconfig`!

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
