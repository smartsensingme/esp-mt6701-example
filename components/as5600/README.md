# AS5600 Magnetic Encoder Driver - ESP-IDF Component

*Read in other languages: [Português](README.pt-br.md)*

This directory contains a clean, decoupled, and reusable component for the **AMS AS5600** magnetic rotary encoder, designed for **ESP-IDF v6** and targeting high-speed operations.

It utilizes the modern ESP-IDF master I2C driver (`driver/i2c_master.h`) and features optional thread safety.

---

## 🛠️ Features

1.  **Modern ESP-IDF Integration:** Fully compatible with ESP-IDF v5.0+ and v6.0+ master-bus architecture.
2.  **Thread Safety Toggle (Kconfig):**
    *   **`CONFIG_AS5600_THREAD_SAFE=y`** (Default): Protects physical register access using a FreeRTOS Mutex, ensuring safe concurrent accesses.
    *   **`CONFIG_AS5600_THREAD_SAFE=n`**: Compiles out all mutex operations, providing lock-free, zero-overhead register read/write routines for maximum sampling speeds.
3.  **Configurable Filters:** Allows runtime configuration of Hysteresis, Slow Filter, and Fast Filter Threshold parameters.
4.  **Optimized Burst Reads:** The `as5600_read_angles_and_status` method fetches Status, Raw Angle, and Filtered Angle registers in a single, continuous 5-byte I2C transaction, maximizing bus efficiency.

---

## ⚙️ Configuration Properties

Via `menuconfig` (`Component config` -> `AS5600 Driver Configuration`):
*   **`CONFIG_AS5600_THREAD_SAFE`**: Enable or disable Mutex protection.

---

## 🚀 How to Add to Your Project

Add this repository as a Git submodule in your ESP-IDF project's `components` directory:
```bash
git submodule add git@github-ssme:smartsensingme/as5600-esp-idf-component.git components/as5600
```
Then, update your component `CMakeLists.txt` to require it:
```cmake
idf_component_register(SRCS "main.c"
                       REQUIRES as5600)
```

---

## 📖 API Usage Example

Include the driver header:
```c
#include "as5600.h"
```

Initialize the device:
```c
// 1. Initialize your I2C master bus handle
i2c_master_bus_config_t bus_config = {
    .i2c_port = I2C_NUM_0,
    .sda_io_num = 8,
    .scl_io_num = 9,
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .flags.enable_internal_pullup = true,
};
i2c_master_bus_handle_t bus_handle;
i2c_new_master_bus(&bus_config, &bus_handle);

// 2. Add the AS5600 device to the bus
i2c_device_config_t dev_config = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = AS5600_I2C_ADDRESS,
    .scl_speed_hz = 400000,
};
i2c_master_dev_handle_t i2c_dev;
i2c_master_bus_add_device(bus_handle, &dev_config, &i2c_dev);

// 3. Initialize the driver device handle
as5600_config_t as5600_cfg = {
    .hysteresis = 0,             // 0 LSB
    .slow_filter = 2,            // 2x slow filter
    .fast_filter_threshold = 2,  // 2 LSB threshold
};
as5600_dev_t as5600_device;
ESP_ERROR_CHECK(as5600_init(&as5600_device, i2c_dev, &as5600_cfg));
```

Read raw sensor data:
```c
uint16_t raw_val;
if (as5600_read_raw_angle(&as5600_device, &raw_val) == ESP_OK) {
    float deg = ((float)raw_val * 360.0f) / 4096.0f;
    printf("Raw Angle: %.2f deg\n", deg);
}
```

---

## 🗄️ API Reference

```c
esp_err_t as5600_init(as5600_dev_t *dev, i2c_master_dev_handle_t i2c_dev, const as5600_config_t *config);
```
Initializes the driver, sets up internal structures/mutexes, and writes filter configs to the sensor.

```c
esp_err_t as5600_read_raw_angle(as5600_dev_t *dev, uint16_t *raw_angle);
```
Reads the 12-bit raw angle value directly from the CORDIC processor.

```c
esp_err_t as5600_read_angle(as5600_dev_t *dev, uint16_t *angle);
```
Reads the 12-bit filtered angle value.

```c
esp_err_t as5600_read_status(as5600_dev_t *dev, uint8_t *status);
```
Reads the Status register (`AS5600_STATUS_MD`, `AS5600_STATUS_ML`, `AS5600_STATUS_MH`).

```c
esp_err_t as5600_read_angles_and_status(as5600_dev_t *dev, uint16_t *raw_angle, uint16_t *angle, uint8_t *status);
```
Performs a fast 5-byte sequential burst read to acquire status and both angle variables in one I2C transaction.

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

