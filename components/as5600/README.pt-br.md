# Driver do Encoder Magnético AS5600 - Componente ESP-IDF

*Read in other languages: [English](README.md)*

Este diretório contém um componente desacoplado e reutilizável para o encoder magnético rotativo **AMS AS5600**, projetado para o **ESP-IDF v6** e otimizado para operações de alta velocidade.

O componente utiliza o driver moderno de I2C Master do ESP-IDF (`driver/i2c_master.h`) e conta com segurança de threads configurável.

---

## 🛠️ Funcionalidades

1.  **Integração ESP-IDF Moderna:** Totalmente compatível com a arquitetura de barramento Master do ESP-IDF v5.0+ e v6.0+.
2.  **Segurança de Threads Configurável (Kconfig):**
    *   **`CONFIG_AS5600_THREAD_SAFE=y`** (Padrão): Protege o acesso físico aos registradores utilizando um Mutex do FreeRTOS, garantindo chamadas seguras por múltiplas tarefas simultaneamente.
    *   **`CONFIG_AS5600_THREAD_SAFE=n`**: Remove todas as operações de mutex da compilação, fornecendo rotinas de leitura/escrita diretas, livre de travas (lock-free) e com overhead zero, para velocidades máximas de amostragem.
3.  **Filtros Configuráveis:** Permite a configuração em tempo de execução dos parâmetros de Histerese, Filtro Lento e Limiar do Filtro Rápido do chip.
4.  **Leituras em Bloco Otimizadas (Burst Read):** O método `as5600_read_angles_and_status` realiza a leitura sequencial de 5 bytes (Status, Ângulo Bruto e Ângulo Filtrado) em uma única transação I2C contínua, maximizando a eficiência do barramento.

---

## ⚙️ Propriedades de Configuração

Via `menuconfig` (`Component config` -> `AS5600 Driver Configuration`):
*   **`CONFIG_AS5600_THREAD_SAFE`**: Ativar ou desativar a proteção por Mutex.

---

## 🚀 Como Adicionar ao Seu Projeto

Adicione este repositório como submódulo Git no diretório `components` do seu projeto ESP-IDF:
```bash
git submodule add git@github-ssme:smartsensingme/as5600-esp-idf-component.git components/as5600
```
Em seguida, atualize o `CMakeLists.txt` do seu componente para requerê-lo:
```cmake
idf_component_register(SRCS "main.c"
                       REQUIRES as5600)
```

---

## 📖 Exemplo de Uso da API

Inclua o cabeçalho do driver:
```c
#include "as5600.h"
```

Inicialize o dispositivo:
```c
// 1. Inicialize a estrutura de configuração do barramento I2C Master
i2c_master_bus_config_t bus_config = {
    .i2c_port = I2C_NUM_0,
    .sda_io_num = 8,
    .scl_io_num = 9,
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .flags.enable_internal_pullup = true,
};
i2c_master_bus_handle_t bus_handle;
i2c_new_master_bus(&bus_config, &bus_handle);

// 2. Adicione o dispositivo AS5600 ao barramento
i2c_device_config_t dev_config = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = AS5600_I2C_ADDRESS,
    .scl_speed_hz = 400000,
};
i2c_master_dev_handle_t i2c_dev;
i2c_master_bus_add_device(bus_handle, &dev_config, &i2c_dev);

// 3. Inicialize a estrutura de controle do driver
as5600_config_t as5600_cfg = {
    .hysteresis = 0,             // 0 LSB (Sem histerese)
    .slow_filter = 2,            // Filtro lento 2x
    .fast_filter_threshold = 2,  // Limiar do filtro rápido de 2 LSB
};
as5600_dev_t as5600_device;
ESP_ERROR_CHECK(as5600_init(&as5600_device, i2c_dev, &as5600_cfg));
```

Ler dados brutos do sensor:
```c
uint16_t raw_val;
if (as5600_read_raw_angle(&as5600_device, &raw_val) == ESP_OK) {
    float deg = ((float)raw_val * 360.0f) / 4096.0f;
    printf("Angulo Bruto: %.2f graus\n", deg);
}
```

---

## 🗄️ Referência da API

```c
esp_err_t as5600_init(as5600_dev_t *dev, i2c_master_dev_handle_t i2c_dev, const as5600_config_t *config);
```
Inicializa o driver, aloca estruturas internas/mutexes, e escreve as configurações de filtros no registrador do sensor.

```c
esp_err_t as5600_read_raw_angle(as5600_dev_t *dev, uint16_t *raw_angle);
```
Lê o ângulo bruto de 12 bits diretamente do processador CORDIC do sensor.

```c
esp_err_t as5600_read_angle(as5600_dev_t *dev, uint16_t *angle);
```
Lê o ângulo de 12 bits com a filtragem de hardware aplicada.

```c
esp_err_t as5600_read_status(as5600_dev_t *dev, uint8_t *status);
```
Lê o registrador de status (`AS5600_STATUS_MD`, `AS5600_STATUS_ML`, `AS5600_STATUS_MH`).

```c
esp_err_t as5600_read_angles_and_status(as5600_dev_t *dev, uint16_t *raw_angle, uint16_t *angle, uint8_t *status);
```
Realiza uma leitura em bloco (burst read) rápida de 5 bytes sequenciais para adquirir o status e ambos os ângulos em uma única transação I2C.
