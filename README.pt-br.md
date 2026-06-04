# Sensor Magnético AS5600, Controle de Motor por Ponte H & Filtro de Kalman - ESP-IDF v6

*Read in other languages: [English](README.md)*

Este repositório contém a demonstração da integração do encoder magnético **AS5600** e do driver de motor por ponte H **BTS7960 (IBT-2)** com o **ESP-IDF v6**, utilizando componentes nativos e reutilizáveis para ESP-IDF, e um **Filtro de Kalman 3D** avançado para estimar a posição, velocidade (RPM) e aceleração (RPM/s) de um eixo giratório em tempo real.

O projeto está configurado para rodar no microcontrolador **ESP32-S3** e consome suas dependências externas através de submódulos do Git.

---

## 🛠️ Arquitetura do Projeto

O espaço de trabalho está estruturado da seguinte forma:
- **`components/as5600`**: Submódulo Git para o driver do sensor AS5600, utilizando o driver moderno I2C Master do ESP-IDF (`driver/i2c_master.h`).
- **`components/esp-engine-driver`**: Submódulo Git para o driver de motor por ponte H BTS7960, utilizando o periférico de alta performance **MCPWM** nativo do ESP32-S3.
- **`components/kalman-filter-c`**: Submódulo Git apontando para a biblioteca pura em C do Filtro de Kalman.
- **`main/`**: Código da aplicação que inicializa o barramento I2C, configura o sensor AS5600, inicializa o driver da ponte H (aplicando 50% de PWM Horário para teste) e executa o loop de estimativa a 1 kHz.

---

## ⚙️ Configurações do Projeto

### Segurança de Threads (Configurações dos Componentes)
Ambos os drivers incluem flags do Kconfig para alternar a sincronização por Mutex do FreeRTOS em tempo de compilação:
*   **`CONFIG_AS5600_THREAD_SAFE`** (Padrão: `y`): Sincroniza os acessos aos registradores via barramento I2C.
*   **`CONFIG_ENGINE_THREAD_SAFE`** (Padrão: `y`): Sincroniza a rotina de escrita de velocidade do motor.
*   *Nota:* Se desmarcados, todas as operações de mutex correspondentes são compiladas fora para fornecer transações livres de travas (lock-free) e com overhead zero.

### Configuração de Pinos GPIO I2C (Configuração da Aplicação)
Configurável diretamente via `menuconfig`. Padrões:
*   **`CONFIG_APP_I2C_SDA_PIN`** (Padrão: `8`)
*   **`CONFIG_APP_I2C_SCL_PIN`** (Padrão: `9`)

### Configuração do Driver do Motor / Ponte H (Configuração do Componente)
Expõe as seguintes opções no Kconfig para controle do BTS7960:
*   **`CONFIG_ENGINE_PWM_FREQ_HZ`** (Padrão: `20000` / 20 kHz): Frequência do PWM físico. Sob o limite de 80 MHz do clock do timer do MCPWM no ESP32-S3, essa frequência fornece exatamente **4000 passos de resolução**.
*   **`CONFIG_ENGINE_PIN_RPWM`** (Padrão: `1`): GPIO para o sinal PWM Horário (RPWM).
*   **`CONFIG_ENGINE_PIN_LPWM`** (Padrão: `2`): GPIO para o sinal PWM Anti-horário (LPWM).
*   **`CONFIG_ENGINE_PIN_ENABLE`** (Padrão: `3`): GPIO de Enable para R_EN/L_EN interligados.

---

## 📈 Filtro de Kalman (3D)

O projeto integra a biblioteca pura em C do Filtro de Kalman de [kalman-filter-c](https://github.com/smartsensingme/kalman-filter-c.git) para estimar posição ($\theta$), velocidade ($\omega$) e aceleração ($\alpha$).

### Correção de Transição Angular (Wrap-around)
Devido ao comportamento circular do encoder ($0^\circ \to 360^\circ$), o loop principal ([main.c](main/main.c)) implementa a função especializada `engine_angle_kalman_3d_update` para normalizar o erro de medição (inovação) no intervalo de $[-180^\circ, 180^\circ]$ a fim de evitar picos falsos ao cruzar a borda física.

---

## 🚀 Como Compilar e Executar

1.  **Clone o projeto e suas dependências:**
    Este repositório utiliza submódulos do Git. Clone de forma recursiva:
    ```bash
    git clone --recursive git@github-ssme:smartsensingme/as5600-sensor-esp-idf.git
    ```
    Se você já clonou o projeto sem os submódulos, baixe as dependências executando:
    ```bash
    git submodule update --init --recursive
    ```

2.  **Configure o Ambiente do ESP-IDF:**
    Ative o ESP-IDF (ajuste o caminho para a sua instalação):
    ```bash
    . ~/.espressif/v6.0/esp-idf/export.sh
    ```

3.  **Configure o Target e a Pinagem:**
    ```bash
    idf.py set-target esp32s3
    idf.py menuconfig
    ```
    *(Ajuste os pinos de I2C, pinos de MCPWM e frequência, e configure as flags de thread-safety de acordo com sua necessidade).*

4.  **Compile o Projeto:**
    ```bash
    idf.py build
    ```

5.  **Grave o Firmware e Monitore:**
    ```bash
    idf.py flash monitor
    ```

---

## 📦 Como Reutilizar estes Drivers em Outro Projeto

Como os drivers foram desenvolvidos como componentes ESP-IDF limpos e desacoplados, você pode adicioná-los diretamente a outro projeto:
1. Adicione o driver do sensor AS5600:
   ```bash
   git submodule add git@github-ssme:smartsensingme/as5600-esp-idf-component.git components/as5600
   ```
2. Adicione o driver do motor:
   ```bash
   git submodule add git@github-ssme:smartsensingme/esp-engine-driver-.git components/esp-engine-driver
   ```
3. No código da sua aplicação, inclua-os:
   ```c
   #include "as5600.h"
   #include "engine_driver.h"
   ```
4. As configurações dos componentes aparecerão automaticamente no `menuconfig` do seu novo projeto!

---
![Logo SmartSensing.me](https://smartsensing.me/ssme-logo.png)

## 📝 Descrição

Este projeto é parte do ecossistema **SmartSensing.me**. Aplicamos fundamentos reais de engenharia de instrumentação e sistemas embarcados de alta performance.

Diferente de conteúdos superficiais ou clickbaits, este repositório oferece:
- **Originalidade:** Implementações únicas baseadas em quase 30 anos de experiência acadêmica.
- **Profundidade Técnica:** Uso profissional do framework ESP-IDF e FreeRTOS.
- **Pedagogia:** Código documentado e estruturado para quem busca crescimento técnico genuíno.

> "Transformamos sinais do mundo físico em inteligência digital, sem atalhos."

---

## 👤 Sobre o Autor

**José Alexandre de França** *Professor Associado do Departamento de Engenharia Elétrica da UEL*

Engenheiro Eletricista com quase três décadas de experiência na docência de graduação e pós-graduação. Doutor em Engenharia Elétrica, pesquisador em instrumentação eletrônica e desenvolvedor de sistemas embarcados. SmartSensing.me é meu compromisso para elevar a barra do ensino tecnológico no Brasil.

- 🌐 **Website:** [smartsensing.me](https://smartsensing.me)
- 📧 **E-mail:** [info@smartsensing.me](mailto:info@smartsensing.me)
- 📺 **YouTube:** [@smartsensingme](https://youtube.com/@smartsensingme)
- 📸 **Instagram:** [@smartsensing.me](https://instagram.com/smartsensing.me)

---

## 📄 Licença

Este projeto está licenciado sob a Licença MIT. Veja o arquivo [LICENSE](LICENSE) para detalhes.
