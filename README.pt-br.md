# Sensor Magnético MT6701, Controle de Motor por Ponte H & Filtro de Kalman - ESP-IDF v6

*Read in other languages: [English](README.md)*

Este repositório contém a demonstração da integração do encoder magnético **MT6701** de 14 bits e do driver de motor por ponte H **BTS7960 (IBT-2)** com o **ESP-IDF v6**, utilizando componentes nativos e reutilizáveis para ESP-IDF, e um **Filtro de Kalman 3D** avançado para estimar a posição, velocidade (RPM) e aceleração (RPM/s) de um eixo giratório em tempo real.

O projeto está configurado para rodar no microcontrolador **ESP32-S3** e consome suas dependências externas através de submódulos do Git.

---

## 🛠️ Arquitetura do Projeto

O espaço de trabalho está estruturado da seguinte forma:
- **`components/esp-mt6701`**: Submódulo Git para o driver do sensor MT6701, utilizando o driver moderno I2C Master do ESP-IDF (`driver/i2c_master.h`) e otimizado para rodar estritamente como somente leitura (calibrações de offset e direção resolvidas em software).
- **`components/esp-engine-driver`**: Submódulo Git para o driver de motor por ponte H BTS7960, utilizando o periférico de alta performance **MCPWM** nativo do ESP32-S3.
- **`components/kalman-filter-c`**: Submódulo Git apontando para a biblioteca pura em C do Filtro de Kalman.
- **`main/`**: Código da aplicação que inicializa o barramento I2C, configura o sensor MT6701, inicializa o driver da ponte H (aplicando 50% de PWM Horário para teste) e executa o loop de estimativa a 1 kHz.

---

## ⚙️ Configurações do Projeto

### Segurança de Threads (Configurações dos Componentes)
Ambos os drivers incluem flags do Kconfig para alternar a sincronização por Mutex do FreeRTOS em tempo de compilação:
*   **`CONFIG_MT6701_THREAD_SAFE`** (Padrão: `y`): Sincroniza os acessos aos registradores via barramento I2C.
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

## ⚡ Otimizações de Alta Velocidade & Precisão de Tempo

Para suportar altas velocidades de rotação (como 30.000 RPM ou mais) e garantir a máxima precisão de estimativa, o firmware implementa os seguintes comportamentos otimizados:

### 1. Calibração de Direção e Zero por Software
*   Em vez de gravar configurações na EEPROM física do MT6701 (que requer alimentação de 5.0V VDD e introduz delays longos de travamento de escrita), os offsets de calibração de zero e a direção lógica CW/CCW são calculados matematicamente em software.
*   Isso torna a interface I2C **somente leitura** durante o funcionamento do sistema, assegurando compatibilidade com tensões de 3.3V no barramento e eliminando riscos de corromper a EEPROM física.

### 2. Leitura I2C de Alta Velocidade (2 Bytes em Burst)
*   Para reduzir ao mínimo a transação no barramento, o loop principal a 1 kHz executa uma única leitura em lote de **2 bytes** para obter o ângulo completo de 14 bits dos registradores `0x03` e `0x04`.
*   Isso reduz a duração da transação I2C para apenas **~73 µs** (com clock de 400 kHz) ou **~29 µs** (com clock de 1 MHz), permitindo taxas de amostragem extremamente altas.

### 3. Medição Dinâmica do Delta de Tempo (`dt`)
*   Em vez de assumir um tempo de ciclo fixo e ideal de `0.001s` (1 ms), o loop mede o tempo real decorrido entre as iterações usando a função **`esp_timer_get_time()`**.
*   Esse delta de tempo real (`dt`) é passado diretamente para o Filtro de Kalman.
*   Isso garante uma estimativa de velocidade (RPM) e aceleração (RPM/s) 100% precisa, compensando integralmente jitters do scheduler, preempção de tarefas e latências do barramento I2C.

---

## 📈 Filtro de Kalman (3D)

O projeto integra a biblioteca pura em C do Filtro de Kalman de [kalman-filter-c](https://github.com/smartsensingme/kalman-filter-c.git) para estimar posição ($\theta$), velocidade ($\omega$) e aceleração ($\alpha$).

### Correção de Transição Angular (Wrap-around)
Devido ao comportamento circular do encoder ($0^\circ \to 360^\circ$), o loop principal ([main.c](main/main.c)) implementa a função especializada `engine_angle_kalman_3d_update` para normalizar o erro de medição (inovação) no intervalo de $[-180^\circ, 180^\circ]$ a fim de evitar picos falsos ao cruzar a borda física.

### Ajuste de Alta Precisão para o MT6701
A covariância do ruído de medição `.r` no Filtro de Kalman foi ajustada para **`0.0004f`** (desvio padrão de $0.02^\circ$). Como o MT6701 possui um ruído de transição típico baixíssimo de apenas $0.01^\circ$ RMS, essa sintonia faz o filtro confiar de forma muito mais agressiva no sensor (comparado ao ruído do AS5600), minimizando atrasos de fase e entregando velocidades dinâmicas muito mais realistas.

---

## 🚀 Como Compilar e Executar

1.  **Clone o projeto e suas dependências:**
    Este repositório utiliza submódulos do Git. Clone de forma recursiva:
    ```bash
    git clone --recursive git@github-ssme:smartsensingme/esp-mt6701-example-esp-idf.git
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
1. Adicione o driver do sensor MT6701:
   ```bash
   git submodule add https://github.com/smartsensingme/esp-mt6701.git components/esp-mt6701
   ```
2. Adicione o driver do motor:
   ```bash
   git submodule add git@github-ssme:smartsensingme/esp-engine-driver-.git components/esp-engine-driver
   ```
3. No código da sua aplicação, inclua-os:
   ```c
   #include "mt6701.h"
   #include "engine_driver.h"
   ```
4. As configurações dos componentes aparecerão automaticamente no `menuconfig` do seu novo projeto!

---
![Logo SmartSensing.me](https://smartsensing.me/ssme-logo.png)

## 📝 Descrição

Este projeto é parte do ecossistema **SmartSensing.me**. Aplicamos fundamentos reais de engenharia de instrumentação e sistemas embarcados de alta performance.

Diferente de conteúdos superficiais ou clickbaits, este repositório oferece:
- **Originalidade:** Implementações únicas baseadas em quase 30 anos de experiência acadêmica.
- **Profundidade Técnico:** Uso profissional do framework ESP-IDF e FreeRTOS.
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

Este projeto é licensed sob a Licença MIT. Veja o arquivo [LICENSE](LICENSE) para detalhes.
