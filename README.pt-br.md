# Sensor Magnético AS5600 & Filtro de Kalman - ESP-IDF v6

*Read in other languages: [English](README.md)*

Este repositório contém a demonstração migrada da integração do encoder magnético **AS5600** com o **ESP-IDF v6**, utilizando um componente de sensor nativo e reutilizável para ESP-IDF, e um **Filtro de Kalman 3D** avançado para estimar a posição, velocidade (RPM) e aceleração (RPM/s) de um eixo giratório em tempo real.

O projeto está configurado para rodar no microcontrolador **ESP32-S3** e consome suas dependências externas através de subpódulos do Git.

---

## 🛠️ Arquitetura do Projeto

O espaço de trabalho está estruturado da seguinte forma:
- **`components/as5600`**: Componente de driver independente do sensor AS5600, utilizando o driver moderno I2C Master do ESP-IDF (`driver/i2c_master.h`).
- **`components/kalman-filter-c`**: Submódulo Git apontando para a biblioteca pura em C do Filtro de Kalman.
- **`main/`**: Código da aplicação que inicializa o barramento I2C, configura o sensor AS5600 e executa o loop de estimativa a 1 kHz.

---

## ⚙️ Configurações do Projeto

### Segurança de Threads (Configuração do Componente)
O componente `as5600` inclui um arquivo `Kconfig` que expõe uma flag de compilação:
*   **`CONFIG_AS5600_THREAD_SAFE`** (Padrão: `y`):
    *   *Marcado:* Utiliza um Mutex do FreeRTOS para sincronizar a comunicação ao chamar o driver a partir de múltiplas tarefas.
    *   *Desmarcado:* Remove todos os bloqueios de mutex da compilação para fornecer velocidades máximas de transação I2C, sem travas (lock-free) e com overhead zero.

### Configuração de Pinos GPIO I2C (Configuração da Aplicação)
Você pode configurar os pinos GPIO para as linhas SCL e SDA diretamente usando o `menuconfig`. Os padrões configurados em `main/Kconfig.projbuild` são:
*   **`CONFIG_APP_I2C_SDA_PIN`** (Padrão: `8`)
*   **`CONFIG_APP_I2C_SCL_PIN`** (Padrão: `9`)

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
    *(Vá em `Application I2C Configurations` para modificar os pinos I2C, ou em `AS5600 Driver Configuration` para alternar a segurança de threads).*

4.  **Compile o Projeto:**
    ```bash
    idf.py build
    ```

5.  **Grave o Firmware e Monitore:**
    ```bash
    idf.py flash monitor
    ```

---

## 📦 Como Reutilizar este Driver em Outro Projeto

Como o driver foi desenvolvido como um componente ESP-IDF limpo e desacoplado, você pode adicioná-lo diretamente a outro projeto:
1. Adicione-o como submódulo Git na pasta `components/as5600` do seu novo projeto:
   ```bash
   git submodule add git@github-ssme:smartsensingme/as5600-esp-idf-component.git components/as5600
   ```
2. No código da sua aplicação, inclua-o:
   ```c
   #include "as5600.h"
   ```
3. As configurações do componente (como a segurança de threads) aparecerão automaticamente no `menuconfig` do seu projeto!

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
