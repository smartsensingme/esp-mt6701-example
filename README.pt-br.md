# Sensor Magnético MT6701, Controle de Motor por Ponte H & Filtro de Kalman - ESP-IDF v6

*Read in other languages: [English](README.md)*

Este repositório contém a demonstração da integração do encoder magnético **MT6701** de 14 bits e do driver de motor por ponte H **BTS7960 (IBT-2)** com o **ESP-IDF v6**, utilizando componentes nativos e reutilizáveis para ESP-IDF, e um **Filtro de Kalman 3D** avançado para estimar a posição, velocidade (RPM) e aceleração (RPM/s) de um eixo giratório em tempo real.

O projeto está configurado para rodar no microcontrolador **ESP32-S3** e consome suas dependências externas através de submódulos do Git.

> **Para entender o código:** leia o [Guia do código e da arquitetura de tempo real](docs/arquitetura-tempo-real.md). Ele descreve o boot, o ciclo de 3 kHz, o controle de 1 kHz, todas as estruturas internas e cada campo dos logs.

---

## 🛠️ Arquitetura do Projeto

O espaço de trabalho está estruturado da seguinte forma:
- **`components/esp-mt6701`**: Submódulo Git para o driver do sensor MT6701, utilizando o driver moderno I2C Master do ESP-IDF (`driver/i2c_master.h`) e otimizado para rodar estritamente como somente leitura (calibrações de offset e direção resolvidas em software).
- **`components/esp-engine-driver`**: Submódulo Git para o driver da ponte H BTS7960, responsável pelo MCPWM e pela aquisição opcional de `R_IS` e `L_IS` com ADC1/DMA.
- **`components/esp_pid`**: PID reutilizável por instância, com saturação, derivada filtrada e anti-windup por back-calculation.
- **`components/kalman-filter-c`**: Submódulo Git apontando para a biblioteca pura em C do Filtro de Kalman.
- **`components/esp_rt_diagnostics`**: Componente reutilizável de desenvolvimento para estatísticas temporais limitadas, contadores de eventos, deadlines e snapshots imutáveis de diagnóstico. Não captura séries temporais.
- **`components/esp_rt_diagnostics_reporter`**: Reporter assíncrono reutilizável que transfere diagnóstico e um payload da aplicação para uma tarefa de baixa prioridade sem bloquear o loop monitorado.
- **`main/`**: Aplicação em tempo real que lê o MT6701 e atualiza o Kalman a **3 kHz**, executa um PID de velocidade a **1 kHz**, comanda a ponte H e publica telemetria a cada 5 segundos.

### Separação de responsabilidades

| Responsabilidade | Onde está | Função |
|---|---|---|
| Aquisição e estimação | `realtime_loop.c` e `engine_angle_kalman.c` | Lê o sensor e estima ângulo, velocidade e aceleração a 3 kHz |
| Controle | `components/esp_pid` e `motor_controller.c` | Executa o PID reutilizável a 1 kHz e alterna a referência do ensaio a cada 2 s por padrão |
| Instrumentação | `components/esp_rt_diagnostics` | Mede jitter, duração, erros e violações de deadline; não imprime |
| Apresentação | `components/esp_rt_diagnostics_reporter` | Mantém a fila overwrite, a tarefa no Core 0, os logs genéricos e a classificação quiet/log-affected |
| Telemetria da aplicação | `realtime_telemetry.c` e `.h` | Define o payload do motor e formata estado do motor/corrente/gravador por callback |

**Instrumentação mede o comportamento temporal. Telemetria transporta e apresenta essas medidas.** Nenhuma delas faz parte da lei de controle.

---

## ⚙️ Configurações do Projeto

### Segurança de Threads (Configurações dos Componentes)
Ambos os drivers incluem flags do Kconfig para alternar a sincronização por Mutex do FreeRTOS em tempo de compilação:
*   **`CONFIG_MT6701_THREAD_SAFE`** (Padrão: `y`): Sincroniza os acessos aos registradores via barramento I2C.
*   **`CONFIG_ENGINE_THREAD_SAFE`** (Padrão: `y`): Sincroniza a rotina de escrita de velocidade do motor.
*   *Nota:* Se desmarcados, todas as operações de mutex correspondentes são compiladas fora para fornecer transações livres de travas (lock-free) e com overhead zero.
*   Nesta aplicação, o `sdkconfig.defaults` desativa ambos os mutexes porque, após a inicialização, somente a tarefa de tempo real no Core 1 acessa o sensor e comanda o motor.

### Configuração de Pinos GPIO I2C (Configuração da Aplicação)
Configurável diretamente via `menuconfig`. Padrões:
*   **`CONFIG_APP_I2C_SDA_PIN`** (Padrão: `8`)
*   **`CONFIG_APP_I2C_SCL_PIN`** (Padrão: `9`)
*   **`CONFIG_APP_I2C_CLOCK_HZ`** (Padrão: `1000000`): clock do MT6701, configurável de 100 kHz a 1 MHz.

### Configuração do Driver do Motor / Ponte H (Configuração do Componente)
Expõe as seguintes opções no Kconfig para controle do BTS7960:
*   **`CONFIG_ENGINE_PWM_FREQ_HZ`** (Padrão: `20000` / 20 kHz): Frequência do PWM físico. Sob o limite de 80 MHz do clock do timer do MCPWM no ESP32-S3, essa frequência fornece exatamente **4000 passos de resolução**.
*   **`CONFIG_ENGINE_PIN_RPWM`** (Padrão: `1`): GPIO para o sinal PWM Horário (RPWM).
*   **`CONFIG_ENGINE_PIN_LPWM`** (Padrão: `2`): GPIO para o sinal PWM Anti-horário (LPWM).
*   **`CONFIG_ENGINE_PIN_ENABLE`** (Padrão: `3`): GPIO de Enable para R_EN/L_EN interligados.
*   **`CONFIG_ENGINE_DIRECTION_DEAD_TIME_US`** (Padrão: `50`): Intervalo em COAST inserido antes de inverter um comando não nulo.

### Diagnóstico de Tempo Real (Desenvolvimento)
*   **`CONFIG_ESP_RT_DIAGNOSTICS_ENABLE`** (Padrão: `y`): Habilita a coleta e publicação dos snapshots. Ao desabilitá-lo, a instrumentação do caminho crítico é compilada como no-op.
*   **`CONFIG_ESP_RT_DIAGNOSTICS_DETAILED_TIMING`** (Padrão: `y`): Mede as etapas nomeadas de I2C, Kalman, controle e snapshot, além dos intervalos de amostragem e controle.
*   **`CONFIG_ESP_RT_DIAGNOSTICS_WINDOW_MS`** (Padrão: `5000`): Define a janela padrão; cada instância de `esp_rt_diag_config_t` pode sobrescrevê-la.

Esses snapshots combinam diagnóstico temporal reutilizável com um estado
instantâneo do PID definido pela aplicação. São diagnósticos de desenvolvimento,
não uma captura de séries temporais do controle.

### Medição de corrente da BTS7960

*   **`CONFIG_ENGINE_CURRENT_SENSE_ENABLE`** (Padrão: `y`): Habilita ADC1 contínuo e DMA para as duas saídas de corrente da ponte.
*   **`CONFIG_ENGINE_CURRENT_SENSE_GPIO_R_IS`** (Padrão: `4`): Entrada ADC condicionada para corrente no sentido horário.
*   **`CONFIG_ENGINE_CURRENT_SENSE_GPIO_L_IS`** (Padrão: `5`): Entrada ADC condicionada independente para corrente no sentido anti-horário.
*   **`CONFIG_ENGINE_CURRENT_SENSE_SAMPLE_HZ`** (Padrão: `25000`): Taxa de conversão por canal; cada frame de 1 ms contém 25 amostras de cada entrada e fornece média e mediana.
*   As resistências da placa, série e pulldown, além da relação nominal `k_ILIS`, também são configuráveis.

A placa medida possui 10 kΩ entre cada saída de corrente e GND. Cada entrada usa
seu próprio resistor série de 10 kΩ, pulldown de 1 kΩ no nó do ADC, capacitor de
100 nF e clamps Schottky externos para 3,3 V/GND. Não una `R_IS` e `L_IS`. A
corrente capturada segue o estado efetivo da ponte: `R_IS` no acionamento
positivo e `L_IS`, com sinal negativo, no acionamento negativo. Falhas de cada
entrada, `BRAKE`, `COAST` e indisponibilidade são códigos reservados no mesmo
`int16_t`; portanto, continuam sendo armazenados cinco canais e 13.107 amostras.
Consulte a documentação do driver antes de conectar qualquer GPIO.

### Captura de séries temporais

O componente reutilizável `esp_timeseries_recorder` reserva um buffer contínuo
em DRAM interna e armazena registros intercalados de `int16_t`. O tamanho vem de
`CONFIG_ESP_TIMESERIES_RECORDER_BUFFER_KIB` (padrão: 128 KiB); a quantidade de
amostras é calculada em tempo de execução a partir do número de canais.

Nesta aplicação são registrados velocidade, corrente, ação de controle,
referência e ângulo bruto do sensor. O console Octave arma cada captura sob
demanda. A taxa pertence a cada operação `ARM`,
deve dividir exatamente 1 kHz e também pode ser escolhida pelo comando USB
`ARM <rate_hz>`. Com cinco canais e 128 KiB são 13.107 amostras: 26,214 s a
500 Hz ou 52,428 s a 250 Hz.

A configuração diagnóstica atual habilita um perfil em malha aberta solicitado
por `CAL START`. O motor aplica 40%, 55% e 40% por 8 s cada e retorna a `COAST`.
Um `ARM` comum inicia um novo ensaio em malha fechada com referências de
600/900 RPM. Os valores são configuráveis no Kconfig. Essa
duração permite incluir todos os patamares numa calibração a 500 Hz antes de o
buffer encher.

O componente reutilizável `esp_angle_lut` corrige um ângulo cíclico em contagens
nativas antes do estimador de Kalman. Sua escala completa configurável aceita
resoluções de sensores, em potência de dois, entre 8 e 16 bits; esta aplicação
usa 14 bits para o MT6701. As 256 correções em contagens são interpoladas no
caminho de 3 kHz e armazenadas em dois slots NVS protegidos por CRC. O console
Octave realiza o ensaio, calcula e valida a LUT, envia os dados
binários, verifica a leitura de volta e só então a habilita. O canal registrado
`angle_raw` permanece sempre sem correção, impedindo que uma recalibração
aprenda a LUT anterior.

Quando o buffer enche, permanece imutável em `FULL` até `CLEAR`. A API já expõe
metadados, endereços, escalas, contadores de saturação/valores inválidos e uma
visão estável do payload ao componente separado de transporte USB. A porta USB
Serial/JTAG nativa aceita `PING`, `INFO`, `STATUS`, `ARM`, `DUMP`, `CLEAR`,
`CONTROL ...`,
`CAL ...` e `HELP`. `DUMP` envia um cabeçalho texto autodescritivo seguido das amostras
binárias little-endian protegidas por CRC-32/IEEE. Consulte
`tools/octave/README.md` para receber, converter e plotar a captura. A UART0
continua sendo a porta de gravação e logs, impedindo que logs entrem no fluxo
binário. Execute `ts_console()` no Octave para selecionar a porta e operar o
gravador por menus guiados. A captura automática ao redor dos degraus continua
disponível no Kconfig, mas vem desabilitada para não disputar um `ARM` do host.
O cliente Octave informa separadamente os tempos de aquisição e transferência;
a USB CDC nativa não usa efetivamente o baud rate nominal da API serial.

---

## ⚡ Otimizações de Alta Velocidade & Precisão de Tempo

Para suportar altas velocidades de rotação (como 30.000 RPM ou mais) e garantir a máxima precisão de estimativa, o firmware implementa os seguintes comportamentos otimizados:

### 1. Calibração de Direção e Zero por Software
*   Em vez de gravar configurações na EEPROM física do MT6701 (que requer alimentação de 5.0V VDD e introduz delays longos de travamento de escrita), os offsets de calibração de zero e a direção lógica CW/CCW são calculados matematicamente em software.
*   Isso torna a interface I2C **somente leitura** durante o funcionamento do sistema, assegurando compatibilidade com tensões de 3.3V no barramento e eliminando riscos de corromper a EEPROM física.

### 2. Leitura I2C de Alta Velocidade (2 Bytes em Burst)
*   Em cada ciclo de 333,333 µs do estimador, a aplicação executa uma leitura I2C síncrona em rajada de **2 bytes** dos registradores `0x03` e `0x04`. A transação termina antes da atualização do Kalman, portanto o estimador sempre usa a amostra mais recente e não precisa de projeção do pipeline.
*   A transação combinada requer aproximadamente 45 pulsos de SCL, correspondendo a um mínimo teórico de **~45 µs** com clock de 1 MHz.
*   O datasheet especifica período mínimo de SCL de 1 µs, permitindo 1 MHz, desde que os tempos de subida e descida de SDA/SCL não ultrapassem 150 ns. Pull-ups externos adequados e conexões curtas são recomendados; 400 kHz permanece disponível como alternativa conservadora no `menuconfig`.

### 3. Medição Dinâmica do Delta de Tempo (`dt`)
*   O driver MT6701 marca o instante em que cada aquisição síncrona termina. O intervalo real entre esses instantes, e não um período ideal fixo, é passado ao Kalman.
*   O PID usa diretamente o estado atual do Kalman. Não há projeção futura do ângulo, da velocidade ou da aceleração.

### 4. Agendamento em Duas Taxas
*   Um **GPTimer de 1 MHz** usa o padrão periódico de alarmes absolutos **333/333/334 µs**. Os três intervalos totalizam exatamente 1 ms, resultando em média exata de 3 kHz. A ISR rearma o próximo alarme e envia uma notificação direta para a tarefa de tempo real; nenhuma transação I2C ou operação do Kalman é executada dentro da interrupção.
*   O MT6701 e o estado completo do Kalman (posição, velocidade e aceleração) são atualizados a **3 kHz**.
*   A cada três amostras, o PID calcula e aplica um comando com sinal, limitado entre **-100% e 100%**, resultando em uma taxa exata de controle de **1 kHz**. Comandos positivos acionam o motor para frente, comandos negativos aplicam torque reverso e o zero exato seleciona frenagem dinâmica. Para ensaios de sintonia, a referência alterna entre **600 e 900 RPM** a cada 2 segundos por padrão. Antes de `ARM`, o Octave pode consultar ou substituir temporariamente `Kp`, `Ki`, `Kd` e o período; um reset restaura os padrões compilados.
*   Uma tarefa de baixa prioridade no Core 0 recebe telemetria a cada **5 segundos**. Ela preserva a janela afetada pela impressão anterior e a exibe junto da janela silenciosa seguinte, identificando-as como `log-affected` e `quiet`. A configuração de teste emite uma linha compacta de diagnóstico e uma linha compacta de estado por janela. O relatório aparece a cada **10 segundos**, sem esconder o impacto da própria instrumentação. Nenhuma formatação ou impressão ocorre no Core 1 depois que o GPTimer é iniciado.

### 5. Isolamento do Loop de Tempo Real
*   A tarefa completa de aquisição, estimação e controle é criada com `xTaskCreatePinnedToCore()` no **Core 1**, usando a prioridade `configMAX_PRIORITIES - 1`.
*   O barramento I2C, o MT6701 e o GPTimer são inicializados dentro dessa própria tarefa. Assim, as interrupções dos periféricos são alocadas a partir do Core 1, evitando migração da tarefa e cruzamentos de núcleo no caminho crítico.
*   A tarefa `app_main`, os serviços de `esp_timer` e a tarefa de telemetria permanecem no **Core 0**.
*   O ESP32-S3 opera a **240 MHz**, o firmware é compilado com otimização de desempenho e os mutexes dos drivers são removidos porque os periféricos possuem um único proprietário.
*   O tick do FreeRTOS permanece em **1 kHz**: a temporização de 3 kHz vem do GPTimer e não exige elevar a frequência global do escalonador.

### 6. Diagnóstico Temporal por Janela
*   A telemetria é copiada para o Core 0 somente uma vez a cada 5 segundos, em vez de atualizar uma fila a cada ciclo de controle. Janelas silenciosas e afetadas pelo log são apresentadas separadamente.
*   Cada janela informa taxas efetivas, notificações perdidas, erros I2C, violações do deadline de 334 µs e mínimos/máximos de `dt`. Falhas mostram o valor da janela e o total acumulado.
*   Médias, máximos e contadores de violação dos orçamentos de cada etapa são reiniciados em cada janela. Um máximo vitalício do processamento é mantido apenas como referência.

---

## 📈 Filtro de Kalman (3D)

O projeto integra a biblioteca pura em C do Filtro de Kalman de [kalman-filter-c](https://github.com/smartsensingme/kalman-filter-c.git) para estimar posição ($\theta$), velocidade ($\omega$) e aceleração ($\alpha$).

### Correção de Transição Angular (Wrap-around)
Devido ao comportamento circular do encoder ($0^\circ \to 360^\circ$), o módulo [engine_angle_kalman.c](main/engine_angle_kalman.c) implementa a função especializada `engine_angle_kalman_3d_update` para normalizar o erro de medição (inovação) no intervalo de $[-180^\circ, 180^\circ]$ a fim de evitar picos falsos ao cruzar a borda física.

### Ajuste de Alta Precisão para o MT6701
A covariância de medição usada no diagnóstico atual é **`0.002f`** (desvio
padrão de aproximadamente $0.0447^\circ$). Ela foi aumentada de `0.0004f` para
testar maior suavização da velocidade; as capturas mostraram apenas pequena
redução da ondulação síncrona com a rotação.

---

## 🚀 Como Compilar e Executar

1.  **Clone o projeto e suas dependências:**
    Este repositório utiliza submódulos do Git. Clone de forma recursiva:
    ```bash
    git clone --recursive https://github.com/smartsensingme/esp-mt6701-example.git
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
   git submodule add https://github.com/smartsensingme/esp-engine-driver-.git components/esp-engine-driver
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
