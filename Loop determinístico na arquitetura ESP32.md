# Loop determinístico na arquitetura ESP32

Em aplicações de aquisição de dados e/ou controle, a CPU ESP32 deveve realizar uma tarefa em intervalo de tempo bem definido. No momento programado, a CPU deve parar tudo o que estiver fazendo para executar as tarefas de aquisição/controle.

Na ESP-IDF, as tarefas normalmente são criadas usando a função do FreeRTOS `vTask`.  As tarefas criadas com `vTask` são escalanoadas pelo próprio Kernel do FreeRTOS.

O problema é que, por definição, o FreeRTOS é configurado com um tick interno de apenas 1 ms. Assim, caso sua tarefa de controle/aquisição exija um período menor, você deve reduzir esse tick como segue.

### 1. Reduzindo o tick do FreeRTOS

Para tonar o valor do tick do FreeRTOS mais breve, você precisa ajustar as configurações do projeto via **`menuconfig`**.

#### 1.1. Qual o limite razoável? (O quão pequeno ele pode ser?)

Embora o FreeRTOS teoricamente permita configurar o tick rate em valores muito altos, o limite prático nos chips ESP32 (como as linhas S3, C3, C6 ou P4) gira em torno de **1000 Hz a 2000 Hz (ticks de 1 ms a 0,5 ms)**.

##### O Limite Prático: Até 2000 Hz

Subir o tick para **2000 Hz** (0,5 ms) ainda é considerado aceitável se a sua aplicação exigir precisão estrita em temporizações de tarefas nativas do FreeRTOS. Passar disso raramente vale a pena devido ao impacto no desempenho.

#### 1.2. O que acontece se você colocar o tick muito alto? (Ex: 10000 Hz / 0,1 ms)

1. **Sobrecarga da CPU (Overhead):** A cada tick, o FreeRTOS interrompe a execução do código principal para salvar o contexto da tarefa atual, rodar o scheduler (escalonador), verificar qual tarefa deve rodar e restaurar o contexto. Se o tick for de 0,1 ms, a CPU passará uma porcentagem massiva do seu tempo apenas gerenciando o sistema operacional, em vez de executar suas rotinas de controle ou comunicação.

2. **Desperdício de Energia:** O processador não conseguirá entrar em modos de low-power (como o tickless idle) de forma eficiente, aumentando drasticamente o consumo de corrente.

## 2. Solução para tempos muito curtos

Se sua aplicação exige um tempo de aquisição/controle muito curta, algumas ações devem ser tomadas.

### 2.1. Divida os recursos de hardware

As CPUs ESP32 possuem dois núcleos de processamento e muitas vezes dois barramentos SPI. Para garantir que seu loop de aquisição/controle seja executado com a menor latência possível, reserve um das duas CPUs e um dos dois barramentos SPI apenas para esse loop. Isto é feito como segue.

#### 2.1.1. Definindo o `core` de execução de uma tarefa

Ao criar uma tarefa usando `vTask`, o próprio FreeRTOS escolhe em qual CPU a terafa irá rodar. Por isso, você deve criar suas tarefas usando `xTaskCreatePinnedToCore`.

O código que segue cria duas tarefas. Uma roda no Core 0, enquanto a outra é executada no Core 1.

```c
/* Variáveis static são alocadas na mesma região de memória do código */
static TaskHandle_t control_task_handle = NULL;

int main(void){
    /* Configurações iniciais. */

    /* Distribuição das tarefas por núcleos. */
    xTaskCreatePinnedToCore(rms_calc_task, "rms_calc", 8192, NULL, 5,
                          &rms_task_handle, 0); // Core 0
    xTaskCreatePinnedToCore(control_loop_task, "control_loop", 8192, NULL,
                          configMAX_PRIORITIES - 1, &control_task_handle,
                          1); // Core 1

return 0;
}
```

A ideia é que o Core 1 execute apenas o loop de controle/aquisição (neste exemplo, a `control_loop_task`). O Core 0 será executará todas as tarefas secundárias.

#### 2.1.2. Blindagem do Core 1 e Configuração via `menuconfig`

Alguns componentes padrão da ESP-IDF Na ESP-IDF, isso é alcançado através da reconfiguração dos parâmetros de afinidade do RTOS e do particionamento de rede. As seguintes alterações devem ser rigorosamente aplicadas através da interface `idf.py menuconfig`:

1. **Afinidade da Tarefa Principal (`app_main`)**:
   A tarefa padrão que inicializa o sistema operacional deve ser compulsoriamente movida para o núcleo de gerenciamento (Core 0).

   * **Caminho:** *Component config → ESP System Settings*.
   * **Configuração:** `Main task core affinity` configurado para `CPU0`.
   * **Impacto:** Garante que toda a lógica de inicialização de *drivers*, configuração de periféricos e instanciação dinâmica de outras tarefas assíncronas não roube ciclos de processamento do Core 1.

2. **Isolamento das Pilhas de Comunicação (Wi-Fi e TCP/IP)**:
   As pilhas de rede são os maiores geradores de seções críticas que desabilitam interrupções globalmente no silício.

   * **Wi-Fi:** *Component config → Wi-Fi → WiFi Task Core ID* → `Core 0`.
   * **LwIP:** *Component config → LWIP → TCPIP task affinity* → `CPU0`.
   * **Impacto:** Impede que a enxurrada de processamento gerada por pacotes de rede cause *jitter* na *Task* matemática, mantendo o *pipeline* do Core 1 livre das preempções do Wi-Fi.

3. **Gestão do Temporizador de Sistema (ESP Timer)**:
   O serviço de *timers* de alto nível da IDF (usado internamente pelo FreeRTOS) pode interferir na baixa latência do nosso núcleo dedicado.

   * **Caminho:** *Component config → ESP Timer*.
   * **Configuração:** `ESP Timer task core affinity` → `CPU0`.
   * **Impacto:** Garante que funções temporizadas do sistema (como *timeouts* de *software* e *callbacks* lentos) não concorram pelo processador no Core 1.

4. **Otimização de Latência e a FPU na ISR**:
   Ao contrário de arquiteturas onde a matemática reside na interrupção, o modelo de Interrupção Postergada adotado dispensa a necessidade de salvar o contexto da FPU na ISR. A opção `Enable FPU context save/restore in ISR` (dentro de *FreeRTOS → Kernel*) pode e **deve ser mantida desabilitada**.

   * **Impacto:** Essa decisão arquitetural reduz drasticamente os ciclos de *clock* exigidos para o processador saltar para a função `adc_on_conv_done`, criando um gatilho de latência quase nula. O salvamento seguro dos registradores da FPU fica totalmente a cargo do chaveamento de contexto nativo do FreeRTOS quando a *Task* de controle é acordada.

5. **Saúde do Sistema e o Task Watchdog (TWDT)**:
   Em malhas presas em ISRs contínuas, a CPU é sufocada, forçando o desenvolvedor a desligar o cão de guarda (*Watchdog*). Com a adoção da chamada bloqueante `ulTaskNotifyTake`, a *Task* matemática do Core 1 cede a CPU nos exíguos microssegundos em que aguarda a conversão do DMA.

   * **Impacto:** Esse bloqueio ultrarrápido permite que a tarefa *Idle* do Core 1 seja executada, "alimentando" o *Watchdog* de hardware naturalmente. Não é necessário desabilitar as proteções nativas do sistema.

##### Definindo as configurações direto no `sdkconfig.defaults`

As configurações anteriores (Seção 2.1.2) podem ser definidas diretamente no arquivo `sdkconfig.defaults` (que fica na raíz do projeto). Para isso, basta fazer o conteúdo deste arquivo igual ao que segue.

```
# 1. Afinidade da Tarefa Principal (app_main)
# Component config -> ESP System Settings -> Main task core affinity -> CPU0
CONFIG_ESP_MAIN_TASK_AFFINITY_CPU0=y
CONFIG_ESP_MAIN_TASK_AFFINITY=0

# 2. Isolamento das Pilhas de Comunicação (Wi-Fi e TCP/IP)
# Component config -> Wi-Fi -> WiFi Task Core ID -> Core 0
# Component config -> LWIP -> TCPIP task affinity -> CPU0
# Wi-Fi Task Core ID
CONFIG_ESP_WIFI_TASK_PINNED_TO_CORE_0=y
# (Caso esteja utilizando uma versão antiga/legada da ESP-IDF, use a linha abaixo):
# CONFIG_ESP32_WIFI_TASK_PINNED_TO_CORE_0=y

# LwIP TCP/IP Task Affinity
CONFIG_LWIP_TCPIP_TASK_AFFINITY_CPU0=y
CONFIG_LWIP_TCPIP_TASK_AFFINITY=0

# 3. Gestão do Temporizador de Sistema (ESP Timer)
# Component config -> ESP Timer -> ESP Timer task core affinity -> CPU0
CONFIG_ESP_TIMER_TASK_AFFINITY_CPU0=y
CONFIG_ESP_TIMER_TASK_AFFINITY=0

# 4. Otimização de Latência e a FPU na ISR
# FreeRTOS -> Kernel -> Enable FPU context save/restore in ISR -> Desabilitado
# CONFIG_FREERTOS_FPU_IN_ISR is not set

# 5. Saúde do Sistema e o Task Watchdog (TWDT)
# Mantidos ativos por padrão. Não é necessário desativar ou alterar nada.
# Explicitando as configurações padrão de proteção ativa:
CONFIG_ESP_TASK_WDT=y
```

> No sistema de configuração **Kconfig** (utilizado pela ESP-IDF, Zephyr e pelo próprio Kernel do Linux), a sintaxe padrão para **desabilitar** uma opção do tipo boolean é escrevê-la exatamente dessa forma:
>
>  `# CONFIG_FREERTOS_FPU_IN_ISR is not set`
>
> Embora pareça um comentário comum (por começar com `#`), o gerador de configurações da ESP-IDF lê essa linha específica e a interpreta como uma instrução explícita de: **"defina esta opção como desabilitada (falso)"**.

#### 2.1.3 Cuidado com bibliotecas de terceiros

Ao usar bibliotecas de terceiros, é importante certifica-se que nenhuma dessas bibliotecas usa `vTask`para criar as tarefas. Caso a biblioteca precise criar tarefas, é importante que você altere o código da biblioteca para `xTaskCreatePinnedToCore` no lugar de `xTask`.

#### 2.1.2. Defina um barramento SPI exclusivo para sua tarefa

O barramento SPI é um recurso compartilhado. Sempre que tarefa toma esse barramento, ela não pode ser interrompida. Assim, seu loop de controle/aquisição não pode ter que esperar sempre que precisar usar esse barramento.

Felizmente, o ESP32 possui dois barramentos SPI. Você precisa apenas garantir que seu loop de controle/aquisição tome controle de um barramento SPI e não o libere.

O código seguinte adquire o barramento com `spi_device_acquire_bus`, mas nunca chama `spi_device_release_bus`para liberar o acesso. Assim, garantimos 100% de utilização do barramento.

```c
static void IRAM_ATTR control_loop_task(void *arg) {
  ESP_LOGI(TAG, "Entering DMA Control Loop on Core %d", xPortGetCoreID());

  // Para o laço de controle não ter que pedir acesso ao barramento a
  // cada interrupção, nós adquirimos um barramento aqui e não o
  // liberamos mais.
  spi_device_acquire_bus(dac_ctx.spi_handle, portMAX_DELAY);

  while(1){
    /* Loop de controle/aquisição. */
  }
}
```

##### Cuidados com funções padrão de escrita no barramento

O FreeRTOS usa semáforos complexos para controlar o acesso a recursos. Por exemplo, a função `gpio_set_level` (usada para alterar o estado dos pinos do chip) usa travas internas (*spinlocks*) para garantir que escritas simultâneas em múltiplos pinos por cores diferentes não corrompam os registradores. Isso introduz grande latência. Por isso, elas não devem ser usadas para manipular dispositivos SPI pelo loop de controle/aquisição. Ao invés disso, use funções com o pré-fixo **`_ll_`**.

Funções de pré-fixo **`_ll_`** confiam cegamente nos parâmetros que você passou. Por exemplo, `gpio_ll_set_level` é uma função de baixo nível, que não faz nenhuma checagem extra para garantir a integridade do sistema Assim, se você passar um GPIO inválido, causará comportamento indefinido. Contudo, use funções com o pré-fixo **`_ll_`** para garantir o mínimo de latência em seus loops de controle/aquisição.

### 2.2. Executando uma tarefa dentro da memória interna do chip

As CPUs ESP32 possuem uma memória interna limitada. Assim, ao gerar seu arquivo executável, preferencialmente, todo o código será alocado em memória externa ao chip principal. Para sistemas de tempo real isso é um problema, pois a CPU tem que tomar o controle do barramento de memória para executar código localizado na memória externa. Isso provoca latência, aumentando o atraso de execução do código.

Para que sua tarefa de controle/aquisição seja executada com latência mínima, ela teve ser armazenada em memória interna. Isso é feito usando a macro `IRAM_ATTR` como segue.

```c
static void IRAM_ATTR control_loop_task(void *arg) {
    /* Código interno da função */
}
```

No nosso exemplo, não apenas `control_loop_task`, mas todas as funções chamadas dentro de `control_loop_task` devem ser declaradas com o pré-fixo `static` e a macro `IRAM_ATTR`. Isso garante o mínimo tempo de latência na execução do código.

### 2.3. Use uma fonte de interrupção externa para acordar seu loop de controle/aquisição

Uma vez que a garantimos que `Core 1` irá executar apenas o loop de controle/aquisição, que o código está armazenado em memória interna e que um barramento SPI está dedicado 100% a esse código, a latência de atendimento do nosso loop será mínima.

O próximo passo é garantir que o loop seja executado no instante correto. A melhor forma de fazer isso é através de uma interrupção de hardware, ou seja, algum periférico do ESP32, independete da CPU e do FreeRTOS, irá temporizar as ações de aquisição/controle.

No código que segue, a função `ulTaskNotifyTake` suspende a execução do `control_loop_task` até que ela seja "acordada" por uma rotina de interrupção.

```c
static void IRAM_ATTR control_loop_task(void *arg) {

    /* Executa toda configuração inicial para preparar o loop. */

    while (1) {
        /* Espera notificação externa para acordar. */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        /* Executa o código que aquisição e controle. */
    }
}
```

Uma fonte de interrupção muito utilizada é o conversor A/D. No ESP32, o conversor A/D pode ser programado em modo contínuo, onde a conversão A/D ocorre em um intervalo de tempo pré-definido, pode gerar uma interrupção quando as conversões estiverem disponíveis.

No trecho que segue, o DAC está sendo configurado para chamar a rotina de interrupção `adc_on_conv_done` assim que a conversão A/D for finalizada.

```c
 adc_continuous_evt_cbs_t cbs = {
      .on_conv_done = adc_on_conv_done, // Função que será chamada quando
                                        // um frame for completo
  };
  ESP_ERROR_CHECK(
      adc_continuous_register_event_callbacks(adc_handle, &cbs, NULL));
```

Por sua vez, na função `adc_on_conv_done`, a chamada a `vTaskNotifyGiveFromISR` acorda a tarefa identificada por `control_task_handle`.

```c
static bool IRAM_ATTR adc_on_conv_done(adc_continuous_handle_t handle,
                                       const adc_continuous_evt_data_t *edata,
                                       void *user_data) {
  BaseType_t high_task_awoken = pdFALSE;

  /* Envia notificação para acordar o loop de controle. */
  vTaskNotifyGiveFromISR(control_task_handle, &high_task_awoken);
  return (high_task_awoken == pdTRUE);
}
```

É importante que a rotina de interrupção seja a mais enxuta possível, pois, na ESP-IDF, uma rotina de interrupção, por exemplo, não salva os registradores da **FPU**, a unidade de multiplicação em ponto flutuante. Assim, se `adc_on_conv_done` interromper uma tarefa que usa a **FPU**, todo o contexto da rotina interrompida será perdido. Ao contrário, `control_loop_task`é uma função normal. Isso garante que o sistema salve salve todo o contexto antes de executá-la.

## 3. Dica sobre otimização do código

O microcontrolador ESP32-S3 é ótimo para tarefas de controle porque tem uma FPU que pode realizar multiplicações em ponto flutuante em um único ciclo de CPU. Contudo, para usar a FPU de forma adequada, você tem que tomar as seguintes precauções.

1. Ao usar constantes nas suas equações, sempre use o sufixo `f` (ex.: `#define CONST 3.56f`). Isso garante que o compilador vai considerar a constante como um tipo `float` (a FPU só realiza multiplicações com operando do tipo `float`.).

2. A FPU não realiza divisões. Assim, qualquer divisão em será executada em vários ciclos de CPU.

3. No caso de constantes, uma divisão pode ser transformada em multiplicação. Por exemplo, ao inves de `float a = b/7.0f`, faça `float a = b * (1.0f/7.0f)`. O próprio compilador trata de resolver `(1.0f/7.0f)`, transformando em uma única constante.

## Conclusão

A implementação de loops determinísticos de alta velocidade na arquitetura ESP32 exige uma mudança de paradigma: deve-se deixar de depender puramente das abstrações de software do sistema operacional para apoiar-se diretamente nos recursos de hardware do chip. Embora a redução do tick rate do FreeRTOS funcione para ajustes finos, ela encontra um limite físico intransponível devido à sobrecarga de processamento que impõe à CPU.

Para aplicações de tempo real estrito que demandam latências mínimas e precisão de microssegundos, o sucesso do projeto depende de três pilares fundamentais:

- **Isolamento de Hardware:** A separação física de tarefas entre os núcleos através de `xTaskCreatePinnedToCore` e a dedicação exclusiva de um barramento SPI eliminam o tempo de espera gerado por concorrência e troca de contexto.

- **Execução em Memória Interna e Baixo Nível:** O uso da macro `IRAM_ATTR` impede os atrasos de leitura da memória flash externa, enquanto a substituição de funções padrão por suas contrapartes Low-Level (`_ll_`) remove camadas redundantes de proteção de threads.

- **Sincronismo Baseado em Interrupções:** Ao delegar a temporização a periféricos independentes (como o ADC em modo contínuo) e acordar a tarefa principal via notificações de ISR (`vTaskNotifyGiveFromISR`), garante-se que o código de controle reaja imediatamente ao evento de hardware.

Combinando essas estratégias, é possível blindar o loop de controle e aquisição contra as imprevisibilidades do escalonador do sistema operacional. O resultado é um firmware robusto, altamente eficiente e verdadeiramente determinístico, capaz de explorar o limite de desempenho do ESP32 sem comprometer a integridade das demais funções da aplicação.
