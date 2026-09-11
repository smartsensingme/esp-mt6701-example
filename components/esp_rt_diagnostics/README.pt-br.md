# esp_rt_diagnostics

`esp_rt_diagnostics` é uma instrumentação limitada para o desenvolvimento de
tarefas periódicas e determinísticas em ESP-IDF. Ele ajuda a responder:

- A tarefa recebeu todos os eventos periódicos?
- Quanto tempo depois da ISR do timer ela despertou?
- Quanto demoraram o processamento total e cada etapa?
- Quantas vezes um ciclo ou uma etapa ultrapassou seu orçamento?
- Quais foram os menores e maiores intervalos definidos pela aplicação?

O componente apenas acumula medições numéricas e cria snapshots imutáveis. Ele
não cria o timer da aplicação, não suspende a tarefa monitorada, não formata
logs, não grava séries temporais e não interpreta motor ou sensor. Use
`esp_rt_diagnostics_reporter` para levar snapshots a uma tarefa de apresentação
de baixa prioridade.

Agentes de IA podem consultar o `AGENTS.md` deste diretório para instrumentar o
caminho determinístico corretamente e o `AGENTS.md` do reporter para interpretar
as evidências e planejar experimentos controlados de temporização.

## Arquitetura e propriedade

Cada `esp_rt_diag_t` pertence exclusivamente a uma tarefa monitorada:

```text
ISR do timer                       tarefa monitorada
    |                                      |
    +-- isr_capture(timestamp)              |
                                           +-- cycle_begin_from_isr(...)
                                           +-- stage/event/interval
                                           +-- cycle_end_now()
                                           +-- snapshot quando devido
                                                      |
                                                      +-- cópia ao consumidor
```

O acumulador tem vetores de tamanho fixo, não aloca memória e não usa locks.
Com exceção de `esp_rt_diag_isr_capture()`, as atualizações e os snapshots devem
ser feitos pela tarefa proprietária. Para monitorar várias tarefas, crie uma
instância independente para cada uma.

As funções do caminho crítico são `static inline`. Ao desabilitar
`CONFIG_ESP_RT_DIAGNOSTICS_ENABLE`, seus corpos viram no-ops na compilação. A
medição de etapas e intervalos também depende de
`CONFIG_ESP_RT_DIAGNOSTICS_DETAILED_TIMING`.

## Conceitos

### Ciclo, latência de despertar, processamento e deadline

```text
evento na ISR               início da tarefa                 fim da tarefa
     |-----------------------------|-------------------------------|
           latência de despertar            processamento
     |-------------------------------------------------------------|
                   consumo do deadline / tempo do ciclo
```

O deadline é violado quando
`wake_latency_us + processing_us > deadline_us`.

A latência de despertar só é válida quando existe exatamente um evento
pendente. Com `pending_events > 1`, o único timestamp armazenado pela ISR não
permite determinar a latência de todos os eventos acumulados. O componente
conta `pending_events - 1` como perdas e omite a latência desse ciclo, em vez de
produzir uma medição enganosa.

### Janelas e contadores de vida inteira

As medições são agrupadas em janelas. `window_duration_us` define a duração
desejada; zero usa `CONFIG_ESP_RT_DIAGNOSTICS_WINDOW_MS`. O snapshot contém a
duração efetivamente transcorrida e recebe um `window_id` crescente, iniciado
em 1.

Extrair um snapshot limpa contadores, máximos, etapas e intervalos da janela,
mas preserva os totais e `lifetime_max_processing_time_us`. Reinicializar a
instância apaga tanto a janela quanto o histórico completo.

### Etapas, eventos e intervalos

- Uma **etapa** é um trecho como I2C, estimador ou controle. O modo detalhado
  mede chamadas, duração total/média, duração máxima e violações opcionais.
- Um **evento** é um contador definido pela aplicação, como atualização do
  estimador ou erro do sensor. Há totais por janela e por vida inteira.
- Um **intervalo** é uma duração já medida, como o tempo entre amostras. São
  guardados quantidade de amostras, mínimo e máximo.

A aplicação atribui IDs inteiros pequenos. Não há busca de nomes no caminho de
tempo real. Cada instância suporta oito etapas, oito eventos e quatro intervalos.

## Configuração

### Kconfig

| Opção | Significado |
|---|---|
| `CONFIG_ESP_RT_DIAGNOSTICS_ENABLE` | Habilita ciclo, despertar, deadline e eventos. |
| `CONFIG_ESP_RT_DIAGNOSTICS_DETAILED_TIMING` | Habilita timestamps e estatísticas de etapas e intervalos. |
| `CONFIG_ESP_RT_DIAGNOSTICS_WINDOW_MS` | Duração padrão quando a instância seleciona zero. |

### `esp_rt_diag_config_t`

| Campo | Unidade / faixa | Significado |
|---|---|---|
| `expected_period_us` | microssegundos, diferente de zero | Período nominal usado como metadado e na taxa esperada. |
| `deadline_us` | microssegundos, diferente de zero | Maior soma permitida de latência e processamento; pode diferir do período. |
| `window_duration_us` | microssegundos | Janela da instância; zero seleciona o Kconfig. |
| `stage_count` | 0 a 8 | Quantidade de IDs válidos de etapas. |
| `event_count` | 0 a 8 | Quantidade de IDs válidos de eventos. |
| `interval_count` | 0 a 4 | Quantidade de IDs válidos de intervalos. |
| `stage_budget_us[id]` | microssegundos | Limite estrito da etapa; zero desabilita apenas a contagem de violações. |

Os contadores usam aritmética sem sinal e não saturam. Escolha duração e
incrementos que não excedam `uint32_t`. Todos os timestamps de uma instância
devem usar o mesmo relógio monotônico em microssegundos.

## Exemplo completo de integração

Defina IDs estáveis e inicialize antes de iniciar o timer periódico:

```c
enum { STAGE_I2C, STAGE_ESTIMATOR, STAGE_CONTROL, STAGE_COUNT };
enum { EVENT_ESTIMATOR_UPDATE, EVENT_SENSOR_ERROR, EVENT_COUNT };
enum { INTERVAL_SAMPLE, INTERVAL_COUNT };

static esp_rt_diag_t diagnostics;
static volatile uint32_t last_timer_isr_us;

ESP_ERROR_CHECK(esp_rt_diag_init(
    &diagnostics,
    &(esp_rt_diag_config_t) {
        .expected_period_us = 1000,
        .deadline_us = 1000,
        .window_duration_us = 5000000,
        .stage_count = STAGE_COUNT,
        .event_count = EVENT_COUNT,
        .interval_count = INTERVAL_COUNT,
        .stage_budget_us = {
            [STAGE_I2C] = 300,
            [STAGE_ESTIMATOR] = 100,
            [STAGE_CONTROL] = 150,
        },
    },
    esp_timer_get_time()));
```

A ISR armazena somente o timestamp do evento mais recente:

```c
static bool IRAM_ATTR timer_alarm_callback(/* argumentos do driver */)
{
    esp_rt_diag_isr_capture(&last_timer_isr_us);
    /* Notifica a tarefa periódica. */
    return higher_priority_task_woken;
}
```

A tarefa proprietária faz todas as demais atualizações:

```c
uint32_t pending_events = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
esp_rt_diag_cycle_begin_from_isr(
    &diagnostics, &last_timer_isr_us, pending_events);

int64_t i2c_start_us = esp_rt_diag_stage_begin();
esp_err_t sensor_error = read_sensor();
esp_rt_diag_stage_end(&diagnostics, STAGE_I2C, i2c_start_us);

esp_rt_diag_event(
    &diagnostics,
    sensor_error == ESP_OK ? EVENT_ESTIMATOR_UPDATE : EVENT_SENSOR_ERROR,
    1U);
esp_rt_diag_interval(&diagnostics, INTERVAL_SAMPLE, measured_sample_dt_us);

int64_t cycle_end_us = esp_rt_diag_cycle_end_now(&diagnostics);
if (esp_rt_diag_snapshot_due(&diagnostics, cycle_end_us)) {
    esp_rt_diag_snapshot_t snapshot;
    bool snapshot_taken = false;
    ESP_ERROR_CHECK(esp_rt_diag_try_take_snapshot(
        &diagnostics, cycle_end_us, &snapshot, &snapshot_taken));
    if (snapshot_taken) {
        publish_copy_without_waiting(&snapshot);
    }
}
```

`esp_rt_diag_stage_begin()` e `esp_rt_diag_stage_end()` leem o timer. Quando a
aplicação já possui ambos os timestamps, use `esp_rt_diag_stage_record()` para
evitar uma leitura adicional.

## Guia da API

### Ciclo de vida e snapshots

- `esp_rt_diag_init()` copia a configuração e abre a janela 1.
- `esp_rt_diag_snapshot_due()` é o teste inline que não altera estado.
- `esp_rt_diag_try_take_snapshot()` distingue “ainda não devido” de erro por
  meio de `snapshot_taken`.
- `esp_rt_diag_take_snapshot()` encerra a janela incondicionalmente e falha se
  houver um ciclo aberto, evitando dividir uma execução entre dois snapshots.

### Registro no caminho crítico

- `esp_rt_diag_isr_capture()` é a única função usada na ISR.
- `esp_rt_diag_cycle_begin_from_isr()` é seu par normal na tarefa.
- `esp_rt_diag_cycle_begin()` e `_now()` atendem outros fluxos de timestamps.
- `esp_rt_diag_cycle_end()` e `_now()` finalizam a contabilização do deadline.
- `esp_rt_diag_event()` incrementa contadores da janela e da vida inteira.
- `esp_rt_diag_stage_begin()`, `_record()` e `_end()` medem etapas.
- `esp_rt_diag_interval()` registra um intervalo calculado pela aplicação.

### Valores derivados

`esp_rt_diag_cycle_rate_hz()`, `esp_rt_diag_event_rate_hz()`,
`esp_rt_diag_deadline_overrun_percent()` e
`esp_rt_diag_stage_average_us()` destinam-se à apresentação fora do caminho de
tempo real. Entradas inválidas ou vazias retornam zero, não NaN ou infinito.

## Como interpretar o snapshot

- `cycles` é o trabalho efetivamente concluído na janela.
- `missed_events` é o número de notificações periódicas pendentes excedentes.
- `deadline_overruns` conta ciclos cuja execução, somada à latência válida,
  ultrapassou `deadline_us`.
- `max_processing_time_us` não inclui a latência do escalonador.
- `max_cycle_time_us` inclui latência somente nos ciclos sem ambiguidade.
- `cycles_with_valid_wake_time` informa a quantidade de evidências da latência.
- campos com `total_` e `lifetime_` cobrem toda a vida da instância.

Não compare apenas `max_processing_time_us` com o deadline quando a latência do
escalonador importa; analise também `max_cycle_time_us` e o contador de violações.

## Erros e diagnóstico

- `ESP_ERR_INVALID_ARG` na inicialização costuma indicar período/deadline zero
  ou quantidade excessiva de IDs.
- O snapshot rejeita tempo regressivo e ciclo aberto. Termine o ciclo também nos
  caminhos de erro da aplicação.
- Ausência de etapas/intervalos normalmente indica modo detalhado desabilitado
  ou ID fora da quantidade configurada.
- Ciclos presentes sem latência válida indicam zero ou vários eventos pendentes;
  a medição foi omitida de propósito.
- A instrumentação ainda custa algumas leituras de timer e operações. Compare
  builds habilitados e desabilitados ao validar deadlines rígidos.

## Fronteira do componente

O componente deve permanecer independente de sensores, controladores, motores,
formatos de log e transportes. Estado específico da aplicação pertence a um
payload separado, copiado junto do snapshot. O reporter complementar oferece
essa ligação sem contaminar o núcleo determinístico com essas dependências.

## Integração neste projeto

`main/realtime_loop.c` mantém um acumulador na tarefa de sensor e controle. O
ciclo de aquisição de 3 kHz registra as etapas I2C e Kalman; a cada três ciclos,
registra também a etapa do controlador de 1 kHz. A aplicação mede intervalos de
sensor/controle e conta atualizações do estimador, do controle e erros do sensor.
Ao terminar a janela, o snapshot é copiado por `main/realtime_telemetry.c`; o
relatório nunca é impresso pelo núcleo monitorado.
