# esp_rt_diagnostics

Instrumentação de desenvolvimento para tarefas determinísticas em ESP-IDF. O
componente acumula estatísticas temporais limitadas e contadores em memória
estática pertencente à tarefa e produz snapshots imutáveis sob demanda.

Ele deliberadamente não cria timers ou tarefas, não acessa sensores, não
transporta amostras, não formata logs e não grava séries temporais. A aplicação
define essas políticas e pode anexar ao snapshot seu próprio estado instantâneo
de controle.

## Propriedade e comportamento em tempo real

Cada `esp_rt_diag_t` possui exatamente uma tarefa proprietária. A API do caminho
crítico não usa heap, mutex, fila, busca por strings ou logging. O snapshot deve
ser extraído por essa mesma tarefa e pode então ser copiado para uma tarefa de
prioridade inferior.

A latência de despertar só é registrada quando existe exatamente um evento
periódico pendente. Quando eventos se acumulam, o último timestamp da ISR não
permite identificar com segurança o primeiro evento atrasado. Nesse caso, o
componente informa os excedentes em `missed_events`, em vez de publicar uma
latência enganosa.

Desabilite `CONFIG_ESP_RT_DIAGNOSTICS_ENABLE` para transformar a instrumentação
inline do caminho crítico em no-ops durante a compilação.

## Integração típica

A aplicação atribui IDs compactos; o componente nunca procura nomes no caminho
crítico:

```c
enum { STAGE_I2C, STAGE_CONTROL, STAGE_COUNT };
enum { EVENT_SENSOR_OK, EVENT_SENSOR_ERROR, EVENT_COUNT };
enum { INTERVAL_SAMPLE, INTERVAL_COUNT };

esp_rt_diag_t diagnostics;
ESP_ERROR_CHECK(esp_rt_diag_init(
    &diagnostics,
    &(esp_rt_diag_config_t) {
        .expected_period_us = 250,
        .deadline_us = 250,
        .stage_count = STAGE_COUNT,
        .event_count = EVENT_COUNT,
        .interval_count = INTERVAL_COUNT,
    },
    esp_timer_get_time()));
```

A ISR captura somente o timestamp do evento. A tarefa despertada é proprietária
de todas as outras atualizações:

```c
// ISR
esp_rt_diag_isr_capture(&last_isr_time_us);

// Tarefa periódica
esp_rt_diag_cycle_begin_from_isr(&diagnostics, &last_isr_time_us,
                                 pending_events);

int64_t start_us = esp_rt_diag_stage_begin();
esp_err_t result = read_sensor();
esp_rt_diag_stage_end(&diagnostics, STAGE_I2C, start_us);
esp_rt_diag_event(&diagnostics,
                  result == ESP_OK ? EVENT_SENSOR_OK : EVENT_SENSOR_ERROR, 1);

int64_t cycle_end_us = esp_rt_diag_cycle_end_now(&diagnostics);
if (esp_rt_diag_snapshot_due(&diagnostics, cycle_end_us)) {
    esp_rt_diag_snapshot_t snapshot;
    ESP_ERROR_CHECK(esp_rt_diag_take_snapshot(
        &diagnostics, cycle_end_us, &snapshot));
    publish_without_waiting(&snapshot);
}
```

Cada contexto aceita até oito etapas, oito eventos da aplicação e quatro
intervalos. Várias tarefas determinísticas usam contextos independentes.
