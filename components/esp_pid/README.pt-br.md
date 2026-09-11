# `esp_pid`

O `esp_pid` é um controlador PID reutilizável e sem alocação dinâmica para
aplicações ESP-IDF. Cada objeto `esp_pid_t` contém uma cópia de sua configuração
e todo o estado dinâmico. Assim, uma aplicação pode criar várias malhas de
controle independentes sem que o componente use variáveis globais.

O componente calcula uma ação de controle limitada. Ele não acessa PWM, GPIO,
motores, aquecedores, válvulas ou qualquer outro hardware. A aplicação fornece
a referência e a medição ao PID e decide como aplicar ao atuador a ação
calculada.

## Funcionalidades

- saturação configurável da saída;
- ação integral com anti-windup opcional por back-calculation;
- derivada aplicada à medição ou ao erro;
- filtro passa-baixas de primeira ordem no sinal derivado;
- proteção contra intervalos de amostragem inválidos ou implausíveis;
- acesso aos termos P, I e D e às saídas saturada e não saturada;
- nenhuma alocação dinâmica de memória.

## Memória interna e uso no loop determinístico

Todas as funções públicas do PID e seus auxiliares privados possuem
`IRAM_ATTR`. Dessa forma, o código executável é ligado à memória interna de
instruções do ESP32, em vez de ser buscado na flash externa por meio da cache.
Isso inclui `esp_pid_update()` e `esp_pid_get_state()`, chamadas a cada iteração
do controle atual, além das operações de inicialização e reset.

Somente colocar o código em IRAM não garante independência completa da cache. A
instância `esp_pid_t`, o destino da saída e qualquer configuração passada a
`esp_pid_init()` também precisam estar acessíveis na RAM interna enquanto as
respectivas funções executam. Não coloque uma instância de tempo real na PSRAM.
Por padrão, uma instância alocada na pilha de uma tarefa criada pelas APIs
usuais do FreeRTOS no ESP-IDF fica na RAM interna, como ocorre neste projeto.

Divisões de precisão simples e cópias de estruturas podem ser compiladas como
chamadas a `__divsf3` e `memcpy`. Na configuração ESP32-S3/ESP-IDF deste projeto,
ambas são fornecidas pela ROM interna. Ao portar o componente para outro alvo
ou toolchain, o mapa da nova compilação deve ser conferido novamente.

`IRAM_ATTR` não torna uma API segura para interrupções, reentrante ou segura
para acesso concorrente. Cada instância do PID ainda deve possuir um único
dono. A IRAM também é limitada e deve ser reservada para caminhos realmente
críticos.

## Adição do componente a uma aplicação

Coloque o `esp_pid` no diretório `components` do projeto. No componente que irá
utilizá-lo, adicione `esp_pid` a `REQUIRES` ou `PRIV_REQUIRES`:

```cmake
idf_component_register(
    SRCS "meu_controlador.c"
    INCLUDE_DIRS "."
    REQUIRES esp_pid
)
```

Inclua então o cabeçalho público:

```c
#include "esp_pid.h"
```

## Configuração: `esp_pid_config_t`

Todos os campos usam as unidades escolhidas pela aplicação. Se a variável
controlada for velocidade em RPM e a saída for o duty cycle em porcentagem, por
exemplo, o erro estará em RPM e a saída estará em porcentagem.

| Campo | Significado |
|---|---|
| `kp` | Ganho proporcional. O termo proporcional é `kp * erro`. Sua unidade é unidade de saída por unidade de entrada, por exemplo `%/RPM`. |
| `ki` | Ganho integral. O integrador acumula `ki * erro * dt_s`. Sua unidade é unidade de saída por unidade de entrada vezes segundo, por exemplo `%/(RPM.s)`. Use zero para obter um controlador PD ou P. |
| `kd` | Ganho derivativo. Sua unidade é unidade de saída vezes segundo por unidade de entrada, por exemplo `%.s/RPM`. Use zero para obter um controlador PI ou P. |
| `output_min` | Menor comando que o atuador pode receber. Deve ser menor que `output_max`. Pode ser `-100` para permitir reversão total, por exemplo, ou `0` para um atuador unidirecional. |
| `output_max` | Maior comando que o atuador pode receber. Deve ser maior que `output_min`. |
| `derivative_filter_tau_s` | Constante de tempo, em segundos, do filtro passa-baixas de primeira ordem aplicado ao sinal derivado. Um valor maior filtra mais ruído, mas introduz mais atraso. Zero desabilita o filtro. Não pode ser negativo. |
| `anti_windup_tracking_time_s` | Tempo de rastreamento, em segundos, do anti-windup por back-calculation. Um valor positivo menor remove mais agressivamente a ação integral acumulada durante a saturação. Zero desabilita o back-calculation. Não pode ser negativo. Escolha um valor seguramente maior que o período de amostragem. |
| `maximum_dt_s` | Maior intervalo de amostragem aceitável, em segundos. A integral e o filtro derivativo são mantidos quando `dt_s <= 0` ou `dt_s >= maximum_dt_s`. Zero desabilita somente o limite superior; `dt_s` ainda precisa ser positivo. |
| `derivative_source` | Seleciona se a derivada será calculada a partir do erro ou da medição. As opções são explicadas a seguir. |

Todos os campos de ponto flutuante precisam ser finitos: `NaN` e infinito são
rejeitados por `esp_pid_init()`.

### Escolha da origem da derivada

`ESP_PID_DERIVATIVE_ON_MEASUREMENT` calcula a contribuição derivativa como o
negativo da derivada da medição. Normalmente, essa é a opção mais adequada para
um controlador de referência, pois um degrau instantâneo na referência não
provoca um grande pico derivativo.

`ESP_PID_DERIVATIVE_ON_ERROR` deriva `referência - medição`. Portanto, essa
opção também reage às mudanças da referência e pode ser usada quando esse
comportamento for intencional.

## Exemplo completo de utilização

A instância normalmente deve existir durante toda a vida da malha de controle.
Não a crie e inicialize novamente em cada iteração, pois isso apagaria a
integral e o histórico da derivada.

```c
#include "esp_pid.h"

static esp_pid_t speed_pid;

static const esp_pid_config_t speed_pid_config = {
    .kp = 0.25f,
    .ki = 3.0f,
    .kd = 0.0001f,
    .output_min = -100.0f,
    .output_max = 100.0f,
    .derivative_filter_tau_s = 0.020f,
    .anti_windup_tracking_time_s = 0.20f,
    .maximum_dt_s = 0.010f,
    .derivative_source = ESP_PID_DERIVATIVE_ON_MEASUREMENT,
};

esp_err_t inicializar_controlador_velocidade(void)
{
    return esp_pid_init(&speed_pid, &speed_pid_config);
}

esp_err_t executar_controlador_velocidade(float referencia_rpm,
                                          float velocidade_rpm,
                                          float dt_s)
{
    float comando_percentual = 0.0f;
    esp_err_t erro = esp_pid_update(&speed_pid, referencia_rpm, velocidade_rpm,
                                    dt_s, &comando_percentual);
    if (erro != ESP_OK) {
        return erro;
    }

    /* O driver do atuador, e não o esp_pid, interpreta o comando. */
    if (comando_percentual == 0.0f) {
        motor_frear();
    } else {
        motor_definir_velocidade(comando_percentual);
    }
    return ESP_OK;
}
```

Para uma malha de 1 kHz, o valor nominal de `dt_s` é `0.001f`. É melhor
fornecer o tempo transcorrido medido quando o jitter do escalonamento for
relevante. Os nomes `motor_frear()` e `motor_definir_velocidade()` acima são
apenas exemplos para o driver da aplicação.

## API

### `esp_pid_init()`

```c
esp_err_t esp_pid_init(esp_pid_t *pid, const esp_pid_config_t *config);
```

Valida a configuração, copia-a para a instância, limpa todo o estado e marca a
instância como inicializada. Como a configuração é copiada, a estrutura
original não precisa continuar existindo.

Retornos:

- `ESP_OK` em caso de sucesso;
- `ESP_ERR_INVALID_ARG` se algum ponteiro for nulo ou a configuração for
  inválida.

### `esp_pid_update()`

```c
esp_err_t esp_pid_update(esp_pid_t *pid,
                         float reference,
                         float measurement,
                         float dt_s,
                         float *output);
```

Executa uma iteração do controlador. A função calcula

```text
erro = referência - medição
saída = saturar(P + I + D, output_min, output_max)
```

e armazena a ação saturada por meio do ponteiro `output`. O valor retornado pela
função é um código de erro do ESP-IDF, e não a ação de controle.

Retornos:

- `ESP_OK` quando uma saída foi calculada;
- `ESP_ERR_INVALID_STATE` se a instância ou o ponteiro de saída for inválido,
  ou se a instância não tiver sido inicializada;
- `ESP_ERR_INVALID_ARG` se `reference`, `measurement` ou `dt_s` não for finito.

Um `dt_s` finito, mas fora do intervalo aceitável, não é um erro da API. A
função retorna `ESP_OK`, atualiza o termo proporcional e a saída, mantém os
estados integral e do filtro derivativo e informa `last_dt_valid = false` no
estado.

### `esp_pid_reset()`

```c
esp_err_t esp_pid_reset(esp_pid_t *pid);
```

Limpa o integrador, o filtro e o histórico da derivada, as entradas anteriores,
os termos de diagnóstico e a última saída. Os ganhos, limites e o estado de
inicialização são preservados. Use-a ao iniciar um ensaio independente, depois
de uma mudança importante de modo ou sempre que a memória do controlador
anterior não deva influenciar uma nova execução.

Retorna `ESP_OK` ou `ESP_ERR_INVALID_STATE` para uma instância nula ou ainda não
inicializada.

### `esp_pid_get_state()`

```c
esp_err_t esp_pid_get_state(const esp_pid_t *pid, esp_pid_state_t *state);
```

Copia um retrato do diagnóstico sem alterar o controlador. Os campos mais
úteis são:

| Campo | Significado |
|---|---|
| `error` | Último valor de `referência - medição`. |
| `proportional_term` | Última contribuição proporcional. |
| `integral_term` | Valor atual do integrador depois da correção anti-windup. |
| `derivative_term` | Última contribuição derivativa filtrada, incluindo o sinal configurado. |
| `filtered_derivative` | Derivada filtrada interna antes da multiplicação por `kd` e antes da aplicação do sinal da derivada da medição. |
| `previous_error` | Erro retido para o próximo cálculo da derivada. |
| `previous_measurement` | Medição retida para o próximo cálculo da derivada. |
| `unsaturated_output` | Soma `P + I + D` antes da saturação final. |
| `output` | Comando depois da limitação aos valores configurados. |
| `previous_input_valid` | Indica que o histórico necessário para calcular a derivada já foi inicializado. |
| `last_dt_valid` | Indica se o último intervalo foi aceito para atualizar os estados dinâmicos. |

Retorna `ESP_OK` ou `ESP_ERR_INVALID_STATE` para um ponteiro inválido ou uma
instância ainda não inicializada.

## Saturação e anti-windup

Os limites da saída representam a faixa de comandos que o atuador consegue
realizar. Quando o valor solicitado por `P + I + D` ultrapassa essa faixa, a
saída é saturada. Com o back-calculation habilitado, a diferença entre as saídas
saturada e solicitada também conduz o integrador de volta a um valor
compatível:

```text
I <- I + Ki * erro * dt
I <- I + (saída_realizável - saída_solicitada) * dt / tempo_de_rastreamento
```

Isso reduz o windup integral, mas não limita corrente, temperatura, tensão,
aceleração ou esforço mecânico do motor. Essas proteções devem ser
implementadas pela aplicação e pelo driver do atuador.

## Recomendações práticas

- Use uma instância `esp_pid_t` para cada malha de controle.
- Chame `esp_pid_init()` uma vez antes da primeira atualização.
- Chame `esp_pid_update()` em uma frequência razoavelmente regular.
- Mantenha unidades dimensionalmente coerentes em todos os sinais e ganhos.
- Comece a sintonia com `ki = 0` e `kd = 0`, ajuste `kp` e introduza depois as
  ações integral e derivativa de maneira deliberada.
- Não atualize a mesma instância simultaneamente por várias tarefas sem
  sincronização.
- Tempo morto na inversão, escolha entre freio e COAST, geração de PWM e
  tratamento de falhas de hardware pertencem à camada do atuador.

Documentação em inglês: [README.md](README.md).
