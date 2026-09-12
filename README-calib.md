# Calibração da leitura angular por LUT

Este documento descreve o procedimento completo usado pelo projeto para
identificar o erro angular periódico do conjunto sensor–ímã, calcular uma LUT
(*lookup table*) no GNU Octave, transferi-la para o ESP32 e aplicá-la durante a
execução do controle.

O procedimento foi dividido de modo que aquisição, transporte, cálculo da LUT
e aplicação da correção tenham responsabilidades distintas. Essa separação
permite reutilizar o algoritmo de linearização com outros sensores angulares e
em outras aplicações.

## Fluxo completo da calibração

```text
Sensor angular
      │
      ▼
ângulo bruto
      │
      ▼
esp_timeseries_recorder
      │
      ▼
esp_timeseries_usb_transport ──USB──► Octave
                                         │
                                         ▼
                               cálculo e validação da LUT
                                         │
                                         ▼
                          arquivo MAT + gráficos + CRC
                                         │
                                         ▼
ESP32 ◄────────── CAL WRITE + LUT binária
  │
  ▼
esp_angle_lut valida e grava na NVS
  │
  ▼
ângulo bruto → LUT → ângulo corrigido → Kalman → controle
```

O fluxo pode ser resumido da seguinte forma:

1. o ESP32 executa um ensaio de rotação em malha aberta;
2. o ângulo bruto e os demais canais são armazenados na memória interna;
3. a captura é enviada ao Octave pela USB;
4. o Octave identifica o erro periódico, calcula e valida a LUT;
5. a captura e a calibração são salvas em um arquivo MAT;
6. mediante confirmação do usuário, o Octave envia a LUT ao ESP32;
7. o firmware valida o conteúdo, grava-o na NVS e lê a tabela de volta;
8. depois da verificação, a correção é habilitada;
9. as novas leituras passam pela LUT antes de serem entregues ao Kalman.

## 1. Ensaio em malha aberta

O procedimento começa quando o Octave envia:

```text
CAL START 500
```

O número no comando é a frequência de amostragem solicitada para a captura. Na
configuração atual, o ensaio usa 500 Hz.

A aplicação executa três patamares de ação de controle em malha aberta:

```text
40% → 55% → 40%
```

Cada patamar dura aproximadamente oito segundos. O eixo deve permanecer livre
durante o ensaio.

A malha aberta é usada porque o objetivo é observar o erro periódico do sensor.
Em malha fechada, o PID reagiria à ondulação da velocidade estimada e alteraria
continuamente a ação de controle. Isso dificultaria distinguir o erro do sensor
da resposta produzida pelo próprio controlador.

Os patamares constantes fazem o motor girar com velocidade aproximadamente
constante. Essa condição permite comparar o ângulo medido durante cada volta
com a progressão angular uniforme esperada.

## 2. Aquisição dos dados

O componente `esp_timeseries_recorder` armazena a captura em um buffer contíguo
na memória interna do ESP32. Na aplicação atual, cada registro contém:

- velocidade;
- corrente;
- ação de controle;
- referência;
- ângulo bruto, identificado como `angle_raw`.

Para o cálculo da LUT, os dados essenciais são:

- instante da amostra;
- ângulo bruto;
- ação de controle.

O canal `angle_raw` contém a leitura anterior à aplicação da LUT. Esse requisito
é fundamental: uma calibração nova não deve ser estimada a partir de dados que
já foram modificados por uma correção anterior.

Os valores físicos são convertidos pelo gravador em registros `int16_t`. O
cabeçalho da captura preserva o nome, a unidade, a escala e o offset de cada
canal, permitindo que o Octave reconstrua os valores físicos.

Quando o buffer fica cheio, o gravador encerra automaticamente a aquisição. A
captura permanece disponível e não é sobrescrita até que seja lida e liberada.

## 3. Transferência para o Octave

Depois que a captura atinge o estado `FULL`, o Octave solicita o `DUMP`. O
componente `esp_timeseries_usb_transport` envia:

- o cabeçalho da captura;
- os descritores e as escalas dos canais;
- as amostras binárias intercaladas;
- o CRC32 dos dados.

O transporte USB é genérico e não conhece sensores, motores nem LUTs. Os
comandos `CAL ...` são extensões implementadas pela aplicação em
[`main/angle_lut_usb_commands.c`](main/angle_lut_usb_commands.c).

Essa extensão coordena o ensaio de calibração, recebe e envia tabelas, consulta
o estado do `esp_angle_lut` e usa os serviços binários fornecidos pelo
transporte.

## 4. Seleção dos trechos estáveis

No Octave, a função `ts_calculate_angle_lut()` adapta a captura desta aplicação
ao algoritmo genérico de cálculo da LUT. Ela procura os patamares constantes no
canal `control` e seleciona somente as regiões estabilizadas.

Atualmente, são descartados em cada patamar:

- os primeiros 1,5 s, para permitir a estabilização depois da transição;
- os últimos 0,5 s, para evitar a influência da transição seguinte.

São aceitos apenas patamares positivos, suficientemente longos e com pelo menos
três segundos úteis. Os vetores de índices resultantes são entregues a
`esp_angle_lut_calculate()`.

As responsabilidades permanecem separadas:

- `ts_calculate_angle_lut()` conhece os canais e o perfil de ensaio deste
  projeto;
- `esp_angle_lut_calculate()` recebe apenas tempo, ângulo bruto, segmentos
  estáveis e os parâmetros da LUT;
- a biblioteca Octave contida em `components/esp_angle_lut/tools/octave` não
  depende do motor, do driver do sensor nem do protocolo USB desta aplicação.

## 5. Cálculo da LUT

Para cada segmento estável, o Octave executa as etapas a seguir.

### 5.1 Desenrolamento do ângulo

A leitura absoluta é cíclica e retorna ao início depois de uma volta:

```text
358° → 359° → 0° → 1°
```

O desenrolamento transforma essa sequência em uma trajetória contínua:

```text
358° → 359° → 360° → 361°
```

Isso permite identificar voltas completas e calcular corretamente a velocidade
angular.

### 5.2 Sentido e fronteiras das voltas

O sentido de rotação é estimado pela mediana das diferenças sucessivas do
ângulo desenrolado. Em seguida, o algoritmo interpola os instantes em que a
trajetória cruza cada múltiplo de uma volta completa.

São necessários pelo menos três cruzamentos consecutivos, pois eles delimitam
no mínimo duas voltas completas utilizáveis.

### 5.3 Trajetória angular ideal

Entre dois cruzamentos consecutivos, o algoritmo assume progressão angular
uniforme. Se uma volta começa no instante \(t_0\) e termina em \(t_1\), sua
fração ideal é:

\[
f(t)=\frac{t-t_0}{t_1-t_0}
\]

e o ângulo ideal avança linearmente por \(2\pi\) radianos durante esse
intervalo.

O erro angular usado para construir a LUT é:

\[
e(\theta)=\theta_{\mathrm{ideal}}-\theta_{\mathrm{medida}}
\]

O erro é normalizado para o intervalo angular principal e associado à fase
bruta na qual foi observado.

Essa construção é feita separadamente em cada volta. Portanto, pequenas
variações lentas da velocidade média não são interpretadas diretamente como
erro periódico do sensor.

### 5.4 Treinamento e validação

As voltas completas são divididas em dois conjuntos:

- voltas ímpares são usadas para calcular a LUT;
- voltas pares são reservadas para validação independente.

Assim, a redução de erro informada ao usuário não é avaliada somente sobre os
mesmos dados empregados no ajuste da tabela.

### 5.5 Agrupamento angular

Uma volta é dividida em `bin_count` regiões uniformes. Com a configuração atual
de 256 bins:

\[
\Delta\theta_{bin}=\frac{360^\circ}{256}=1{,}40625^\circ
\]

O algoritmo calcula a mediana dos erros observados em cada bin. A mediana reduz
a influência de ruído impulsivo e de amostras isoladas.

Bins sem observações são preenchidos por interpolação circular. O primeiro e o
último bin são tratados como vizinhos, respeitando a natureza periódica do
ângulo. Pelo menos um quarto dos bins deve ter observações diretas; caso
contrário, a calibração é rejeitada.

### 5.6 Suavização harmônica por FFT

O perfil medido contém tanto a imperfeição angular determinística quanto ruído.
Para preservar principalmente a parcela periódica de baixa ordem, o Octave:

1. calcula a FFT do erro em função do ângulo;
2. preserva a componente contínua e os harmônicos positivos e negativos de
   baixa ordem;
3. remove os componentes espaciais de frequência mais alta;
4. reconstrói o perfil suavizado por meio da transformada inversa.

O algoritmo começa preservando até oito harmônicos. Se a tabela resultante não
mantiver a relação angular monotônica, a quantidade de harmônicos é reduzida
progressivamente.

Depois da suavização, a componente média da correção é removida. A LUT corrige
a não linearidade periódica, mas não redefine o zero mecânico escolhido pela
aplicação.

### 5.7 Nós da tabela e quantização

O erro é inicialmente estimado no centro de cada bin. Como o firmware interpola
a correção nas fronteiras dos bins, o Octave converte as estimativas centrais
para esses nós antes da quantização.

A correção angular é então convertida para contagens nativas:

\[
c_k=\operatorname{round}\left(e_k\frac{N}{2\pi}\right)
\]

onde \(N\) é a quantidade de contagens por volta. Para um sensor de 14 bits:

\[
N=2^{14}=16384
\]

Na configuração atual, a tabela contém 256 correções com sinal, armazenadas
como `int16_t`.

## 6. Validação da LUT

Antes da exportação, o Octave verifica:

- se a quantidade de bins é compatível com a resolução;
- se a resolução é uma potência de dois entre 256 e 65536;
- se a resolução é divisível pela quantidade de bins;
- se todas as correções são inteiras e representáveis como `int16_t`;
- se nenhuma correção supera o limite configurado no firmware;
- se a transformação angular corrigida permanece monotônica;
- se nenhum passo corrigido é excessivamente grande.

Para bins consecutivos, o passo corrigido é:

\[
\Delta_k=\frac{N}{B}+c_{k+1}-c_k
\]

onde \(B\) é a quantidade de bins. O firmware exige:

\[
1\leq\Delta_k\leq4\frac{N}{B}
\]

Essa regra impede que a LUT dobre, inverta ou comprima completamente o mapa
angular.

Depois da validação, as correções são serializadas como inteiros de 16 bits em
ordem little-endian e é calculado um CRC32 IEEE do payload.

## 7. Métricas e arquivo MAT

A validação compara o erro mediano reservado com a correção interpolada nos
mesmos ângulos. O resultado inclui:

- erro angular RMS antes da LUT;
- erro angular RMS depois da LUT;
- redução percentual do erro angular;
- ondulação RMS da velocidade instantânea antes da LUT;
- ondulação RMS da velocidade instantânea depois da LUT;
- redução percentual da ondulação de velocidade;
- quantidade de voltas e segmentos utilizados;
- quantidade de harmônicos preservados;
- correções em contagens e graus;
- payload binário e seu CRC32.

A velocidade usada nessa avaliação é obtida pela derivada do ângulo desenrolado
em cada segmento. Uma tendência linear é removida antes do cálculo RMS para que
a métrica represente principalmente a ondulação periódica, e não uma aceleração
lenta do motor.

O arquivo de calibração é salvo em formato MAT e contém duas estruturas:

```octave
capture
calibration
```

`capture` preserva os dados originais do ensaio. `calibration` contém a tabela,
os parâmetros usados, o payload, o CRC e as métricas calculadas. Portanto, é
possível reabrir o arquivo, reproduzir os gráficos e reenviar a LUT sem repetir
o ensaio.

## 8. Envio da LUT ao ESP32

Mediante confirmação do usuário, o Octave envia um cabeçalho semelhante a:

```text
CAL WRITE 256 16384 D2847242
```

Os campos representam:

1. quantidade de entradas;
2. contagens por volta;
3. CRC32 do payload.

Depois que o firmware responde que está pronto, o Octave envia os dados
binários. Para 256 entradas:

```text
256 entradas × 2 bytes = 512 bytes
```

O módulo `main/angle_lut_usb_commands.c` recebe o payload e chama:

```c
esp_angle_lut_install_detailed(corrections,
                               bin_count,
                               full_scale_counts,
                               payload_crc32,
                               &failure);
```

O componente `esp_angle_lut` repete no ESP32 as verificações críticas de
metadados, CRC, amplitude e monotonicidade. Portanto, uma falha ou versão
incorreta do programa no computador não instala automaticamente uma tabela
insegura.

## 9. Persistência e ativação

Uma LUT válida é gravada na NVS. O componente usa dois slots persistentes: a
nova tabela é escrita no slot inativo e somente depois de um commit bem-sucedido
passa a ser a tabela preferencial. Isso preserva a última calibração válida se
uma gravação for interrompida.

Uma tabela recém-instalada permanece desabilitada. O Octave executa uma leitura
de retorno e compara:

- todas as correções;
- a resolução angular;
- a quantidade de bins;
- o CRC32.

Somente depois dessa verificação ele envia:

```text
CAL ENABLE
```

O estado de habilitação também é persistido. Depois da reinicialização, o
componente procura os dois slots, escolhe a geração válida mais recente e
restaura o estado da correção.

## 10. Aplicação da correção durante o controle

No caminho de tempo real, a sequência é:

```text
leitura do sensor
        ↓
normalização para contagens da LUT
        ↓
esp_angle_lut_apply()
        ↓
ângulo corrigido
        ↓
Kalman
        ↓
velocidade estimada
        ↓
PID
```

A correção é aplicada antes do Kalman. O estimador recebe, portanto, uma
medição angular na qual a parcela determinística identificada durante a
calibração já foi reduzida.

Para uma posição entre os bins \(k\) e \(k+1\), a correção é interpolada
linearmente:

\[
c(\theta)=(1-\alpha)c_k+\alpha c_{k+1}
\]

O ângulo corrigido é:

\[
\theta_{corrigido}=\operatorname{wrap}
\left(\theta_{bruto}+c(\theta_{bruto})\right)
\]

A interpolação é circular: o vizinho posterior ao último bin é o primeiro bin.
O resultado também é normalizado para uma volta completa.

`esp_angle_lut_apply()` não acessa a NVS, não aloca memória e não usa locks. O
componente mantém dois buffers de execução, permitindo que uma tabela nova seja
publicada sem bloquear o leitor no caminho de tempo real.

## 11. Responsabilidades dos módulos

| Módulo | Responsabilidade |
|---|---|
| Driver do sensor | Ler o ângulo absoluto e fornecer contagens nativas válidas |
| Aplicação em `main` | Executar o perfil em malha aberta e definir os canais da captura |
| `esp_timeseries_recorder` | Armazenar amostras `int16_t` em memória interna |
| `esp_timeseries_usb_transport` | Transportar comandos, status e capturas binárias |
| `main/angle_lut_usb_commands.c` | Implementar os comandos de calibração específicos da aplicação |
| `tools/octave/ts_calculate_angle_lut.m` | Encontrar os patamares estáveis desta aplicação |
| Biblioteca Octave do `esp_angle_lut` | Calcular, validar, aplicar, empacotar e representar a LUT |
| `esp_angle_lut` no ESP32 | Validar, persistir e aplicar a correção angular |
| Kalman | Estimar ângulo, velocidade e aceleração a partir do ângulo corrigido |

Essa divisão permite trocar o sensor ou o perfil do ensaio sem reimplementar o
algoritmo matemático da LUT. Para um novo sensor, normalmente são necessários
um driver, a configuração correta de contagens por volta e uma adaptação da
aplicação para registrar o ângulo bruto. O cálculo genérico, a validação, a
persistência e a interpolação permanecem os mesmos.
