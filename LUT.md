# Calibração angular por LUT

## 1. Objetivo

Este documento explica a calibração usada para reduzir o erro angular periódico
causado, principalmente, pelo desalinhamento entre o ímã e o eixo do sensor
magnético MT6701. O procedimento foi pensado para ser reproduzível em
laboratório e compreensível por alunos de Instrumentação Eletrônica.

A calibração não substitui o alinhamento mecânico. Ela caracteriza o erro que
se repete a cada volta e armazena sua correção em uma tabela de consulta, ou
**LUT** (*lookup table*). Depois disso, cada nova leitura angular é corrigida
antes de entrar no filtro de Kalman e no controlador.

O firmware oferece dois ensaios distintos:

- `ARM <taxa>` inicia o ensaio normal em **malha fechada**, com o PID e a
  referência alternando entre 600 e 900 rpm;
- `CAL START <taxa>` inicia somente o ensaio de calibração em **malha aberta**,
  com patamares fixos de ação de controle.

Assim, calibrar o sensor não deixa os ensaios seguintes em malha aberta.

## 2. Por que aparece uma ondulação periódica na velocidade?

Considere que o ângulo medido seja

\[
\theta_m(t)=\theta(t)+e(\theta),
\]

em que \(e(\theta)\) é um erro que depende da posição angular. Se o ímã estiver
excêntrico ou inclinado, esse erro tende a se repetir a cada volta. Ao derivar o
ângulo para obter a velocidade, resulta

\[
\omega_m(t)=\frac{d\theta_m}{dt}
             =\omega(t)\left(1+\frac{de}{d\theta}\right).
\]

Mesmo que a velocidade mecânica seja quase constante, a derivada espacial do
erro produz uma ondulação na velocidade medida. A frequência dessa ondulação
aumenta com a rotação, mas sua forma permanece vinculada à posição do eixo.

Esse erro é determinístico: para uma mesma montagem, ele reaparece em posições
angulares semelhantes. Um filtro estatístico como o Kalman é muito útil contra
ruído aleatório, mas não conhece, por si só, esse mapa espacial. Aumentar a
filtragem pode atenuar a ondulação, porém também atrasa a medição e reduz a
banda do controle. A LUT ataca a causa observável: a não linearidade angular.

## 3. Visão geral do procedimento

O fluxo completo é:

```text
Octave: CAL START 500
          |
          v
ESP32: motor em malha aberta, 40% -> 55% -> 40% -> COAST
          |
          v
ESP32: coleta angle_raw, speed, current, control e reference
          |
          v
Octave: separa voltas, estima erro por ângulo e calcula 256 correções
          |
          v
Octave: valida, envia 512 bytes e confere a leitura de retorno
          |
          v
ESP32: grava em NVS e habilita a LUT
          |
          v
Ensaios futuros: ângulo bruto -> LUT -> Kalman -> PID
```

O procedimento guiado está no menu **Calibrar linearidade angular** de
[`tools/octave/ts_console.m`](tools/octave/ts_console.m). O algoritmo completo
está em
[`tools/octave/ts_calculate_angle_lut.m`](tools/octave/ts_calculate_angle_lut.m),
e a aplicação embarcada está em
[`components/esp_angle_lut/esp_angle_lut.c`](components/esp_angle_lut/esp_angle_lut.c).

## 4. Ensaio de calibração em malha aberta

### 4.1 Por que usar malha aberta durante a calibração?

Se o PID permanecesse ativo, ele enxergaria a ondulação da velocidade como um
erro real e modularia a ação de controle para combatê-la. Nesse caso, parte do
erro do sensor se transformaria em variação real de torque e velocidade,
dificultando separar uma coisa da outra.

Em malha aberta, a ação é mantida constante em cada patamar. Depois do
transitório, admite-se que a velocidade média varie lentamente durante uma
volta. Isso permite construir uma referência angular interna sem precisar de
um segundo encoder.

### 4.2 Perfil aplicado ao motor

O comando de calibração atual é `CAL START 500`. O firmware aplica:

| Intervalo | Ação de controle | Finalidade |
|---:|---:|---|
| 0 a 8 s | 40% | observar várias voltas em velocidade baixa |
| 8 a 16 s | 55% | repetir a observação em outra velocidade |
| 16 a 24 s | 40% | verificar a repetibilidade no patamar inicial |
| após 24 s | 0% — COAST | terminar o acionamento com segurança |

Os valores e a duração são configuráveis por Kconfig. O eixo deve estar livre,
sem uma perturbação externa intencional durante o ensaio.

### 4.3 Grandezas coletadas

O loop de controle registra cinco canais em palavras de 16 bits:

| Canal | Significado durante a calibração |
|---|---|
| `angle_raw` | ângulo sem a correção da LUT; é a grandeza principal |
| `control` | ação aplicada, usada para reconhecer os patamares |
| `speed` | velocidade estimada; serve para diagnóstico e comparação |
| `current` | corrente medida; ajuda a identificar perturbações e falhas |
| `reference` | vale zero em malha aberta |

O cabeçalho da captura informa escala, deslocamento e unidade de cada canal.
Portanto, embora a memória armazene `int16`, o Octave recupera os valores
físicos.

Com o buffer padrão de 128 KiB e cinco canais de 2 bytes, cabem

\[
N=\left\lfloor\frac{128\times1024}{5\times2}\right\rfloor=13107
\]

amostras. A 500 Hz isso corresponde a aproximadamente 26,21 s, suficiente para
os 24 s do perfil e seu término. A taxa de 500 Hz é a escolha atual da rotina
guiada, não uma limitação da LUT.

É importante que `angle_raw` continue realmente bruto, mesmo quando já existe
uma LUT habilitada. Assim, uma calibração nova nunca aprende novamente uma
correção antiga.

## 5. Como o Octave estima o erro angular

### 5.1 Seleção dos trechos estacionários

O programa detecta mudanças maiores que 0,2 ponto percentual em `control` e
separa os patamares. Para cada patamar útil:

- ignora os primeiros 1,5 s, que contêm o transitório;
- ignora os últimos 0,5 s;
- rejeita ação menor ou igual a 5%;
- exige ao menos 3 s de dados aproveitáveis.

São necessários pelo menos dois patamares válidos. Usar duas velocidades e
repetir 40% ajuda a revelar se o mapa é realmente espacial e repetível, em vez
de ser apenas um efeito específico de uma condição de operação.

### 5.2 Desenrolamento e identificação das voltas

O sensor fornece um ângulo circular entre 0 e 360 graus. A função `unwrap`
remove os saltos artificiais entre 359 e 0 graus e produz um ângulo acumulado.
O sinal da mediana das diferenças determina o sentido de rotação.

Em seguida, o algoritmo interpola os instantes exatos em que o ângulo acumulado
cruza múltiplos inteiros de uma volta. Somente revoluções completas são usadas.

### 5.3 Construção de uma referência angular interna

Não há um encoder de referência no equipamento. Para cada volta completa, o
algoritmo conhece o instante inicial \(t_k\) e o instante final \(t_{k+1}\). A
hipótese de calibração é que, dentro dessa volta, o movimento ideal é
aproximadamente uniforme:

\[
\theta_{ideal}(t)=\theta_k+s\,2\pi
\frac{t-t_k}{t_{k+1}-t_k},
\]

em que \(s\) vale +1 ou -1 conforme o sentido de rotação. A correção observada
em cada amostra é

\[
c(\theta_m)=\operatorname{wrap}_{[-\pi,\pi)}
\left(\theta_{ideal}-\theta_m\right).
\]

Portanto, se o sensor lê um ângulo adiantado, a correção é negativa; se lê um
ângulo atrasado, a correção é positiva.

Essa é uma **autocalibração**: ela usa a regularidade média da própria rotação.
Não alcança a rastreabilidade de um encoder angular de maior exatidão usado
como padrão. Uma oscilação mecânica real e síncrona com a posição pode ser
parcialmente confundida com erro do sensor.

### 5.4 Treinamento e validação separados

As voltas ímpares formam a LUT e as voltas pares são reservadas para validá-la.
Essa separação reduz o risco de considerar bom um mapa que apenas reproduz o
ruído das mesmas amostras usadas no cálculo.

Na configuração desta aplicação, o círculo é dividido em 256 intervalos. Como
o MT6701 tem 14 bits,

\[
16384/256=64\ \text{contagens por intervalo}.
\]

Em cada intervalo é calculada a **mediana** das correções das voltas de
treinamento. A mediana é menos sensível a amostras espúrias que a média. Os
intervalos eventualmente vazios são preenchidos por interpolação linear
circular.

### 5.5 Suavização espacial

A sequência de 256 medianas ainda contém ruído. O programa aplica uma FFT ao
mapa angular e mantém, no máximo, os oito primeiros harmônicos espaciais. Isso
preserva formas lentas e repetíveis ao longo da volta e rejeita detalhes muito
rápidos que provavelmente não podem ser identificados com segurança.

O valor médio da correção é removido. Um deslocamento angular constante muda o
zero, mas não muda a velocidade; o objetivo desta LUT é corrigir a não
linearidade ao longo da volta.

Se a tabela não satisfizer os limites de segurança, o número de harmônicos é
reduzido até que se obtenha um mapa válido.

### 5.6 Conversão para a LUT de 16 bits

Seja \(C\) o número configurado de contagens por volta. As correções em
radianos são convertidas para contagens do sensor:

\[
c_i[\text{counts}]=\operatorname{round}
\left(c_i[\text{rad}]\frac{C}{2\pi}\right).
\]

Cada entrada é um `int16_t`; 256 entradas ocupam somente 512 bytes. O firmware
impõe duas verificações principais:

- magnitude máxima configurada; no padrão do MT6701 são 1024 contagens,
  equivalentes a 22,5 graus;
- monotonicidade da transformação. Para cada par de pontos consecutivos,
  inclusive na passagem 255 para 0,

\[
0 < B+c_{i+1}-c_i \leq 4B,
\]

em que \(B=C/N\) é a largura de um intervalo e \(N\) é o número de pontos da
LUT. Para o MT6701 com 256 pontos, \(B=64\).

A primeira desigualdade impede que ângulos corrigidos invertam sua ordem. O
limite superior rejeita saltos excessivos.

## 6. Como o resultado é validado

Nas voltas pares, que não participaram do ajuste, o Octave calcula:

- RMS do erro de fase antes e depois da correção;
- RMS da ondulação da velocidade instantânea antes e depois da correção;
- redução percentual dessas duas métricas.

Para avaliar a velocidade, o ângulo bruto e o corrigido são desenrolados,
derivados e convertidos para rpm. Uma tendência linear é removida antes do RMS,
pois o interesse é a ondulação e não a velocidade média.

A interface avisa quando a redução da ondulação de velocidade é menor que 50%
ou quando o erro de fase não melhora. Um aviso não deve ser ignorado: devem ser
inspecionados o gráfico, a liberdade do eixo, a corrente e a estabilidade dos
patamares antes de instalar a tabela.

## 7. Trecho didático em Octave

O exemplo abaixo mostra a ideia central para **uma volta** já isolada. Ele não
inclui todas as verificações, separação treino/validação, interpolação circular
e tratamento dos vários patamares usados no programa real.

```octave
N = 256;                         % numero de pontos da LUT
t = capture.time_s(indices);
theta = unwrap(angle_raw_deg(indices) * pi / 180);

% Modelo ideal: uma volta uniforme entre o primeiro e o ultimo instante.
sentido = sign(median(diff(theta)));
theta_ideal = theta(1) + sentido * 2*pi * ...
              (t - t(1)) / (t(end) - t(1));

% Correcao que deve ser somada ao angulo medido.
erro = mod(theta_ideal - theta + pi, 2*pi) - pi;
fase = mod(theta, 2*pi);
indice = min(floor(fase * N / (2*pi)) + 1, N);

% Junta observacoes que pertencem a uma mesma regiao angular.
mapa = NaN(N, 1);
for k = 1:N
  mapa(k) = median(erro(indice == k));
endfor

% Suavizacao espacial: componente DC e oito harmonicos.
E = fft(mapa);
mascara = false(N, 1);
mascara(1:9) = true;
mascara((end-7):end) = true;
E(!mascara) = 0;
mapa_suave = real(ifft(E));
mapa_suave -= mean(mapa_suave);

% Valor inteiro transmitido ao ESP32.
full_scale_counts = 16384;       % MT6701; use 4096 para um sensor de 12 bits
lut_counts = int16(round(mapa_suave * full_scale_counts / (2*pi)));
```

Na implementação completa, consulte
[`ts_calculate_angle_lut.m`](tools/octave/ts_calculate_angle_lut.m). Ela calcula
a LUT a partir de várias voltas, desloca valores de centro para as fronteiras
dos intervalos, testa a monotonicidade, gera o CRC-32 e produz os gráficos de
validação.

## 8. Envio e armazenamento no ESP32

O envio usa um cabeçalho textual e uma carga binária:

1. Octave envia `CAL WRITE 256 16384 <CRC32>`; o terceiro campo informa a
   escala completa usada no cálculo;
2. ESP32 responde que está pronto;
3. Octave envia 512 bytes: 256 inteiros de 16 bits em *little endian*;
4. ESP32 recalcula o CRC-32 e valida limites e monotonicidade;
5. a tabela é gravada no slot NVS inativo, com versão e geração novas;
6. a tabela recém-gravada permanece desabilitada;
7. Octave executa `CAL READ`, compara todos os valores e o CRC;
8. somente depois da conferência envia `CAL ENABLE`.

Há dois slots na NVS. Uma gravação nova não sobrescreve diretamente o slot que
estava ativo; isso melhora a recuperação após falha de energia durante a
atualização. O estado habilitado também é persistente.

Os comandos `CAL STATUS`, `CAL READ`, `CAL ENABLE`, `CAL DISABLE` e `CAL CLEAR`
permitem consultar, recuperar, ativar, desativar e apagar a calibração.

## 9. Aplicação da correção no firmware

Para uma leitura bruta \(r\), uma escala completa \(C\) e \(N\) pontos, o
firmware encontra o intervalo e a fração dentro dele. No caso padrão
\(C=16384\), \(N=256\) e \(B=C/N=64\):

\[
i=\left\lfloor r/64\right\rfloor,\qquad f=(r\bmod64)/64.
\]

A correção é interpolada entre `LUT[i]` e `LUT[(i+1) mod 256]` e somada à
leitura. O módulo final mantém o resultado entre 0 e \(C-1\).

O trecho abaixo é uma versão didática equivalente à função embarcada:

```c
#define SENSOR_COUNTS CONFIG_ESP_ANGLE_LUT_FULL_SCALE_COUNTS
#define LUT_SIZE 256
#define BIN_WIDTH (SENSOR_COUNTS / LUT_SIZE)  /* 64 */

uint16_t correct_angle(uint16_t raw, const int16_t lut[LUT_SIZE]) {
    raw &= SENSOR_COUNTS - 1;

    uint32_t i = raw / BIN_WIDTH;
    uint32_t fraction = raw % BIN_WIDTH;
    int32_t c0 = lut[i];
    int32_t c1 = lut[(i + 1) % LUT_SIZE];
    int32_t correction = c0 + (c1 - c0) * (int32_t)fraction / BIN_WIDTH;

    int32_t corrected = ((int32_t)raw + correction) % SENSOR_COUNTS;
    if (corrected < 0) {
        corrected += SENSOR_COUNTS;
    }
    return (uint16_t)corrected;
}
```

A interpolação evita degraus de 64 contagens. A função de produção
`esp_angle_lut_apply()` também usa acesso atômico à tabela ativa, de modo que o
loop de tempo real não precisa bloquear para aplicar a correção.

## 10. A LUT entra antes ou depois do Kalman?

A correção entra **antes** do filtro de Kalman:

```text
leitura I2C de 14 bits
        -> direção e zero definidos pelo driver
        -> interpolação da LUT
        -> conversão para graus
        -> Kalman circular: ângulo, velocidade e aceleração
        -> PID de velocidade
        -> ponte de potência e motor
```

No código, essa ordem aparece em
[`main/realtime_loop.c`](main/realtime_loop.c): `esp_angle_lut_apply()` é
chamada antes de `engine_angle_kalman_3d_update()`.

Aplicar a LUT depois do Kalman corrigiria apenas a saída angular escolhida. Os
estados internos de velocidade e aceleração ainda teriam sido estimados a
partir da medição distorcida, e o PID continuaria reagindo à ondulação. Corrigir
a medição na entrada permite que todos os estados sejam estimados a partir do
ângulo linearizado.

O canal de gravação `angle_raw` é uma exceção deliberada: ele é capturado antes
da LUT para permitir diagnóstico e recalibração. Já o canal `speed` é o estado
de velocidade do Kalman alimentado pelo ângulo corrigido quando a LUT está
habilitada.

## 11. Limitações e boas práticas

- Recalibre depois de mover o ímã, sensor, suporte, eixo ou alterar a direção e
  o zero angular no software.
- Faça o ensaio com o eixo livre e sem mudanças de carga. Torque pulsante real
  sincronizado com a posição pode ser confundido com não linearidade do sensor.
- O uso de várias voltas, duas velocidades, repetição do primeiro patamar,
  mediana e validação em voltas separadas reduz essa ambiguidade, mas não a
  elimina.
- A LUT corrige erro espacial repetível. Ela não corrige amostras I2C perdidas,
  vibração aleatória, folga mecânica, ruído de corrente ou sintonia inadequada
  do Kalman e do PID.
- Uma calibração metrológica absoluta exigiria um padrão angular externo mais
  exato que o MT6701. O método atual busca principalmente reduzir a ondulação
  periódica da velocidade.
- Compare capturas com a LUT habilitada e desabilitada nas mesmas condições.
  Uma melhoria visual deve ser confirmada pelas métricas RMS e pela resposta em
  malha fechada.

## 12. Roteiro sugerido para a aula

1. Observe `angle_raw` e `speed` com a LUT desabilitada.
2. Relacione a frequência da ondulação com o número de voltas por segundo.
3. Execute a calibração guiada e identifique os três patamares de `control`.
4. Discuta a hipótese de velocidade uniforme dentro de cada volta.
5. Compare média e mediana na estimação do erro por intervalo angular.
6. Observe o espectro espacial do erro e o efeito de manter poucos harmônicos.
7. Verifique por que uma LUT não monotônica seria fisicamente indesejável.
8. Habilite a tabela e repita um `ARM 250` ou `ARM 500`: esse novo ensaio será
   em malha fechada, com referência de 600/900 rpm.
9. Compare erro, ação de controle e ondulação de velocidade antes e depois.

O componente aceita escalas em potência de dois entre 256 e 65536 contagens por
volta. Assim, o mesmo algoritmo pode trabalhar, por exemplo, com 4096 contagens
para um sensor de 12 bits, 16384 para o MT6701 ou 65536 para um sensor de 16
bits. A escala faz parte dos metadados da LUT, e o firmware rejeita uma tabela
calculada para uma resolução diferente da configurada.

Uma discussão final importante é distinguir três operações: **calibração**
(estimar o mapa), **correção** (aplicar o mapa a cada leitura) e **filtragem**
(combinar modelo e medições no tempo). Elas se complementam, mas não são a
mesma coisa.
