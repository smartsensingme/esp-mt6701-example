# Saturação de atuadores e anti-windup por back-calculation

## 1. Objetivo

Este texto apresenta um problema que aparece quando um controlador PID deixa o
papel e passa a comandar um sistema físico: **a ação de controle disponível é
limitada**. Uma ponte H não pode fornecer tensão infinita, uma válvula não abre
mais que 100%, um aquecedor possui potência máxima e uma fonte não entrega
corrente ilimitada.

Quando esses limites não são considerados, o termo integral pode acumular uma
ação impossível de aplicar. Esse fenômeno é chamado **integral windup**. Seus
efeitos mais comuns são overshoot excessivo, recuperação lenta após uma
saturação, respostas muito diferentes para degraus ascendentes e descendentes
e, em sistemas não lineares, oscilações persistentes.

O objetivo deste capítulo é explicar:

- por que a saturação modifica o comportamento esperado do PID;
- como reconhecer o windup em dados experimentais;
- como funciona o anti-windup por **back-calculation**;
- como escolher seus parâmetros e implementá-lo em tempo discreto;
- quais limitações permanecem quando o atuador possui modos como acionamento,
  frenagem e roda livre.

O motor DC deste projeto será usado como exemplo, mas o método se aplica a
diversos sistemas de instrumentação e controle.

## 2. O PID ideal e o atuador real

Na forma paralela, um controlador PID contínuo pode ser escrito como

\[
u_{raw}(t)=K_p e(t)+I(t)+K_d\frac{de(t)}{dt},
\]

com

\[
\frac{dI(t)}{dt}=K_i e(t),
\]

em que:

- \(r(t)\) é a referência;
- \(y(t)\) é a variável medida;
- \(e(t)=r(t)-y(t)\) é o erro;
- \(u_{raw}(t)\) é a ação solicitada pelo controlador;
- \(I(t)\) é o estado do integrador.

Essas equações, isoladamente, permitem que \(u_{raw}\) assuma qualquer valor.
O atuador físico, porém, somente consegue aplicar uma ação dentro de um
intervalo:

\[
u_{min}\leq u(t)\leq u_{max}.
\]

Consequentemente, a ação efetivamente aplicada é

\[
u_{sat}(t)=\operatorname{sat}\left(u_{raw}(t),u_{min},u_{max}\right).
\]

A função de saturação é definida por

\[
\operatorname{sat}(x,a,b)=
\begin{cases}
a, & x<a,\\
x, & a\leq x\leq b,\\
b, & x>b.
\end{cases}
\]

Na configuração atual do controle de velocidade deste projeto, o comando está
limitado a

\[
-100\%\leq u_{sat}\leq100\%.
\]

Assim, o PID pode solicitar 130%, mas a ponte somente recebe 100%. Da mesma
forma, uma solicitação de -120% é limitada a -100%.

## 3. Como nasce o integral windup

Considere um motor inicialmente parado e uma referência elevada. Enquanto a
velocidade estiver muito abaixo da referência, o erro será positivo e o PID
poderá solicitar mais de 100%:

\[
u_{raw}=130\%,\qquad u_{sat}=100\%.
\]

O erro permanece positivo porque o motor precisa de tempo para acelerar. Em um
PID sem anti-windup, o integrador continua crescendo durante todo esse
intervalo, embora o atuador já esteja no máximo e não possa usar a ação
adicional.

Quando a velocidade finalmente se aproxima da referência, o termo
proporcional diminui, mas o integrador pode ter acumulado um valor muito alto.
O controlador continua solicitando uma ação elevada, a velocidade ultrapassa a
referência e somente então o erro negativo começa a descarregar o integrador.

Esse acúmulo não representa energia armazenada no atuador. É uma **memória
numérica incompatível com o que o atuador conseguiu realizar**.

O mesmo problema aparece na saturação inferior. Se o PID solicitar -20%, mas o
menor comando disponível for 0%, integrar ainda mais erro negativo apenas torna
a solicitação impossível mais negativa.

### 3.1 Windup não é sinônimo de overshoot

Um sistema pode apresentar overshoot mesmo com \(K_i=0\). Inércia, atraso,
ganho proporcional elevado e polos pouco amortecidos também produzem uma
resposta oscilatória. Portanto:

- **overshoot** é uma característica observada na resposta;
- **windup** é o acúmulo inadequado do estado integral durante uma limitação;
- o windup pode aumentar o overshoot, mas nem todo overshoot é causado por ele.

Essa distinção é importante no laboratório. Reduzir \(K_i\) não corrige uma
oscilação cuja origem principal seja ganho proporcional excessivo, atraso de
medição ou comutação brusca do atuador.

## 4. Por que limitar somente a saída não basta

Uma primeira implementação costuma ser:

```c
integral += ki * error * dt;
raw_output = kp * error + integral + derivative;
output = clamp(raw_output, minimum, maximum);
```

O `clamp()` protege o atuador, mas não protege o estado interno do controlador.
O valor enviado permanece entre os limites, enquanto `integral` pode crescer
continuamente.

Também não é suficiente limitar arbitrariamente o integrador a, por exemplo,
`-100%` e `100%`. Os limites corretos do termo integral dependem dos termos
proporcional e derivativo. Se \(P=70\%\), bastam mais 30% para atingir o limite
superior. Um limite fixo para \(I\) não representa essa interação.

## 5. Técnicas usuais de anti-windup

Existem várias estratégias para impedir ou corrigir o acúmulo integral.

### 5.1 Limitação do integrador

O estado integral é confinado a um intervalo predeterminado. É simples e útil
como proteção numérica, mas não considera os demais termos do PID.

### 5.2 Integração condicional

O integrador é congelado quando a saída está saturada e o erro empurraria o
controlador ainda mais para fora dos limites. A integração volta a ocorrer
quando o erro ajuda a retirar o controlador da saturação.

Essa técnica é eficiente e fácil de compreender, mas a transição entre
“integrar” e “congelar” é abrupta. Além disso, o estado integral apenas deixa de
piorar; ele não é conduzido explicitamente ao valor compatível com a saída
realizável.

### 5.3 Back-calculation

O back-calculation compara a saída solicitada com a saída efetivamente
disponível. A diferença retorna ao integrador por uma realimentação adicional.
Assim, o integrador acompanha gradualmente o estado que seria compatível com o
atuador.

Essa estratégia é especialmente útil quando se deseja uma recuperação suave e
parametrizável após a saturação.

## 6. Equação do back-calculation

Primeiro são calculadas a saída solicitada e a saída limitada:

\[
u_{raw}=P+I+D,
\]

\[
u_{sat}=\operatorname{sat}(u_{raw},u_{min},u_{max}).
\]

Define-se o erro de rastreamento do atuador como

\[
e_t=u_{sat}-u_{raw}.
\]

O integrador passa a obedecer a

\[
\boxed{
\frac{dI}{dt}=K_i e+\frac{u_{sat}-u_{raw}}{T_t}
}
\]

ou, equivalentemente,

\[
\frac{dI}{dt}=K_i e+K_{aw}(u_{sat}-u_{raw}),
\qquad K_{aw}=\frac{1}{T_t}.
\]

O parâmetro \(T_t\), medido em segundos, é chamado **tempo de rastreamento**.
Ele determina quão rapidamente o integrador é corrigido durante a saturação.

O fluxo de sinais pode ser visualizado como

```text
referência -> PID -> u_raw -> saturação -> u_sat -> atuador -> planta
                       \                    /
                        +-> [u_sat-u_raw] <-+
                                   |
                                   v
                         integrador do PID
```

A realimentação inferior não mede o erro da planta. Ela mede a diferença entre
o que o controlador pediu e o que o atuador pôde receber.

### 6.1 Operação dentro dos limites

Quando o controlador não está saturado,

\[
u_{sat}=u_{raw}.
\]

Portanto,

\[
u_{sat}-u_{raw}=0
\]

e a equação se reduz ao integrador comum:

\[
\frac{dI}{dt}=K_i e.
\]

O back-calculation não altera o PID enquanto o atuador consegue realizar a
ação solicitada.

### 6.2 Saturação superior

Suponha que

\[
u_{raw}=120\%,\qquad u_{sat}=100\%.
\]

Então,

\[
e_t=100-120=-20\%.
\]

A parcela de back-calculation é negativa e reduz o integrador. Isso traz a
saída solicitada de volta em direção ao limite de 100%.

### 6.3 Saturação inferior

Agora suponha que

\[
u_{raw}=-18\%,\qquad u_{sat}=0\%.
\]

Logo,

\[
e_t=0-(-18)=+18\%.
\]

A parcela positiva aumenta o integrador e conduz a saída solicitada de -18%
em direção ao limite realizável de 0%. Esse sinal pode parecer
contraintuitivo, mas impede que o controlador continue acumulando uma ação
negativa que o atuador não consegue aplicar.

## 7. Exemplo: motor DC entre 900 e 600 rpm

No regime de 900 rpm, suponha que o motor necessite de aproximadamente 57% de
ação para vencer as perdas mecânicas. Esse valor fica armazenado no integrador:

\[
I\approx57\%.
\]

Quando a referência muda instantaneamente de 900 para 600 rpm, o erro inicial
é aproximadamente -300 rpm. Com \(K_p=0{,}25\ \%/\text{rpm}\),

\[
P=0{,}25(-300)=-75\%.
\]

Desprezando momentaneamente a derivada,

\[
u_{raw}=P+I=-75+57=-18\%.
\]

Como o limite inferior é zero,

\[
u_{sat}=0\%.
\]

Para \(T_t=0{,}20\) s, a contribuição do back-calculation é

\[
\frac{u_{sat}-u_{raw}}{T_t}
=\frac{18}{0{,}20}
=90\ \%/s.
\]

Essa contribuição atua para tornar o estado integral compatível com a
saturação inferior. A integração normal, \(K_i e\), continua existindo e pode
ter sinal contrário. Por isso, o back-calculation não torna aceitável um
\(K_i\) arbitrariamente grande: os dois termos devem ter escalas coerentes.

Por exemplo, com \(K_i=10\ \%/(\text{rpm}\cdot s)\) e erro de -300 rpm,

\[
K_i e=-3000\ \%/s.
\]

Esse valor é muito maior que os 90 %/s da correção de rastreamento. O
anti-windup ajuda durante a saturação, mas não substitui uma sintonia adequada
do controlador.

## 8. Implementação em tempo discreto

Considere um período de amostragem \(T_s\). Pelo método de Euler explícito, uma
implementação possível é

\[
I[k]=I[k-1]+K_i e[k]T_s
+\frac{T_s}{T_t}\left(u_{sat}[k]-u_{raw}[k]\right).
\]

Uma sequência prática de cálculo é:

1. calcular os termos proporcional e derivativo;
2. integrar normalmente o erro, obtendo um integrador candidato;
3. calcular a saída candidata;
4. limitar essa saída aos valores realizáveis;
5. calcular a diferença entre saída limitada e candidata;
6. aplicar essa diferença ao integrador;
7. recalcular e limitar a saída final.

### 8.1 Código C reutilizável

O exemplo abaixo usa a forma paralela do PID. As unidades dos ganhos devem ser
compatíveis com as unidades escolhidas para erro, saída e tempo.

```c
typedef struct {
    float integral;
    float previous_measurement;
    bool previous_measurement_valid;
} pid_state_t;

static float clamp(float value, float minimum, float maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

float pid_update(pid_state_t *state,
                 float reference,
                 float measurement,
                 float kp,
                 float ki,
                 float kd,
                 float tracking_time,
                 float output_min,
                 float output_max,
                 float dt)
{
    float error = reference - measurement;
    float proportional = kp * error;

    float measurement_derivative = 0.0f;
    if (state->previous_measurement_valid && dt > 0.0f) {
        measurement_derivative =
            (measurement - state->previous_measurement) / dt;
    }
    state->previous_measurement = measurement;
    state->previous_measurement_valid = true;

    /* Derivada na medição: evita derivative kick no degrau da referência. */
    float derivative = -kd * measurement_derivative;

    float candidate_integral = state->integral + ki * error * dt;

    float requested =
        proportional + candidate_integral + derivative;
    float realizable = clamp(requested, output_min, output_max);

    float tracking_error = realizable - requested;
    candidate_integral += tracking_error * dt / tracking_time;
    state->integral = candidate_integral;

    requested = proportional + state->integral + derivative;
    return clamp(requested, output_min, output_max);
}
```

Em uma aplicação real, devem ser verificados pelo menos:

- ponteiros válidos;
- `dt > 0` e um limite máximo plausível para `dt`;
- `tracking_time >= 0`, adotando zero apenas quando se deseja desabilitar o
  back-calculation;
- valores finitos nas medições e nos estados;
- inicialização ou reinicialização coerente do integrador;
- proteção numérica adicional para situações de falha prolongada.

O código deste projeto está em
[`components/esp_pid/esp_pid.c`](components/esp_pid/esp_pid.c), configurado por
[`main/motor_controller.c`](main/motor_controller.c). A malha de velocidade usa
\(T_t=0{,}20\) s e executa no loop de controle de 1 kHz.

O cálculo da derivada foi mantido simples nesse exemplo para destacar o
anti-windup. Em uma aplicação prática, a derivada da medição normalmente deve
ser filtrada, pois a diferenciação amplifica ruído e quantização. Esse filtro é
independente da realimentação de back-calculation.

### 8.2 O que usar como saída realizável

O sinal realimentado no back-calculation deve representar a ação que o atuador
**realmente pode aplicar**, depois de todas as limitações relevantes. Isso pode
incluir:

- saturação de amplitude;
- limite de corrente;
- limite de tensão;
- limite de taxa de variação;
- zona morta;
- bloqueio por temperatura;
- redução de potência pela fonte;
- prioridades impostas por outro controlador.

Se o software limita o PID em 100%, mas outro subsistema reduz o comando a 60%,
usar 100% como `u_sat` ainda deixa uma diferença entre o modelo do controlador
e a ação física. Quando possível, o back-calculation deve receber os 60% que
foram realmente autorizados.

## 9. Escolha do tempo de rastreamento

O efeito de \(T_t\) pode ser resumido assim:

| Valor de \(T_t\) | Comportamento esperado |
|---:|---|
| muito grande | correção lenta; comportamento próximo ao PID sem back-calculation |
| intermediário | recuperação suave e redução do windup |
| muito pequeno | correção agressiva, sensível à discretização e às comutações do atuador |

Não existe um valor universal. Uma escolha inicial deve considerar:

- período de amostragem;
- tempo integral \(T_i=K_p/K_i\), quando \(K_i>0\);
- constantes de tempo da planta;
- duração típica das saturações;
- ruído e descontinuidades do atuador.

Como regra de implementação, \(T_t\) deve ser significativamente maior que o
período de amostragem. Em um loop de 1 kHz, \(T_s=1\) ms; valores como 0,1 s,
0,2 s ou 0,5 s são numericamente muito mais confortáveis que um valor próximo
de 1 ms.

Um procedimento experimental razoável é:

1. sintonizar inicialmente \(K_p\), \(K_i\) e \(K_d\) sem saturações longas;
2. começar com um \(T_t\) moderado;
3. aplicar degraus que provoquem saturação de forma controlada;
4. reduzir \(T_t\) se a recuperação ainda for lenta;
5. aumentar \(T_t\) se a correção provocar transições abruptas ou oscilações;
6. repetir os testes nos limites superior e inferior.

Neste projeto, \(T_t=0{,}20\) s é um ponto inicial experimental, não uma
constante universal para motores.

### 9.1 Caso sem ação integral

Se \(K_i=0\) for usado para estudar um controlador P ou PD puro, normalmente o
estado integral deve permanecer em zero e o back-calculation deve ser
desabilitado. Manter a realimentação de rastreamento sobre um estado integral
com \(K_i=0\) cria uma memória dinâmica que já não corresponde a um P ou PD
puro. Isso pode ser desejado em estruturas especiais de rastreamento, mas deve
ser uma decisão explícita.

## 10. Particularidades da ponte H deste projeto

No modelo mais simples, `0%` representa ação nula. Entretanto, a aplicação
atual possui a seguinte política:

```text
saída maior que 0%  -> acionamento PWM para frente
saída menor que 0%  -> acionamento PWM reverso
saída igual a 0%   -> freio dinâmico
```

No freio dinâmico, os terminais do motor são conectados de forma a dissipar sua
energia eletromecânica e produzir torque contrário ao movimento. Portanto,
`0%` não significa necessariamente torque zero. Existe uma mudança descontínua
entre um pequeno comando, em qualquer sentido, e o freio completo. Além disso,
se o motor ainda gira para frente, um comando negativo aplica contracorrente e
pode produzir uma corrente significativamente maior que a frenagem dinâmica.

O back-calculation continua útil porque corrige a inconsistência numérica da
saturação. Porém, ele não transforma essa comutação física em uma ação
contínua. Se ainda houver um ciclo-limite próximo de zero, podem ser necessárias
técnicas adicionais:

- histerese para entrar e sair do modo de frenagem;
- tempo mínimo ou máximo de permanência no freio;
- frenagem PWM com intensidade controlável;
- limites distintos para o acionamento positivo e o torque reverso;
- ganhos diferentes para aceleração e desaceleração;
- máquina de estados que represente `DRIVE`, `BRAKE` e `COAST`.

Em um controlador com ação assinada, o sinal usado pelo anti-windup deve ser
expresso preferencialmente em uma grandeza relacionada ao torque realizável, e
não apenas no valor numérico do PWM.

## 11. Saturação assimétrica e respostas diferentes

Em muitos sistemas, os limites positivo e negativo não são equivalentes:

- um aquecedor fornece calor, mas não refrigera;
- uma bomba aumenta a pressão, mas não a reduz ativamente;
- uma válvula abre entre 0 e 100%, sem posição negativa;
- um motor acelera com PWM contínuo, mas freia por outro circuito;
- uma fonte fornece corrente, mas talvez não absorva energia regenerada.

Por isso, um degrau ascendente pode parecer bem controlado enquanto um degrau
descendente produz grande undershoot ou oscilação. O fenômeno não implica que o
algoritmo “falhou depois do terceiro degrau”; pode simplesmente ser a primeira
vez que o ensaio exigiu o outro lado do atuador.

O anti-windup deve ser testado em todas as regiões de saturação e em todas as
transições fisicamente distintas.

## 12. Inicialização e transferência sem descontinuidade

O integrador não precisa ser zero em regime permanente. Se um motor necessita
de 40% para manter 600 rpm, um termo integral próximo de 40% pode ser exatamente
o valor correto para eliminar o erro estacionário.

Zerar o integrador a cada mudança de referência produz uma descontinuidade na
ação de controle. Em geral, o estado deve ser preservado durante mudanças
normais de referência. Já ao transferir entre controle manual e automático,
pode ser desejável inicializar o integrador para que

\[
I=u_{atual}-P-D.
\]

Essa inicialização é chamada transferência **bumpless**, pois evita um salto na
saída ao mudar o modo de operação. Back-calculation e transferência bumpless
são técnicas relacionadas pela ideia de fazer o estado interno acompanhar a
ação realmente aplicada.

## 13. Como identificar o problema experimentalmente

Para estudar saturação e windup, recomenda-se registrar simultaneamente:

- referência \(r\);
- variável medida \(y\);
- erro \(e\);
- termos \(P\), \(I\) e \(D\);
- saída solicitada \(u_{raw}\);
- saída limitada \(u_{sat}\);
- estado do atuador, como `DRIVE`, `BRAKE` ou `COAST`;
- corrente, tensão e sinais de falha relevantes.

Indícios de windup incluem:

- `u_raw` continua se afastando do limite enquanto `u_sat` permanece constante;
- o integrador cresce durante uma saturação prolongada;
- o erro muda de sinal, mas a saída demora a abandonar a saturação;
- a recuperação após retirar uma perturbação é muito lenta;
- a resposta depende fortemente do estado integral acumulado antes do degrau.

Já uma oscilação com `u_raw` sempre dentro dos limites deve levar à investigação
de ganho de malha, atrasos, ruído, quantização e dinâmica da planta, e não
somente de windup.

## 14. Roteiro de laboratório

Um ensaio didático pode ser organizado em quatro etapas.

### Etapa 1 — PID sem saturação relevante

Use degraus pequenos e registre a resposta. Verifique que `u_raw` e `u_sat` são
praticamente iguais. Nessa condição, ativar o back-calculation não deve alterar
significativamente o comportamento.

### Etapa 2 — Saturação sem anti-windup

Aplique um degrau suficientemente grande para alcançar 100%, mantendo limites
seguros para o equipamento. Observe o crescimento do integrador e meça o tempo
necessário para a saída abandonar a saturação depois que o erro muda de sinal.

### Etapa 3 — Integração condicional

Repita o ensaio congelando o integrador quando o erro empurrar a saída para fora
dos limites. Compare overshoot, tempo de recuperação e estado integral.

### Etapa 4 — Back-calculation

Repita o ensaio com dois ou três valores de \(T_t\). Compare:

- máximo overshoot;
- integral do erro absoluto;
- tempo em saturação;
- tempo de acomodação;
- continuidade da ação de controle;
- comportamento nos degraus ascendente e descendente.

O teste deve respeitar os limites elétricos, térmicos e mecânicos do conjunto.
Provocar saturação para fins didáticos não significa operar o motor, a ponte ou
a fonte fora de suas especificações.

## 15. Erros comuns de implementação

1. **Limitar a saída e ignorar o integrador.** O atuador fica protegido, mas o
   estado interno continua acumulando.
2. **Usar a saída solicitada como se fosse a aplicada.** O rastreamento deve
   considerar todas as limitações posteriores ao PID.
3. **Escolher \(T_t\) próximo ou menor que o período de amostragem.** A correção
   discreta pode ficar excessivamente agressiva.
4. **Usar back-calculation para compensar ganhos mal sintonizados.** Anti-windup
   trata principalmente a não linearidade de saturação; não substitui a
   sintonia do PID.
5. **Ignorar limites assimétricos.** Aceleração e frenagem podem representar
   dinâmicas físicas diferentes.
6. **Confundir PWM zero com torque zero.** O estado elétrico da ponte precisa
   ser conhecido.
7. **Não reinicializar estados após falhas.** Medições inválidas, pausas longas
   e mudanças de modo exigem uma política explícita.
8. **Observar somente a saída limitada.** Sem registrar `u_raw` e \(I\), o
   windup pode permanecer escondido.

## 16. Síntese

O controlador PID ideal calcula uma ação sem conhecer, por si só, os limites do
atuador. Quando a ação solicitada não pode ser realizada, a saída aplicada e o
estado interno do controlador deixam de representar a mesma realidade. Se o
integrador continuar acumulando erro, ocorre windup.

O back-calculation fecha uma malha adicional em torno dessa diferença:

\[
\frac{dI}{dt}=K_i e+\frac{u_{sat}-u_{raw}}{T_t}.
\]

Dentro dos limites, o termo adicional é zero. Durante a saturação, ele conduz o
integrador ao estado compatível com a ação realizável. A técnica reduz memória
indevida, overshoot associado à saturação e demora para recuperar o controle.

Seu uso correto exige, entretanto, compreender o atuador. Saturação de tensão,
limitação de corrente, zona morta, frenagem, roda livre e proteção térmica são
partes da planta real. Um bom controlador embarcado não controla apenas uma
equação: ele coordena o modelo matemático com aquilo que o hardware é capaz de
fazer.

## 17. Questões para estudo

1. Por que limitar somente `u_raw` não impede o windup?
2. Qual é o sinal do termo de back-calculation quando a saída satura em 100%?
3. Por que um valor menor de \(T_t\) produz uma correção mais rápida?
4. Um sistema com \(K_i=0\) ainda pode apresentar overshoot? Justifique.
5. Por que o degrau de 900 para 600 rpm pode ser mais difícil que o degrau de
   600 para 900 rpm?
6. Como a limitação de corrente de uma fonte deve entrar no cálculo da saída
   realizável?
7. Que diferença existe entre corrigir windup e melhorar o amortecimento da
   malha linear?
8. Como seria definida `u_sat` em um sistema com comando de torque entre -100%
   e +100%?

## 18. Leituras recomendadas

- Karl J. Åström e Tore Hägglund, *Advanced PID Control*.
- Antonio Visioli, *Practical PID Control*.
- Gene F. Franklin, J. David Powell e Abbas Emami-Naeini, *Feedback Control of
  Dynamic Systems*.
