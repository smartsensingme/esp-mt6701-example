close all; % Avisa ao Octave que isto é um script, não um arquivo de função

dados = load("calibration_capture_20260902_115128.mat");
capture = dados.capture;
calibration = dados.calibration;

function index = channel_index (capture, name)
  index = optional_channel_index (capture, name);
  if (isempty (index))
    error ("Capture does not contain the required '%s' channel", name);
  endif
endfunction

function index = optional_channel_index (capture, name)
  names = {capture.channels.name};
  index = find (strcmpi (names, name), 1);
endfunction

function [angle_estimated, speed_rpm, acceleration] = ...
    kalman_3d_angle_replay (measured_angle, time_s, q_theta, q_omega, ...
                            q_alpha, measurement_r)
  % Reproduz o modelo de aceleração angular constante implementado em
  % main/engine_angle_kalman.c. O estado é [ângulo; velocidade; aceleração],
  % enquanto a única medida direta é o ângulo circular.
  measured_angle = double (measured_angle(:));
  time_s = double (time_s(:));
  sample_count = numel (measured_angle);

  angle_estimated = NaN (sample_count, 1);
  speed_rpm = NaN (sample_count, 1);
  acceleration = NaN (sample_count, 1);

  x = [measured_angle(1); 0; 0];
  P = eye (3);
  angle_estimated(1) = x(1);
  speed_rpm(1) = 0;
  acceleration(1) = 0;

  for sample = 2:sample_count
    dt = time_s(sample) - time_s(sample - 1);
    if (! isfinite (dt) || dt <= 0 || dt >= 0.1 || ...
        ! isfinite (measured_angle(sample)))
      angle_estimated(sample) = x(1);
      speed_rpm(sample) = x(2) / 6;
      acceleration(sample) = x(3);
      continue;
    endif

    % Predição: mesmas equações expandidas usadas no firmware.
    h = 0.5 * dt * dt;
    x_predicted = [x(1) + x(2) * dt + x(3) * h;
                   x(2) + x(3) * dt;
                   x(3)];

    a0 = P(1, 1) + dt * P(2, 1) + h * P(3, 1);
    a1 = P(1, 2) + dt * P(2, 2) + h * P(3, 2);
    a2 = P(1, 3) + dt * P(2, 3) + h * P(3, 3);
    a4 = P(2, 2) + dt * P(3, 2);
    a5 = P(2, 3) + dt * P(3, 3);

    P_predicted = zeros (3, 3);
    P_predicted(1, 1) = a0 + dt * a1 + h * a2 + q_theta;
    P_predicted(1, 2) = a1 + dt * a2;
    P_predicted(1, 3) = a2;
    P_predicted(2, 1) = P_predicted(1, 2);
    P_predicted(2, 2) = a4 + dt * a5 + q_omega;
    P_predicted(2, 3) = a5;
    P_predicted(3, 1) = P_predicted(1, 3);
    P_predicted(3, 2) = P_predicted(2, 3);
    P_predicted(3, 3) = P(3, 3) + q_alpha;

    % Correção: a inovação circular evita um salto artificial em 360 -> 0°.
    innovation = mod (measured_angle(sample) - x_predicted(1) + 180, 360) - 180;
    innovation_covariance = P_predicted(1, 1) + measurement_r;
    gain = P_predicted(:, 1) / innovation_covariance;
    x = x_predicted + gain * innovation;
    x(1) = mod (x(1), 360);

    % P = (I - K*H)*P_pred, com H = [1 0 0], como no código C.
    P(1, 1) = (1 - gain(1)) * P_predicted(1, 1);
    P(1, 2) = (1 - gain(1)) * P_predicted(1, 2);
    P(1, 3) = (1 - gain(1)) * P_predicted(1, 3);
    P(2, 1) = P(1, 2);
    P(2, 2) = P_predicted(2, 2) - gain(2) * P_predicted(1, 2);
    P(2, 3) = P_predicted(2, 3) - gain(2) * P_predicted(1, 3);
    P(3, 1) = P(1, 3);
    P(3, 2) = P(2, 3);
    P(3, 3) = P_predicted(3, 3) - gain(3) * P_predicted(1, 3);

    angle_estimated(sample) = x(1);
    speed_rpm(sample) = x(2) / 6; % deg/s para rpm: 60/360 = 1/6.
    acceleration(sample) = x(3);
  endfor
endfunction

function statistics = speed_statistics (time_s, speed_rpm, window_s)
  % Quantifica somente a janela estacionária. Além do desvio em torno da
  % média, remove uma reta para separar a ondulação rápida de uma deriva lenta.
  selected = find (time_s >= window_s(1) & time_s <= window_s(2) & ...
                   isfinite (speed_rpm));
  if (numel (selected) < 3)
    error ("A janela estatística contém menos de três amostras válidas");
  endif

  selected_time = time_s(selected);
  selected_speed = speed_rpm(selected);
  trend_coefficients = polyfit (selected_time, selected_speed, 1);
  residual = selected_speed - polyval (trend_coefficients, selected_time);

  statistics.mean_rpm = mean (selected_speed);
  statistics.std_rpm = std (selected_speed, 1);
  statistics.ripple_rms_rpm = sqrt (mean (residual .^ 2));
  statistics.ripple_peak_to_peak_rpm = max (residual) - min (residual);
  statistics.sample_count = numel (selected);
endfunction

function print_speed_statistics (labels, statistics, window_s, filename)
  % Mostra uma tabela copiável no terminal e salva os mesmos números em CSV.
  printf ("\nEstatísticas de velocidade entre %.3f e %.3f s\n", ...
          window_s(1), window_s(2));
  printf ("%-14s %11s %11s %13s %13s %8s\n", "Etapa", "média", ...
          "desvio", "RMS ondul.", "pico a pico", "N");
  printf ("%-14s %11s %11s %13s %13s %8s\n", "", "[rpm]", "[rpm]", ...
          "[rpm]", "[rpm]", "");

  file = fopen (filename, "w");
  if (file < 0)
    error ("Não foi possível criar %s", filename);
  endif
  unwind_protect
    fprintf (file, "etapa,media_rpm,desvio_padrao_rpm,rms_ondulacao_rpm,pico_a_pico_ondulacao_rpm,amostras\n");
    for index = 1:numel (statistics)
      item = statistics(index);
      printf ("%-14s %11.3f %11.3f %13.3f %13.3f %8d\n", ...
              labels{index}, item.mean_rpm, item.std_rpm, ...
              item.ripple_rms_rpm, item.ripple_peak_to_peak_rpm, ...
              item.sample_count);
      fprintf (file, "%s,%.9g,%.9g,%.9g,%.9g,%d\n", labels{index}, ...
               item.mean_rpm, item.std_rpm, item.ripple_rms_rpm, ...
               item.ripple_peak_to_peak_rpm, item.sample_count);
    endfor
  unwind_protect_cleanup
    fclose (file);
  end_unwind_protect

  raw_rms = statistics(1).ripple_rms_rpm;
  printf ("Redução do RMS da ondulação: LUT = %.1f%%; LUT + Kalman = %.1f%%.\n", ...
          100 * (1 - statistics(2).ripple_rms_rpm / raw_rms), ...
          100 * (1 - statistics(3).ripple_rms_rpm / raw_rms));
  printf ("Tabela salva em %s\n\n", filename);
endfunction

fontSize = 10;
fontScale = .75;
fontSizeBase = round(.75*(fontSize/fontScale));

speed_index = channel_index (capture, "speed");
current_index = channel_index (capture, "current");
control_index = channel_index (capture, "control");
reference_index = channel_index (capture, "reference");
time_s = capture.time_s;
speed = capture.values(speed_index, :);
reference = capture.values(reference_index, :);
control = capture.values(control_index, :);
current = capture.values(current_index, :);
angle_index = optional_channel_index (capture, "angle_raw");

figure(3);

plot (time_s, speed, "linewidth", 1.5);
grid on;
ylabel ("Velocidade do Eixo [rpm]",'fontsize', fontSizeBase);
xlabel("Tempo (s)",'fontsize', fontSizeBase);
xlim([min(time_s), max(time_s)]);

set(gca, 'fontname', 'Times New Roman');
set(gca, 'fontsize', fontSizeBase);
set(gca, 'linewidth', 1.0);

% 1. Fixa o tamanho da janela da figura (Largura x Altura)
set(gcf, 'Position', [100, 100, 800, 600]);

% 2. Força o 'print' a usar o tamanho exato da tela, ignorando o tamanho do papel
set(gcf, 'PaperPositionMode', 'auto');


print('FigHallEnsaioPantamares.eps', '-depsc2');

figure(4);

plot (time_s, speed, "linewidth", 1.5);
grid on;
ylabel ("Velocidade do Eixo [rpm]",'fontsize', fontSizeBase);
xlabel("Tempo (s)",'fontsize', fontSizeBase);
xlim([5, 5.4]);
ylim([581, 622]);

set(gca, 'fontname', 'Times New Roman');
set(gca, 'fontsize', fontSizeBase);
set(gca, 'linewidth', 1.0);

% 1. Fixa o tamanho da janela da figura (Largura x Altura)
set(gcf, 'Position', [100, 100, 800, 600]);

% 2. Força o 'print' a usar o tamanho exato da tela, ignorando o tamanho do papel
set(gcf, 'PaperPositionMode', 'auto');

print('FigHallEnsaioPantamaresZoom.eps', '-depsc2');

figure(5);

plot (time_s, capture.values(angle_index, :), "linewidth", 1.0);
grid on;
ylabel ("Ângulo [deg]");
xlabel("Tempo (s)",'fontsize', fontSizeBase);
xlim([5, 5.4]);
ylim([0, 360]);

set(gca, 'fontname', 'Times New Roman');
set(gca, 'fontsize', fontSizeBase);
set(gca, 'linewidth', 1.0);
set(gcf, 'Position', [100, 100, 800, 600]);
set(gcf, 'PaperPositionMode', 'auto');

print('FigHallEnsaioAngleZoom.eps', '-depsc2');

c = calibration;

% A LUT é armazenada nas fronteiras dos bins.
% Aqui ela é interpolada para os centros, onde o erro foi avaliado.
lut_centro = 0.5 * ...
  (c.correction_deg + c.correction_deg([2:end, 1]));

figure(6);

plot(c.phase_deg, c.validation_raw_error_deg, "-", "linewidth", 1.5, "markersize", 8);
hold on;
plot(c.phase_deg, lut_centro, "r-", "linewidth", 1.5);
grid on;
xlim([0, 360]);

% 1. Aplique as configurações de fonte e linha no eixo PRIMEIRO
set(gca, 'fontname', 'Times New Roman');
set(gca, 'fontsize', fontSizeBase);
set(gca, 'linewidth', 1.0);

% 2. DEPOIS crie os rótulos e a legenda
xlabel("Ângulo Bruto [graus]");
ylabel("Correção [graus]");
legend("LUT sem suavização",...
       "LUT suavizada pela FFT",...
       "location", "southwest");

set(gcf, 'Position', [100, 100, 800, 600]);
set(gcf, 'PaperPositionMode', 'auto');

print('FigHallEnsaioLUT.eps', '-depsc2');

names = {capture.channels.name};
angle_index = find(strcmpi(names, "angle_raw"), 1);

t = double(capture.time_s(:));
angle_raw = double(capture.values(angle_index, :)');

ts_load_angle_lut_tools ();
angle_corrected = esp_angle_lut_apply( ...
    angle_raw, calibration.correction_counts);

theta_raw = unwrap(angle_raw * pi / 180);
theta_corrected = unwrap(angle_corrected * pi / 180);

dt = diff(t);
time_middle = 0.5 * (t(1:end-1) + t(2:end));

speed_raw = diff(theta_raw) ./ dt * 60 / (2*pi);
speed_corrected = diff(theta_corrected) ./ dt * 60 / (2*pi);

% O firmware executa o estimador a 3 kHz e usa Q por iteração igual a
% [0.000333333, 1.666667, 33.333333], obtido dos valores de referência
% [0.001, 5, 100] multiplicados por 1000/3000. Esta captura contém ângulos a
% uma taxa menor.
% Para preservar aproximadamente o mesmo ruído de processo por segundo no
% replay do Octave, aplicamos a mesma regra 1000/taxa_de_replay. Usar aqui os
% valores numéricos de 4 kHz faria o filtro artificialmente mais lento.
replay_rate_hz = 1 / median (dt);
kalman_reference_rate_hz = 1000;
q_rate_scale = kalman_reference_rate_hz / replay_rate_hz;
kalman_q_theta = 0.001 * q_rate_scale;
kalman_q_omega = 5.0 * q_rate_scale;
kalman_q_alpha = 100.0 * q_rate_scale;
kalman_r = 0.002;

[angle_kalman, speed_lut_kalman, acceleration_kalman] = ...
    kalman_3d_angle_replay (angle_corrected, t, kalman_q_theta, ...
                            kalman_q_omega, kalman_q_alpha, kalman_r);

analysis_window_s = [5, 5.4];
statistics(1) = speed_statistics (time_middle, speed_raw, analysis_window_s);
statistics(2) = speed_statistics (time_middle, speed_corrected, ...
                                  analysis_window_s);
statistics(3) = speed_statistics (t, speed_lut_kalman, analysis_window_s);
statistics_labels = {"Sem LUT", "Com LUT", "LUT + Kalman"};
print_speed_statistics (statistics_labels, statistics, analysis_window_s, ...
                        "FigHallEnsaioFinal_estatisticas.csv");

printf ("Parâmetros do replay Kalman a %.3f Hz: Q_theta=%.9g, ", ...
        replay_rate_hz, kalman_q_theta);
printf ("Q_omega=%.9g, Q_alpha=%.9g, R=%.9g.\n", ...
        kalman_q_omega, kalman_q_alpha, kalman_r);
printf ("No firmware a 3 kHz: Q_theta=0.000333333, Q_omega=1.666667, ");
printf ("Q_alpha=33.333333 e R=0.002.\n");

figure(7);
plot(time_middle, speed_raw, ...
     "r-", "linewidth", 1.2);
hold on;
plot(time_middle, speed_corrected, ...
     "b-", "linewidth", 1.5);
grid on;

xlabel("tempo [s]");
ylabel("Velocidade [rpm]");
xlim(analysis_window_s);
ylim([561, 642]);
legend("Sem LUT", "Com LUT", "location", "northeast");
set(gca, "fontsize", fontSizeBase);

set(gcf, 'Position', [100, 100, 800, 600]);
set(gcf, 'PaperPositionMode', 'auto');

print('FigHallEnsaioFinal.eps', '-depsc2');

figure(8);
plot(time_middle, speed_corrected, ...
     "r-", "linewidth", 1.2);
hold on;
plot(t, speed_lut_kalman, ...
     "b-", "linewidth", 1.5);
grid on;

xlabel("tempo [s]");
ylabel("Velocidade [rpm]");
xlim(analysis_window_s);
ylim([576, 624]);
legend("Com LUT", "LUT + Kalman", "location", "northeast");
set(gca, "fontsize", fontSizeBase);

set(gcf, 'Position', [100, 100, 800, 600]);
set(gcf, 'PaperPositionMode', 'auto');

print('FigHallEnsaioFinalKalman.eps', '-depsc2');
