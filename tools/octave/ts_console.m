function ts_console ()
  if (nargin != 0)
    print_usage ();
  endif

  clc;
  fprintf ("Console de series temporais do controle ESP32\n");
  fprintf ("Carregando instrument-control...\n");
  ts_load_instrument_control ();

  while (true)
    port_name = choose_port ();
    if (isempty (port_name))
      fprintf ("Console encerrado.\n");
      return;
    endif
    if (run_port_session (port_name))
      fprintf ("Console encerrado.\n");
      return;
    endif
  endwhile
endfunction

function exit_console = run_port_session (port_name)
  exit_console = false;
  fprintf ("\nAbrindo %s...\n", port_name);
  try
    device = ts_open_serial (port_name, 30);
  catch err
    fprintf (2, "Nao foi possivel abrir a porta: %s\n", err.message);
    fprintf (2, ["Feche o monitor serial da USB nativa. Use a porta WCH ", ...
                "somente para gravacao e logs.\n"]);
    return;
  end_try_catch

  unwind_protect
    fprintf (["Porta aberta. O Octave deve ser o unico programa conectado ", ...
              "a esta USB nativa.\n"]);
    safe_command (device, "STATUS");
    change_port = false;
    while (! change_port)
      choice = menu (sprintf ("Gravador ESP32 - %s", port_name), ...
                     "Executar ensaio completo", ...
                     "Receber uma captura FULL", ...
                     "Atualizar status", ...
                     "Limpar o buffer cheio", ...
                     "Armar sem aguardar (avancado)", ...
                     "Informacoes do dispositivo", ...
                     "Testar conexao (PING)", ...
                     "Ajuda do protocolo", ...
                     "Escolher outra porta", ...
                     "Sair");
      switch (choice)
        case 1
          run_complete_experiment (device);
        case 2
          receive_full_capture (device);
        case 3
          safe_command (device, "STATUS");
        case 4
          clear_capture (device);
        case 5
          rate_hz = choose_rate ();
          if (! isempty (rate_hz))
            safe_command (device, sprintf ("ARM %d", rate_hz));
          endif
        case 6
          safe_command (device, "INFO");
        case 7
          safe_command (device, "PING");
        case 8
          safe_command (device, "HELP");
        case 9
          change_port = true;
        otherwise
          exit_console = true;
          change_port = true;
      endswitch
    endwhile
  unwind_protect_cleanup
    clear device;
  end_unwind_protect
endfunction

function port_name = choose_port ()
  port_name = "";
  while (true)
    try
      ports = ts_list_serial_ports ();
    catch err
      fprintf (2, "Nao foi possivel listar as portas seriais: %s\n", ...
               err.message);
      ports = {};
    end_try_catch

    options = cell (size (ports));
    for index = 1:numel (ports)
      options{index} = describe_port (ports{index});
    endfor
    options{end + 1} = "Atualizar lista de portas";
    options{end + 1} = "Digitar uma porta manualmente";
    options{end + 1} = "Sair";
    if (isempty (ports))
      title_text = "Nenhuma porta serial encontrada";
    else
      title_text = "Selecione a porta USB nativa do ESP32-S3";
    endif
    choice = menu (title_text, options{:});

    if (choice >= 1 && choice <= numel (ports))
      port_name = ports{choice};
      return;
    elseif (choice == numel (ports) + 1)
      continue;
    elseif (choice == numel (ports) + 2)
      entered = strtrim (input ("Porta (ex.: /dev/cu.usbmodem1101 ou COM5): ", ...
                                "s"));
      if (! isempty (entered))
        port_name = entered;
        return;
      endif
    else
      return;
    endif
  endwhile
endfunction

function label = describe_port (port_name)
  if (! isempty (strfind (lower (port_name), "usbmodem")))
    label = sprintf ("%s  [provavel USB nativa]", port_name);
  elseif (! isempty (strfind (lower (port_name), "usbserial")))
    label = sprintf ("%s  [provavel USB-UART/gravacao]", port_name);
  else
    label = port_name;
  endif
endfunction

function response = safe_command (device, command)
  response = "";
  fprintf ("\n> %s\n", command);
  try
    response = ts_command (device, command, false);
    fprintf ("%s\n", response);
    if (strcmp (command, "STATUS") && ! valid_status (response))
      fprintf (2, ["Resposta STATUS incompleta. Feche qualquer monitor ", ...
                  "conectado a USB nativa.\n"]);
    endif
  catch err
    fprintf (2, "Erro de comunicacao: %s\n", err.message);
  end_try_catch
endfunction

function run_complete_experiment (device)
  try
    status = ts_command (device, "STATUS", false);
    require_status (status);
    state = response_field (status, "state");

    if (strcmp (state, "FULL"))
      choice = menu ("Ja existe uma captura FULL", ...
                     "Receber a captura existente", ...
                     "Limpar e iniciar um novo ensaio", ...
                     "Cancelar");
      if (choice == 1)
        receive_full_capture (device);
        return;
      elseif (choice == 2)
        response = ts_command (device, "CLEAR", false);
        require_ok (response, "CLEAR");
      else
        return;
      endif
    elseif (strcmp (state, "ARMED") || strcmp (state, "CAPTURING"))
      choice = menu (sprintf ("Ja existe uma captura %s", state), ...
                     "Aguardar e receber esta captura", "Cancelar");
      if (choice == 1 && wait_until_full (device))
        receive_full_capture (device);
      endif
      return;
    elseif (! strcmp (state, "EMPTY"))
      error ("Estado desconhecido: %s", state);
    endif

    rate_hz = choose_rate ();
    if (isempty (rate_hz))
      return;
    endif
    [output_file, clear_after, accepted] = capture_options ();
    if (! accepted)
      return;
    endif

    response = ts_command (device, sprintf ("ARM %d", rate_hz), false);
    require_ok (response, "ARM");
    fprintf ("%s\n", response);
    if (wait_until_full (device))
      download_capture (device, output_file, clear_after);
    endif
  catch err
    fprintf (2, "Falha no ensaio: %s\n", err.message);
  end_try_catch
endfunction

function completed = wait_until_full (device)
  completed = false;
  fprintf ("Aguardando o buffer; pressione Ctrl+C para cancelar.\n");
  while (true)
    response = ts_command (device, "STATUS", false);
    require_status (response);
    state = response_field (response, "state");
    samples = str2double (response_field (response, "samples"));
    capacity = str2double (response_field (response, "capacity"));
    percent = 100 * samples / max (capacity, 1);
    fprintf ("\rEstado: %-10s %6d/%6d amostras (%5.1f%%)", ...
             state, samples, capacity, percent);
    fflush (stdout);
    if (strcmp (state, "FULL"))
      fprintf ("\nCaptura concluida; iniciando o download.\n");
      completed = true;
      return;
    elseif (strcmp (state, "EMPTY"))
      fprintf ("\n");
      error ("O gravador voltou para EMPTY antes de concluir a captura");
    endif
    pause (0.5);
  endwhile
endfunction

function receive_full_capture (device)
  try
    status = ts_command (device, "STATUS", false);
    require_status (status);
    if (! strcmp (response_field (status, "state"), "FULL"))
      fprintf ("O gravador ainda nao esta FULL.\n");
      return;
    endif
    [output_file, clear_after, accepted] = capture_options ();
    if (accepted)
      download_capture (device, output_file, clear_after);
    endif
  catch err
    fprintf (2, "Falha ao receber a captura: %s\n", err.message);
    fprintf ("O buffer do ESP32 nao foi limpo automaticamente.\n");
  end_try_catch
endfunction

function download_capture (device, output_file, clear_after)
  fprintf ("Recebendo a captura binaria...\n");
  capture = ts_capture (device, output_file, clear_after, false);
  ts_plot_capture (capture);
  fprintf ("Foram abertas %d janelas, uma para cada canal.\n", ...
           numel (capture.channels));
endfunction

function [output_file, clear_after, accepted] = capture_options ()
  output_file = "";
  clear_after = false;
  accepted = false;
  default_name = ["capture_", datestr(now (), "yyyymmdd_HHMMSS"), ".mat"];
  entered = strtrim (input (sprintf ("Arquivo MAT de destino [%s]: ", ...
                                     default_name), "s"));
  if (isempty (entered))
    output_file = default_name;
  else
    output_file = entered;
  endif
  choice = menu ("Depois de validar o download e o CRC, limpar o buffer?", ...
                 "Limpar depois do CRC", "Manter o buffer FULL", "Cancelar");
  if (choice == 1)
    clear_after = true;
    accepted = true;
  elseif (choice == 2)
    accepted = true;
  endif
endfunction

function rate_hz = choose_rate ()
  rate_hz = [];
  rates = {"1000 Hz", "500 Hz", "250 Hz", "200 Hz", "100 Hz", ...
           "50 Hz", "20 Hz", "10 Hz", "Digitar outra taxa", "Cancelar"};
  values = [1000, 500, 250, 200, 100, 50, 20, 10];
  choice = menu ("Escolha a taxa de amostragem da captura", rates{:});
  if (choice >= 1 && choice <= numel (values))
    rate_hz = values(choice);
  elseif (choice == numel (values) + 1)
    entered = input ("Taxa em Hz (deve dividir 1000 exatamente): ");
    if (! isscalar (entered) || ! isfinite (entered) || ...
        entered <= 0 || entered != fix (entered) || mod (1000, entered) != 0)
      fprintf (2, "A taxa deve ser um divisor inteiro positivo de 1000.\n");
      return;
    endif
    rate_hz = entered;
  endif
endfunction

function clear_capture (device)
  choice = menu ("Limpar o buffer de captura cheio?", ...
                 "Cancelar", "Enviar CLEAR");
  if (choice == 2)
    safe_command (device, "CLEAR");
  endif
endfunction

function require_status (response)
  if (! valid_status (response))
    error (["Resposta STATUS incompleta: '%s'. Outro programa provavelmente ", ...
            "esta lendo a mesma USB nativa."], response);
  endif
endfunction

function valid = valid_status (response)
  valid = strncmp (response, "OK command=STATUS ", 18) && ...
          ! isempty (response_field (response, "state")) && ...
          ! isempty (response_field (response, "samples")) && ...
          ! isempty (response_field (response, "capacity"));
endfunction

function require_ok (response, command)
  prefix = sprintf ("OK command=%s ", command);
  if (! strncmp (response, prefix, length (prefix)))
    error ("Comando %s recusado ou resposta incompleta: %s", command, response);
  endif
endfunction

function value = response_field (response, key)
  token = regexp (response, ["(?:^| )", key, "=([^ ]+)"], ...
                  "tokens", "once");
  if (isempty (token))
    value = "";
  else
    value = token{1};
  endif
endfunction
