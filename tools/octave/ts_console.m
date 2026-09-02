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

    fprintf ("\nPorta selecionada: %s\n", port_name);
    fprintf ("Consultando o estado do gravador...\n");
    safe_command (port_name, "STATUS");

    change_port = false;
    while (! change_port)
      choice = menu (sprintf ("Gravador ESP32 - %s", port_name), ...
                     "Atualizar status", ...
                     "Receber dados gravados (DUMP)", ...
                     "Armar uma nova captura", ...
                     "Limpar o buffer cheio", ...
                     "Informacoes do dispositivo", ...
                     "Testar conexao (PING)", ...
                     "Ajuda do protocolo", ...
                     "Escolher outra porta", ...
                     "Sair");
      switch (choice)
        case 1
          safe_command (port_name, "STATUS");
        case 2
          receive_capture (port_name);
        case 3
          arm_capture (port_name);
        case 4
          clear_capture (port_name);
        case 5
          safe_command (port_name, "INFO");
        case 6
          safe_command (port_name, "PING");
        case 7
          safe_command (port_name, "HELP");
        case 8
          change_port = true;
        otherwise
          fprintf ("Console encerrado.\n");
          return;
      endswitch
    endwhile
  endwhile
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

function response = safe_command (port_name, command)
  response = "";
  fprintf ("\n> %s\n", command);
  try
    response = ts_command (port_name, command);
  catch err
    fprintf (2, "Erro de comunicacao: %s\n", err.message);
  end_try_catch
endfunction

function arm_capture (port_name)
  rates = {"1000 Hz", "500 Hz", "250 Hz", "200 Hz", "100 Hz", ...
           "50 Hz", "20 Hz", "10 Hz", "Digitar outra taxa", "Cancelar"};
  values = [1000, 500, 250, 200, 100, 50, 20, 10];
  choice = menu ("Escolha a taxa de amostragem da captura", rates{:});
  if (choice >= 1 && choice <= numel (values))
    rate_hz = values(choice);
  elseif (choice == numel (values) + 1)
    rate_hz = input ("Taxa em Hz (deve dividir 1000 exatamente): ");
    if (! isscalar (rate_hz) || ! isfinite (rate_hz) || ...
        rate_hz <= 0 || rate_hz != fix (rate_hz))
      fprintf (2, "A taxa deve ser um numero inteiro positivo.\n");
      return;
    endif
  else
    return;
  endif
  safe_command (port_name, sprintf ("ARM %d", rate_hz));
endfunction

function receive_capture (port_name)
  status = safe_command (port_name, "STATUS");
  if (isempty (status) || isempty (strfind (status, "state=FULL")))
    fprintf ("O gravador nao esta FULL; o DUMP nao foi solicitado.\n");
    return;
  endif

  default_name = ["capture_", datestr(now (), "yyyymmdd_HHMMSS"), ".mat"];
  entered = strtrim (input (sprintf ("Arquivo MAT de destino [%s]: ", ...
                                     default_name), ...
                            "s"));
  if (isempty (entered))
    output_file = default_name;
  else
    output_file = entered;
  endif
  clear_choice = menu ("Depois de validar o download e o CRC, limpar o buffer?", ...
                       "Manter o buffer FULL", ...
                       "Limpar depois do CRC", ...
                       "Cancelar download");
  if (clear_choice == 0 || clear_choice == 3)
    return;
  endif

  fprintf ("\nRecebendo a captura binaria...\n");
  try
    capture = ts_capture (port_name, output_file, clear_choice == 2, false);
    ts_plot_capture (capture);
    fprintf ("Foram abertas %d janelas, uma para cada canal.\n", ...
             numel (capture.channels));
  catch err
    fprintf (2, "Falha ao receber a captura: %s\n", err.message);
    fprintf ("O buffer do ESP32 nao foi limpo automaticamente.\n");
  end_try_catch
endfunction

function clear_capture (port_name)
  choice = menu ("Limpar o buffer de captura cheio?", ...
                 "Cancelar", "Enviar CLEAR");
  if (choice == 2)
    safe_command (port_name, "CLEAR");
  endif
endfunction
