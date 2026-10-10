function connection = connectEMGChannels(cfg)
% CONNECTEMGCHANNELS Connect and configure the Mega EMG analog inputs.

    arguments
        cfg (1,1) struct
    end

    port = string(cfg.Hardware.COMPort);
    availablePorts = string(serialportlist("available"));
    if ~any(strcmpi(port, availablePorts))
        if isempty(availablePorts)
            portsText = "none";
        else
            portsText = strjoin(availablePorts, ", ");
        end
        error('MyoDrum:EMGPortUnavailable', ...
            ['Port %s is unavailable. Available ports: %s. Another MATLAB ' ...
             'arduino object or IMU session may hold the port; run clear all ' ...
             'after closing that session.'], port, portsText);
    end

    fprintf('Connecting EMG channels to %s on %s...\n', ...
        cfg.Hardware.Board, port);
    board = arduino(port, cfg.Hardware.Board);
    pins = string(cfg.EMG.AnalogPins(:));
    for pinIndex = 1:numel(pins)
        configurePin(board, pins(pinIndex), "AnalogInput");
    end

    connection.board = board;
    connection.readVoltage = @(pin)readVoltage(board, pin);
    connection.analogPins = pins;
end
