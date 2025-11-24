%% ===========================
%  Setup CAN Channel
% ===========================

clear; clc;

vectorDeviceName = "CANcaseXL 1";
channelNumber = 2;

% Create CAN channel object
canCh = canChannel("Vector", vectorDeviceName, channelNumber);

% Configure bus speed (adjust if needed)
configBusSpeed(canCh, 500000);

% Start channel
start(canCh);

disp("📡 Polling barrier state every 50 ms. Press CTRL+C to stop.");

%% ===========================
%  Signal Tracking
% ===========================

prevValue = -1;   % To track changes

%% ===========================
%  Main Polling Loop
% ===========================

while true
    
    % Read all available messages
    while canCh.MessagesAvailable > 0
        msg = receive(canCh, 1, "OutputFormat", "timetable");

        % Skip if message has no data
        if isempty(msg.Data)
            continue;
        end

        % Extract first byte (barrier state)
        newValue = msg.Data{1}(1);

        % Display only if changed
        if newValue ~= prevValue
            fprintf("🔄 Barrier value changed: %d\n", newValue);
            prevValue = newValue;
        end
    end
    
    % Wait 50 ms before next poll
    pause(0.05);
    
    % Optionally: If your hardware supports remote frame requests,
    % you can send a request here to force it to respond with current state.
    % For example:
    % sendRemoteFrame(canCh, 123, "DataLength", 1);
end

% stop(canCh); % not reached unless loop is interrupted
