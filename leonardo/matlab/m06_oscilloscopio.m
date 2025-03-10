function m06_oscilloscopio()
    % Clean up resources on exit
    mycomm = serialport('/dev/tty.usbmodem103', 921600);
    cleanupObj = onCleanup(@() cleanUp(mycomm));

    % Get initial configuration
    [arr, npoints, trig_off, cnt, src, th1, th2] = getConfiguration(mycomm);
    freq = 1;
    
    % Constants and GUI Configuration
    HEIGHT = 400;
    WIDTH = 800;
    CONTROL_HEIGHT = 20;
    NUM_CONTROLS = 3;
    CONTROLS_SPACE = NUM_CONTROLS * CONTROL_HEIGHT;
    CONTROLS_SPACE_PERC = CONTROLS_SPACE / HEIGHT;
    
    % Frequency array values (Hz)
    arr_values = [ ...
        240000000, 120000000, 48000000, ...
        24000000, 12000000, 4800000, ...
        2400000, 1200000, 480000, ...
        240000, 120000, 48000, ...
        24000, 12000, 4800, ...
        2400, 1200, 480, ...
        240, 120, 64];

    % Create figure and axes for plotting the waveform
    fig = figure('Name', 'MATLAB Oscilloscope', 'NumberTitle', 'off', ...
                 'Position', [100, 100, WIDTH, HEIGHT]);
    ax = axes('Parent', fig, 'OuterPosition', [0, CONTROLS_SPACE_PERC, 1, 1 - CONTROLS_SPACE_PERC]);
    grid on;
    xlabel(ax, 'Time (ms)');
    ylabel(ax, 'Amplitude (V)');
    
    % Initialize controls
    [freq_input, freq_text, trig_off_input, trig_off_text, ...
        trig_src_input, trig_th1_txt, trig_th1_input, trig_th2_txt, trig_th2_input, ...
        trig_manual_btn, continuous_btn] = initControls();
    freq_callback(0, 0);
    trig_off_callback(0, 0);
    reset_graph(0, 0);
    trig_src_callback(0, 0);

    while isvalid(fig)
        if mycomm.NumBytesAvailable > 0
            updatePlot(0, 0)
            drawnow
        else
            pause(.001)
        end
    end
    
    % Cleanup resources upon exit
    function cleanUp(mycomm)
        % Release all resources
        try
            delete(mycomm);
            disp("Resources cleaned up.");
        catch
            disp("An error occurred during cleanup.");
        end
    end

    % Get initial configuration
    function [arr, npoints, trig_off, cnt, src, th1, th2] = getConfiguration(mycomm)
        mycomm.write('s', 'char');
        arr = mycomm.read(1, 'uint32');
        npoints = mycomm.read(1, 'int16');
        trig_off = mycomm.read(1, 'int16');
        cnt = mycomm.read(1, 'uint8');
        src = mycomm.read(1, 'uint8');
        th1 = mycomm.read(1, 'uint16');
        th2 = mycomm.read(1, 'uint16');
    end

    % Initialize GUI Controls
    function [freq_input, freq_text, trig_off_input, trig_off_text, ...
              trig_src_input, trig_th1_txt, trig_th1_input, trig_th2_txt, ...
              trig_th2_input, trig_manual_btn, continuous_btn] = initControls()

        % Frequency controls
        uicontrol('Style', 'text', 'String', 'Frequency: ', ...
                  'Position', [0, CONTROL_HEIGHT*2, 100, CONTROL_HEIGHT], 'HorizontalAlignment', 'right');
        freq_idx = interp1(arr_values, 1:numel(arr_values), arr);
        freq_input = uicontrol('Style', 'slider', 'Value', (freq_idx - 1)/20, ...
                               'Position', [100, CONTROL_HEIGHT*2, 200, CONTROL_HEIGHT], ...
                               'SliderStep', [1/20, 1/20], 'Callback', @freq_callback);
        freq_text = uicontrol('Style', 'text', 'String', '1000 kHz', ...
                              'Position', [300, CONTROL_HEIGHT*2, 100, CONTROL_HEIGHT], 'HorizontalAlignment', 'left');
        
        % Trigger time controls
        uicontrol('Style', 'text', 'String', 'Trigger time: ', ...
                  'Position', [0, CONTROL_HEIGHT, 100, CONTROL_HEIGHT], 'HorizontalAlignment', 'right');
        trig_off_input = uicontrol('Style', 'slider', 'Value', trig_off, 'Position', [100, CONTROL_HEIGHT, 200, CONTROL_HEIGHT], ...
                                   'SliderStep', [1/256, 1/256], 'Min', -127, 'Max', 128, 'Callback', @trig_off_callback);
        trig_off_text = uicontrol('Style', 'text', 'String', '1000 us', ...
                                  'Position', [300, CONTROL_HEIGHT, 100, CONTROL_HEIGHT], 'HorizontalAlignment', 'left');
        
        % Trigger source selection
        uicontrol('Style', 'text', 'String', 'Trigger Source: ', ...
                  'Position', [400, CONTROL_HEIGHT*2, 100, CONTROL_HEIGHT], 'HorizontalAlignment', 'right');
        trig_src_input = uicontrol('Style', 'popupmenu', 'Value', src, 'String', {'None', 'Threshold', 'External'}, ...
                                   'Position', [500, CONTROL_HEIGHT*2, 200, CONTROL_HEIGHT], 'Callback', @trig_src_callback);
        
        % Threshold controls (initially hidden)
        trig_th1_txt = uicontrol('Style', 'text', 'String', 'Trigger TH1: ', ...
                                 'Position', [400, CONTROL_HEIGHT, 100, CONTROL_HEIGHT], 'Visible', 'off', ...
                                 'HorizontalAlignment', 'right');
        trig_th1_input = uicontrol('Style', 'slider', 'Value', th1/(2^16 - 1), 'Position', [500, CONTROL_HEIGHT, 200, CONTROL_HEIGHT], ...
                                   'Visible', 'off', 'Callback', @reset_graph);
        
        trig_th2_txt = uicontrol('Style', 'text', 'String', 'Trigger TH2: ', ...
                                 'Position', [400, 0, 100, CONTROL_HEIGHT], 'Visible', 'off', 'HorizontalAlignment', 'right');
        trig_th2_input = uicontrol('Style', 'slider', 'Value', th2/(2^16 - 1), 'Position', [500, 0, 200, CONTROL_HEIGHT], ...
                                   'Visible', 'off', 'Callback', @reset_graph);
        
        % Additional buttons
        trig_manual_btn = uicontrol('Style', 'pushbutton', 'String', 'Manual trigger', ...
                                    'Position', [400, CONTROL_HEIGHT, 100, CONTROL_HEIGHT], 'Visible', 'off', ...
                                    'Callback', @trig_manual_callback);
        continuous_btn = uicontrol('Style', 'togglebutton', 'String', 'Continuous Mode', ...
                                   'Value', cnt, ...
                                   'Position', [700, CONTROL_HEIGHT*2, 100, CONTROL_HEIGHT], 'Callback', @update_callback);
        uicontrol('Style', 'pushbutton', 'String', 'Update', 'Position', [700, CONTROL_HEIGHT, 100, CONTROL_HEIGHT], ...
                  'Callback', @update_callback);
        uicontrol('Style', 'pushbutton', 'String', 'Manual', 'Position', [700, 0, 100, CONTROL_HEIGHT], ...
                  'Callback', @trig_manual_callback);
    end

    % Helper to conditionally choose a value (like a ternary operator)
    function result = ternary(condition, trueVal, falseVal)
        if condition
            result = trueVal;
        else
            result = falseVal;
        end
    end

    % Update frequency callback
    function freq_callback(~, ~)
        index = round(freq_input.Value * 20) + 1;
        arr = arr_values(index);
        freq = 240 * 1e6 / arr;
        updateFrequencyText();
        trig_off_callback();
    end

    % Update frequency text display
    function updateFrequencyText()
        if freq < 1000
            freq_text.String = sprintf('%.0f Hz', freq);
        elseif freq < 1e6
            freq_text.String = sprintf('%.1f kHz', freq / 1e3);
        else
            freq_text.String = sprintf('%.2f MHz', freq / 1e6);
        end
    end

    % Trigger offset callback
    function trig_off_callback(~, ~)
        time_off = trig_off_input.Value / freq;
        if freq < 100
            trig_off_text.String = sprintf('%.3f s', time_off);
        elseif freq < 100000
            trig_off_text.String = sprintf('%.3f ms', time_off * 1000);
        else
            trig_off_text.String = sprintf('%.3f us', time_off * 1e6);
        end
    end

    % Trigger source callback to show/hide controls
    function trig_src_callback(~, ~)
        isThreshold = (trig_src_input.Value == 2);
        set([trig_th1_txt, trig_th1_input, trig_th2_txt, trig_th2_input], 'Visible', ternary(isThreshold, 'on', 'off'));
        trig_manual_btn.Visible = ternary(trig_src_input.Value == 1, 'on', 'off');
        reset_graph();
    end

    % Reset graph with thresholds
    function reset_graph(~, ~)
        cla;
        if trig_src_input.Value == 2
            hold on;
            addThresholdLines();
            hold off;
        end
    end

    % Add threshold lines on the plot
    function addThresholdLines()
        yline(3.3 * trig_th1_input.Value, 'r', 'VTH1', 'LineWidth', 1);
        yline(3.3 * trig_th2_input.Value, 'g', 'VTH2', 'LineWidth', 1);
        xline(0, 'y', 'T0', 'LineWidth', 1);
        ylim([0, 1])
    end

    % Manual trigger callback
    function trig_manual_callback(~, ~)
        reset_graph();
        write(mycomm, 't', 'char');
    end

    % Update callback
    function update_callback(~, ~)
        reset_graph();

        if mycomm.NumBytesAvailable > 0
            mycomm.read(mycomm.NumBytesAvailable, "char");
        end

        write(mycomm, 'u', 'char');
        write(mycomm, arr, 'uint32');
        write(mycomm, npoints, 'int16');
        write(mycomm, trig_off_input.Value, 'int16');
        write(mycomm, continuous_btn.Value, 'uint8');
        write(mycomm, trig_src_input.Value, 'uint8');
        write(mycomm, cast(trig_th1_input.Value * (2^16 - 1), 'uint16'), 'uint16');
        write(mycomm, cast(trig_th2_input.Value * (2^16 - 1), 'uint16'), 'uint16');
    end

    % Plot update function (called periodically by the timer)
    function updatePlot(~, ~)
        data = mycomm.read(npoints, 'uint16');
        ampls = data / (2^16 - 1) * 3.3;
        times = ((0:numel(ampls) - 1) + trig_off_input.Value) / freq * 1000;
        reset_graph();
        hold on;
        plot(times, ampls, 'b', 'DisplayName', 'Signal', 'LineWidth', 1);
        if trig_src_input.Value == 2
            addThresholdLines();
        end
        hold off;
    end
end
