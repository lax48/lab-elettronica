clear;

function doStuff()
    cleanupObj = onCleanup(@cleanUp);

    mycomm = serialport('/dev/tty.usbmodem1103', 921600);
    write(mycomm, 'c', 'char');
    vref_cal = read(mycomm, 1, "uint16");
    t1 = read(mycomm, 1, "uint16");
    t1_cal = read(mycomm, 1, "uint16");
    t2 = read(mycomm, 1, "uint16");
    t2_cal = read(mycomm, 1, "uint16");
    
    disp("VREF CAL: 3.3 [" + vref_cal + "]");
    disp("T1: " + t1 + " [" + t1_cal + "]");
    disp("T2: " + t2 + " [" + t2_cal + "]");

    figure;
    subplot(3, 1, 1);
    scatterPlot = scatter([], []); % Empty scatter plot
    xlabel('Voltage (V)');
    ylabel('Temperature (C)');
    grid on;

    subplot(3, 1, 2);
    voltagePlot = plot(nan, nan); % Empty scatter plot
    xlabel('Index');
    ylabel('Voltage (V)');
    grid on;

    subplot(3, 1, 3);
    tempPlot = plot(nan, nan); % Empty scatter plot
    xlabel('Index');
    ylabel('Temperature (C)');
    grid on;

    while true
        if mycomm.NumBytesAvailable > 0
            meas_type = read(mycomm, 1, 'char');
            disp(meas_type + " [" + cast(meas_type, "uint8") + "]")
            
            switch meas_type
                case 'v'
                    data = read(mycomm, 1, 'uint16');
                    v = 3.3 * vref_cal / data;
                    disp("VREF = " + v + " V" + "[" + data + "]");
                    scatterPlot.XData(end + 1) = v;
                    a = length(scatterPlot.XData);
                    voltagePlot.XData = 1:a;
                    voltagePlot.YData = scatterPlot.XData;

                case 't'
                    data = read(mycomm, 1, 'uint16');
                    t = (t2 - t1)/(t2_cal - t1_cal)*(data - t1_cal) + t1;
                    disp("T = " + t + " C [" + data + "]");
                    scatterPlot.YData(end + 1) = t;
                    tempPlot.XData = 0:length(tempPlot.YData);
                    tempPlot.YData(end + 1) = t;
                    drawnow;
                
                case '!'
                    scatterPlot.XData = [];
                    scatterPlot.YData = [];
                    voltagePlot.XData = [];
                    voltagePlot.YData = [];
                    tempPlot.XData = [];
                    tempPlot.YData = [];
                    drawnow;
                   
                otherwise
                    disp("Err: " + meas_type);
                    return
            end
        end
    end

    function cleanUp()
        disp("Bye");
        delete(mycomm);
    end
end


doStuff();