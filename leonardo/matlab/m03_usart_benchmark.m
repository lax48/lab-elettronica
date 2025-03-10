clear;

function state = xorshift64(state)
    state = cast(state, "uint64");
    state = bitxor(state, bitshift(state, 13, "uint64"), "uint64");
    state = bitxor(state, bitshift(state, -7, "uint64"), "uint64");
    state = bitxor(state, bitshift(state, 17, "uint64"), "uint64");
end

function doStuff()
    count = 2^16;
    oneSet = 2^10;
    mySeed = cast(0xfac83126, "uint64");
    myEnd = cast(0xABCDEF, "uint64");

    mycomm = serialport('/dev/tty.usbmodem1103', 921600, Timeout=10);

    cleanupObj = onCleanup(@() cleanUp(mycomm));

    % Burst write (MCU VP)
    disp("Configure write burst test")
    write(mycomm, 'r', 'char');
    write(mycomm, 'w', 'char');
    write(mycomm, count, "uint64");
    write(mycomm, mySeed, "uint64");

    ack = read(mycomm, 1, "char");
    if ack == '!'
        disp("Ready")
    else
        disp("error(" + cast(ack, "uint8") + ")");
        pause(1)
        disp(read(mycomm, mycomm.NumBytesAvailable, "char"))
        error('Test not started: ')
    end

    fails = 0;

    myState = mySeed;
    readBytes = 0;
    for i = 1:(count / oneSet)
        vals = read(mycomm, oneSet, "uint64");
        readBytes = readBytes + length(vals)*8;
        
        for j = 1:length(vals)
            myState = xorshift64(myState);

            if cast(myState, "single") ~= cast(vals(j), "single")
                fails = fails + 1;
                disp("Mismatch #" + ((j-1) + (i-1) * oneSet + 1) + ...
                    ": exp(" + myState + ") != val(" + vals(j) + ")");
            end
        end
    end

    disp("Read " + readBytes + " bytes")

    if cast(mycomm.read(1, 'uint64'), 'single') == cast(myEnd, 'single')
    else
        disp('Some loss occured')
    end

    if mycomm.NumBytesAvailable > 0
        disp("Extra bytes sent: " + mycomm.NumBytesAvailable);
    end

    write(mycomm, 'v', 'char');

    clock = read(mycomm, 1, 'uint32');
    psc = read(mycomm, 1, 'uint16');
    cnt = read(mycomm, 1, 'uint16');
    read(mycomm, 1, 'uint64'); % fails

    time = cnt * (psc+1) / clock;
    disp("Time: " + time + " s")

    disp("Test 1 end: v=" + (readBytes/1024) / time + ...
        " kB/s (fails = " + fails + ")");

    function cleanUp(mycomm)
        disp("Bye");
        delete(mycomm);
    end
end

doStuff();