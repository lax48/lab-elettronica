clear;
mycomm = serialport('/dev/tty.usbmodem1103', 115200);

answ = "";
while answ ~= "uscire"
    while mycomm.NumBytesAvailable > 0
        disp("M> " + mycomm.readline());
    end

    disp("-------")
    answ = input("Cosa vuoi rispondere? ");
    writeline(mycomm, answ)
    disp("=======")
    pause(.1) % wait for micro to receive commands
end
mycomm = [];