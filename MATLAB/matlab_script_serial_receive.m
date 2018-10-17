% Author: Abdullah Altawaitan
% Description: MATLAB script for receiving float values over serial
% communication to (Arduino or Teensy 3.2) at a specific frequency.

if ~isempty(instrfind)
    fclose(instrfind);
    delete(instrfind);
end
% Creating serial port object
s = serial('/dev/tty.usbserial-AL03QWT5','BaudRate',230400);
fopen(s);
% Execute loop at fixed frequency (20 Hz in our case)
r = robotics.Rate(20);
% Reset the object prior to the loop execution to reset timer
reset(r)
char = strings(10000,16);
timer = zeros(10000,1);
i = 1;
commandChar = '.';
while 1
    if (i >= 9998)
        break
    end
    time = r.TotalElapsedTime;
    timer(i) = time;
    fprintf(s, '%s', commandChar);
    line = fscanf(s, '%s');
    char(i) = line;
%     value(i) = str2num(newline(1));
    i = i + 1;
    waitfor(r);
end
