% Author: Abdullah Altawaitan 
% Description: MATLAB script for sending float values over serial
% communication to (Arduino or Teensy 3.2) at a specific frequency. 

if ~isempty(instrfind)
    fclose(instrfind);
    delete(instrfind);
end
% Creating serial port object 
s = serial('/dev/tty.usbserial-AL03QWTA','BaudRate',230400);
fopen(s);
% Random float values to test
parameters = [5.0 2.1 3.7];
% Execute loop at fixed frequency (50 Hz in our case)
r = robotics.Rate(50);
% Reset the object prior to the loop execution to reset timer
reset(r)

while 1
    parameters(1) = parameters(1) + 0.1;
    parameters(2) = parameters(2) - 0.1;
    parameters(3) = parameters(3) + 0.2;
    fprintf(s, '<%f, %f, %f>', [parameters(1), parameters(2), parameters(3)]);
    waitfor(r);
end