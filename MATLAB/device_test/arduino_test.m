clear
clc

arduino = serialport("/dev/ttyACM0",9600);
configureTerminator(arduino,"LF");

com_st = 600;
com_ac = 300;
com_str = sprintf("%d,%d",com_st,com_ac);%発進措置
writeline(arduino,com_str);
