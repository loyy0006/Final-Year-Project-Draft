clc;clear; close all;
tic;
while true
time_now = toc
judp('SEND',36500,'192.168.0.118',typecast(time_now,'int8'))
judp('SEND',36500,'localhost',typecast(time_now,'int8'))
end