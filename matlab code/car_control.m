clc;
clear;
close all;
%%
%Communication with an autonomous vehicle
%shlomo odem
%8/2021
%%
clear;
ip_address = "192.168.43.217";
port = 23;
client = tcpclient(ip_address,port);
%configureTerminator(client,"CR/LF")

 
 % data = uint8([100 1 0 1])
 %write(client,data);
 
%%
%client.NumBytesAvailable
data = step_velocity(client,1,[0.5 0.6 0.7],0.5);

%%
order = "drive_motor";
data = [1 255 1];
data = communication(client,order, data)