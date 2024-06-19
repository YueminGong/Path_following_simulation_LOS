clc;
clear all;
data = xlsread('D:\UST ip\shipsim01.xlsx');
figure(1)
plot(data(2:150,2),data(2:150,3),'r');

