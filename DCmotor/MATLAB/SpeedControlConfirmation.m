%% Read Data
close all;
clear all;

Data = xlsread('..\Arduino_code\CheckSpeedController\SpeedcontrollerCheck_v5.xlsx');
% Data = xlsread('SinData1hz_grd.xlsx');

%% Data pushing
Len = length(Data)/4;

time = zeros(Len,1);
Omega = zeros(Len,1);
Ref = zeros(Len,1);
Out = zeros(Len,1);

for i = 1:Len
    time(i)=Data(i*4-3);
    Omega(i)=Data(i*4-2);
    Ref(i)=Data(i*4-1);
    Out(i)=Data(i*4);    
end

%%
figure(1);
plot(time,Omega);
legend(' angular velocity [m/s]')
xlabel('Time[ms]');
ylabel('Speed[m/s]');
grid on;

figure(2);
plot(time,Ref);
xlabel('Time[ms]');
ylabel('position[rad]');
grid on;

figure(3);
plot(time,Out);
xlabel('Time[ms]');
ylabel('Voltage[V]');
legend('Actual Output','Reference Output')
grid on;

figure(4);
plot(time,Ref,time,Omega);
grid on;
xlabel('Time[ms]');
ylabel('Speed[m/s]');
legend('Reference Velocity','Actual Velocity')
