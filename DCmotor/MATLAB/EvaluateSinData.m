%% Read Data
Data = xlsread('..\Data\SinData10hz.xlsx');
Data = xlsread('..\Data\SinData2hz_grd_fast.xlsx');
% Data = xlsread('SinData1hz_grd.xlsx');

freq = 2;
%% Data pushing
Len = length(Data)/4;

time = zeros(Len,1);
Omega = zeros(Len,1);
Pose = zeros(Len,1);
Out = zeros(Len,1);

for i = 1:Len
    time(i)=Data(i*4-3);
    Omega(i)=Data(i*4-2);
    Pose(i)=Data(i*4-1);
    Out(i)=Data(i*4);    
end

%%
figure(11);
plot(time,Omega);
legend(' angular velocity [m/s]')
xlabel('Time[ms]');
ylabel('Speed[m/s]');
grid on;

figure(2);
plot(time,Pose);
xlabel('Time[ms]');
ylabel('position[rad]');
grid on;

figure(3);
Ref = 5*sin(time/1000*5*2*pi);
plot(time,Out/255*5,time,Ref);
xlabel('Time[ms]');
ylabel('Voltage[V]');
legend('Actual Output','Reference Output')
grid on;

%% Data pushing
Len = length(Data)/4;

time = zeros(Len,1);
Omega = zeros(Len,1);
Pose = zeros(Len,1);
Out = zeros(Len,1);

for i = 1:Len
    time(i)=Data(i*4-3);
    Omega(i)=Data(i*4-2);
    Pose(i)=Data(i*4-1);
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
plot(time,Pose);
xlabel('Time[ms]');
ylabel('position[rad]');
grid on;

figure(3);
Ref = 5*sin(time*0.01*2*pi);
plot(time,Out/255*5,time,Ref);
xlabel('Time[ms]');
ylabel('Voltage[V]');
legend('Actual Output','Reference Output')
grid on;

%% Get Rectifi Scaling
% get syms
syms t  y
omega = single(freq*2*pi);
p = sym('p',[1 3]);
func = p(1) * 5 * sin(omega*t + p(2)) + p(3);
R = y - func;
S = R^2;


%% get jacob
ST = Len*2/4;
ED = Len*3/4;

Jaco = jacobian(R,p);
Jacob = subs(Jaco,{t},{time(ST:ED)/1000}); 
SEF = subs(S,{t,y},{time(ST:ED)/1000,Omega(ST:ED)});
r = subs(R,{t,y},{time(ST:ED)/1000,Omega(ST:ED)});
iteration = 20;

%%
lambda = 3;
I = eye(size(Jacob,2));
sinit = (max(Omega(ST:ED))-min(Omega(ST:ED)))/10;
deltainit = -0.1;
binit = 0;

v = [sinit, deltainit,binit].';
for i = 1:iteration
   J = double(subs(Jacob,p,v.'));
   df = double(subs(r,p,v.'));
   delta = - ( J.' * J + lambda * I)\J.' *  df;
   v = v + delta;

   SOE = double(subs(SEF,p,v.'));
   SumOfError(i) = SOE.' * SOE;
end

figure(200);
 plot(SumOfError);
 legend('Squared Error');
 xlabel('iteration');
 grid on;
%%
phat = v.';
if phat(1)<0
    phat(1) = -phat(1)
    phat(2) = phat(2)+pi;
end
%%
%phat(1)= phat(1)*0.95
figure(120)
tt = 1:0.01:10;
estf = subs(func,p,phat);
est = subs(estf,{t},{tt});
plot(time,Omega,'r--',tt*1000,est,'b-');

   SOE = double(subs(SEF,p,phat));
   SumOfError = SOE.' * SOE

   