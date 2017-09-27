%% Read Data
Data = xlsread('..\Data\StepData1sec.xlsx');
Data = xlsread('..\Data\StepData2.xlsx');

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
legend(' angular velocity [rad/s]')
xlabel('Time[ms]');
ylabel('Speed[rad/s]');
grid on;

figure(2);
plot(time,Pose);
xlabel('Time[ms]');
ylabel('position[rad]');
grid on;

figure(3);
plot(time,Out/255*5);
xlabel('Time[ms]');
ylabel('Voltage[V]');
grid on;

%% Get Rectifi Scaling
figure(4)
% Tuning Parameter1
y = histogram(Omega,200);
xlabel('value');
ylabel('frequency');
grid on;

% Tuning Parameter2
LeastBin = 10;
Freq = y.Values(y.Values>LeastBin);
FreqBin = y.BinEdges(y.Values>LeastBin);
FreqSum = FreqBin .* Freq;

plus = sum(FreqSum(FreqSum>=0));
minus = sum(FreqSum(FreqSum<0));
psum = sum(Freq(FreqSum>0));
msum = sum(Freq(FreqSum<0));

Upper = sum(plus)/psum;
Lower = sum(minus)/msum;

figure(5);
plot(time,Omega,'k',time,ones(Len,1)*Upper,'r--',time,ones(Len,1)*Lower,'b--');
xlabel('Time [ms]');
ylabel('output');
legend('output','Upper','Lower','Location','Best')
grid on;

if isnan(Lower)
    Lower = 0;
end

figure(6)
plot(time,(Out-min(Out))/(max(Out)-min(Out)),'r--',time,(Omega-Lower)/(Upper-Lower),'b-');
xlabel('Time [ms]');
ylabel('normalized In/Out');
legend('Input','Output','Location','Best')
grid on;

scale = (Upper -Lower)/(max(Out)-min(Out));

%% estimate tau

tau = 2.5;
sys = tf([1],[tau 1]);
sysd = c2d(sys,time(2)-time(1),'tustin');
filterd = filter(cell2mat(sysd.num),cell2mat(sysd.den),(Out-min(Out))/(max(Out)-min(Out)));

figure(7)
plot(time,(Out-min(Out))/(max(Out)-min(Out)),'y--',time,filterd,'r--',time,(Omega-Lower)/(Upper-Lower),'b-');
xlabel('Time [ms]');
ylabel('normalized In/Out');
legend('Input','Estimated','Output','Location','Best')
grid on;
