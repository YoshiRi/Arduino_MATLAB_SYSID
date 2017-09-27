%% solve equatoin
% clear all
% 
% syms tau K real;
% 
% eq1 = K/(1+tau*2*pi*j) == 0.1260*cos(-0.8261)+0.1260*sin(-0.8261)*j
% eq2 = K/(1+tau*10*pi*j) == 0.0677*cos(-1.4822)+0.0677*sin(-1.4822)*j
% 
% %%
% ean = (1+tau*2*pi*j)*(0.1260*cos(-0.8261)+0.1260*sin(-0.8261)*j) == (1+tau*10*pi*j)*(0.0677*cos(-1.4822)+0.0677*sin(-1.4822)*j)


%%
close all;
clear all;
%% Experimantal data

freq_tf = [1,2,5];
Mag_Exp = 20*log([0.1260,0.1062,0.0677]);
Phase_Exp = [-0.8261, -0.9111, -1.4814 ]*180/pi;

figure(1);
subplot(2,1,1),h=semilogx(freq_tf,Mag_Exp,'-');
grid on;
%axis([freq_min freq_max Mag_min Mag_max]);
xlabel('Frequency[Hz]');
ylabel('Magnitude [dB]');
% title('Frequency response of plant','FontSize',fonts);
hold on

figure(1);
subplot(2,1,2),h=semilogx(freq_tf,Phase_Exp,'-');
grid on;
%axis([freq_min freq_max Phase_min Phase_max]);
xlabel('Frequency [Hz]');
ylabel('Phase [deg]');
hold on


%% Model
a = 15;
b = 100;
K = 1/b
tau = a/b
sys_model = tf([1],[a, b]);
[Mag_Model,Phase_Model] = bode(sys_model,freq_tf*2*pi);
Mag_Model = 20*log10(Mag_Model(1,:)');
Phase_Model = Phase_Model(1,:)';


figure(1);
subplot(2,1,1),h=semilogx(freq_tf,Mag_Model,'r-.');
legend('Measurement','Model')
% title('Frequency response of plant','FontSize',fonts);
grid on;
hold on

figure(1);
subplot(2,1,2),h=semilogx(freq_tf,Phase_Model,'r-.');
xlabel('Frequency [Hz]');
ylabel('Phase [deg]');
grid on;
hold on

% name_bode = 'fig_bode_';
% print('-depsc',[name_bode name_axis]);
% saveas(h,[name_bode name_axis],'fig');
