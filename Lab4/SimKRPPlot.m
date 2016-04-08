clear
close all
clc

% 1000
[time,x,y,t,xEst,yEst,~,xV,yV,xyV] = SimKSLimportfile('Sim_KRP1000.txt');

rho = sqrt((x-xEst).^2 + (y-yEst).^2);
lambdas = sqrt(getLambdas(xV,yV,xyV));

hf = figure(2);
set(hf,'PaperUnits','Points');
set(hf,'PaperPosition',[650,550,350,300]);
set(hf,'Units','Points');
set(hf,'Position',[650,550,350,300]);

clf

subplot(2,1,1);
plot(time,rho);
grid on
ylabel 'Radial Distance [m]'
title 'Radial Error of Estimate'
subplot(2,1,2);
plot(time,lambdas);
title 'Uncertainty of Estimate'
xlabel 'Time [s]'
ylabel 'Uncertainty [m]'
grid on