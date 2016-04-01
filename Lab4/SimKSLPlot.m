clear
close all
clc

hf = figure(2);
set(hf,'PaperUnits','Points');
set(hf,'PaperPosition',[650,550,350,300]);
set(hf,'Units','Points');
set(hf,'Position',[650,550,350,300]);
clf

hold on

[time,x,y,t,xEst,yEst] = SimKSLimportfile('Sim_KSL10.txt');
rho = sqrt((x-xEst).^2 + (y-yEst).^2);
plot(time,rho);
display(rho(end))

tMax = max(time);

[time,x,y,t,xEst,yEst] = SimKSLimportfile('Sim_KSL100.txt');
rho = sqrt((x-xEst).^2 + (y-yEst).^2);
time = tMax * time / max(time);
plot(time,rho);

display(rho(end))

[time,x,y,t,xEst,yEst] = SimKSLimportfile('Sim_KSL1000.txt');
rho = sqrt((x-xEst).^2 + (y-yEst).^2);
time = tMax * time / max(time);
plot(time,rho);

display(rho(end))

[time,x,y,t,xEst,yEst] = SimKSLimportfile('Sim_KSL10000.txt');
rho = sqrt((x-xEst).^2 + (y-yEst).^2);
time = tMax * time / max(time);
plot(time,rho);

display(rho(end))

xlabel 'Time'
ylabel 'Radial Error [m]'
title 'Radial Error Over Time'
set(gca,'YScale','Log');
legend 'N=10' 'N=100' 'N=1000' 'N=10000' 'Location' 'Best'
grid on

