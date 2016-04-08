clear
close all
clc

hf = figure(1);
set(hf,'PaperUnits','Points');
set(hf,'PaperPosition',[650,550,350,300]);
set(hf,'Units','Points');
set(hf,'Position',[650,550,350,300]);
clf
hold on

hc = figure(2);
set(hc,'PaperUnits','Points');
set(hc,'PaperPosition',[650,550,350,300]);
set(hc,'Units','Points');
set(hc,'Position',[650,550,350,300]);
clf

hold on

% 100
[time,x,y,~,xEst,yEst,~,xV,yV,xyV] = SimKSLimportfile('Sim_USL100.txt');

rho = sqrt((x-xEst).^2 + (y-yEst).^2);
lambdas = sqrt(getLambdas(xV,yV,xyV));
rho = rho([1,3,4,5,7,8,10,11,13,14]);
lambdas = lambdas([1,3,4,5,7,8,10,11,13,14]);

figure(hf)
plot(rho);
figure(hc)
plot(lambdas)

% 500
[time,x,y,~,xEst,yEst,~,xV,yV,xyV] = SimKSLimportfile('Sim_USL500.txt');

rho = sqrt((x-xEst).^2 + (y-yEst).^2);
lambdas = sqrt(getLambdas(xV,yV,xyV));
rho = rho([1,3,4,5,7,8,10,11,13,14]);
lambdas = lambdas([1,3,4,5,7,8,10,11,13,14]);

figure(hf)
plot(rho);
figure(hc)
plot(lambdas)

% 1000
[time,x,y,t,xEst,yEst,~,xV,yV,xyV] = SimKSLimportfile('Sim_USL1000.txt');

rho = sqrt((x-xEst).^2 + (y-yEst).^2);
lambdas = sqrt(getLambdas(xV,yV,xyV));
rho = rho([1,3,4,5,7,8,10,11,13,14]);
lambdas = lambdas([1,3,4,5,7,8,10,11,13,14]);

figure(hf)
plot(rho);
figure(hc)
plot(lambdas)

% 3000
[time,x,y,~,xEst,yEst,~,xV,yV,xyV] = SimKSLimportfile('Sim_USL3000.txt');

rho = sqrt((x-xEst).^2 + (y-yEst).^2);
lambdas = sqrt(getLambdas(xV,yV,xyV));
rho = rho([1,3,4,5,7,8,10,11,13,14]);
lambdas = lambdas([1,3,4,5,7,8,10,11,13,14]);

figure(hf)
plot(rho);
figure(hc)
plot(lambdas)



figure(hf);
set(gca,'YScale','Log')
xlabel 'Checkpoint Number'
ylabel 'Radial Distance [m]'
grid on
title 'Radial Error of Estimate'
legend 'N=100' 'N=500' 'N=1000' 'N=3000' 'Location' 'Best'

figure(hc)
set(gca,'YScale','Log')
xlabel 'Checkpoint Number'
ylabel 'Uncertainty [m]'
grid on
title 'Uncertainty of Estimate'
legend 'N=100' 'N=500' 'N=1000' 'N=3000' 'Location' 'Best'
