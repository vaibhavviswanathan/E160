clear
clc
close all

load USL_500.mat

%%

% Checkpoints (0,0) (0,-2.74/2) (0,-2.74) (0, -2.74 - 2.31/2) (5.03/2,
% -2.74 - 2.31/2)

% xChecks = [0, 0, 0, 0, 5.03/2];
% yChecks = [0, -2.74/2, -2.74, -2.74 - 2.31/2, -2.74 - 2.31/2];
xChecks = [0,0,0,0,48, 96, 48, 0, 0, 0]'*2.54/100;
yChecks = [0,-55,-103, -151, -151, -151, -151,-151, -103, -55 ]'*2.54/100;

xEsts = x_est(inds);
yEsts = y_est(inds);
xVs = x_var(inds);
yVs = y_var(inds);
xyVs = xv_covar(inds);
xO = x(inds);
yO = y(inds);
xChecks = xChecks(1:length(inds));
yChecks = yChecks(1:length(inds));
rEsts = sqrt( (xEsts - xChecks).^2 + (yEsts - yChecks).^2);
rO = sqrt( (xO - xChecks).^2 + (yO - yChecks).^2);
lambdas = ones(size(xVs));
for i = 1:length(xVs)
    Sigma = [xVs(i), xyVs(i); xyVs(i), yVs(i)];
    [~,D] = eig(Sigma);
    lambdas(i) = max(max(D));
end

hf = figure(1);
clf
hf = plotMap(hf);
set(hf,'PaperUnits','Points');
set(hf,'PaperPosition',[650,550,350,300]);
set(hf,'Units','Points');
set(hf,'Position',[650,550,350,300]);

a1 = plot(x,y);
a2 = plot(x_est,y_est);
a3 = plot(xChecks, yChecks, 'x', 'LineWidth', 3);
plot(xEsts,yEsts, '+', 'LineWidth', 3);
for i = 1:length(xVs)
    plotConfidenceEllipse(xVs(i), yVs(i), xyVs(i), xEsts(i), yEsts(i));
end

legend([a1,a2, a3],{'Odometry', 'Particle Filter', 'Checkpoints'},...
    'Location','Best');

hold off
axis([-5,5,-6,1]);
grid on

title 'Robot Path Localization'
xlabel 'x-Position [m]'
ylabel 'y-Position [m]'

%%
hf = figure(2);
set(hf,'PaperUnits','Points');
set(hf,'PaperPosition',[650,550,350,300]);
set(hf,'Units','Points');
set(hf,'Position',[650,550,350,300]);
% clf

plot(rEsts);
hold on
plot(rO);

grid on
legend 'Particle Filter' 'Odometry' 'Location' 'NorthWest'
title 'Distance of Estimate To Checkpoint'
xlabel 'Checkpoint Number'
ylabel 'Radial Distance [m]'

%%

hf = figure(3);
set(hf,'PaperUnits','Points');
set(hf,'PaperPosition',[650,550,350,300]);
set(hf,'Units','Points');
set(hf,'Position',[650,550,350,300]);
clf

hold on
plot(sqrt(lambdas));

grid on
title 'Convergence of Particle Filter'
xlabel('Checkpoint Number');
ylabel('Uncertainty [m]');
