clear
close all
clc

load 1000_KSL2.mat

%%
close all

% Checkpoints (0,0) (0,-2.74/2) (0,-2.74) (0, -2.74 - 2.31/2) (5.03/2,
% -2.74 - 2.31/2)

% xChecks = [0, 0, 0, 0, 5.03/2];
% yChecks = [0, -2.74/2, -2.74, -2.74 - 2.31/2, -2.74 - 2.31/2];
xChecks = [0,0,0,0,55]*2.54/100;
yChecks = [0,-62,-117, -172, -172]*2.54/100;



hf = plotMap();

set(hf,'PaperUnits','Points');
set(hf,'PaperPosition',[650,550,350,300]);

a1 = plot(X,Y);
a2 = plot(Xest,Yest);
a3 = plot(xChecks, yChecks, 'x', 'LineWidth', 3);
legend([a1,a2, a3],{'Odometry', 'Particle Filter', 'Checkpoints'},...
    'Location','Best');