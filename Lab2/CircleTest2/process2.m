
clear
clearvars -global
close all
load rotationtestdata2
clc

%%
global timestamp wheelDispL wheelDispR xm ym trot;

trot = trot - trot(1);
trot = -trot;

hf = figure(2);
set(hf,'Units','Points');
set(hf,'Position',[650,550,350,300]);

h1 = plot(xm,ym,'*', 'Color', [     0    0.4470    0.7410]);
hold on
% quiver(xm, ym, 0.5*cos(Angledegrees/180*pi), -0.5*sin(Angledegrees/180*pi));
a = quiver(xm,ym,cos(trot),sin(trot), 0.2);
set(a,'Color', h1.Color);

% inds = [1, 5, 19, 31, 44, 55, 69, 80, 92, 106, 115, 127, 139, 151, 163, 175, 187, 200]/0.053034300000000;
% inds = round(inds);
inds = [20, 140, 400, 600, 800, 1000, ...
    1250, 1450, 1650, 1900, 2100, 2300, ...
    2550, 2750, 2950, 3200, 3400, 3700];
inds = inds(1:14);

xj = xJag(inds);
yj = yJag(inds);
tj = tJag(inds);
h2 = plot(xj,yj,'*', 'Color', [0.8500    0.3250    0.0980]);
a = quiver(xj,yj,cos(tj),sin(tj), 0.2);
set(a,'Color', h2.Color);
a = plot(xJag,yJag);
set(a,'Color', h2.Color);

legend([h1,h2],{'True State', 'Estimated State'}, 'Location', 'West')

xlabel 'x [m]'
ylabel 'y [m]'

grid on

legend([h1,h2],{'True State', 'Estimated State'}, 'Location', 'West')
title 'Half Circle Rotation Test'


%%
R0 = 0.089;
b0 = 0.242;
mmse = objfun([b0,R0,R0]);

argsmin = Inf;
minmmse = Inf;

x0 = [b0,R0,R0];
lb = 0.8*[b0,R0,R0];
ub = 1.2*[b0,R0,R0];

[args_x0,minmmse_x0] = fmincon(@objfun, x0,[],[],[],[],lb,ub);
%%

argsmin = args_x0;

[xop,yop,top,exop,eyop,etop] = integrate_wheeldisps(timestamp,wheelDispL,wheelDispR,...
    argsmin(1), argsmin(2), argsmin(3));

xo = xop(inds);
exo = exop(inds);
yo = yop(inds);
eyo = eyop(inds);
to = top(inds);
eto = etop(inds);

%%

hf = figure(3);
set(hf,'Units','Points');
set(hf,'Position',[650,550,350,300]);


h1 = plot(xm,ym,'*', 'Color', [     0    0.4470    0.7410]);
hold on
% quiver(xm, ym, 0.5*cos(Angledegrees/180*pi), -0.5*sin(Angledegrees/180*pi));
a = quiver(xm,ym,cos(trot),sin(trot), 0.2);
set(a,'Color', h1.Color);


h2 = plot(xo,yo,'*', 'Color', [0.9290    0.6940    0.1250]);
a = quiver(xo,yo,cos(to),sin(to), 0.2);
set(a,'Color', h2.Color);
% a = quiver(xo,yo,cos(to-eto),sin(to-eto), 0.1);
% set(a,'Color', h2.Color);
% a = quiver(xo,yo,cos(to+eto),sin(to+eto), 0.1);
% set(a,'Color', h2.Color);
a = plot(xop,yop);
set(a,'Color', h2.Color);

for i = 1:length(exo)
    a = exo(i); % horz radius
    b = eyo(i); % vert radius
    x0 = xo(i); % horz center
    y0 = yo(i); % vert center
    t=-pi:0.01:pi;
    x=x0+a*cos(t);
    y=y0+b*sin(t);
    plot(x,y, '--', 'Color', h2.Color)
end

xj = xJag(inds);
yj = yJag(inds);
tj = tJag(inds);
h3 = plot(xj,yj,'*', 'Color', [0.8500    0.3250    0.0980]);
a = quiver(xj,yj,cos(tj),sin(tj), 0.2);
set(a,'Color', h3.Color);
a = plot(xJag,yJag);
set(a,'Color', h3.Color);

xlabel 'x [m]'
ylabel 'y [m]'

grid on

legend([h1,h2,h3],{'True State', 'Optimized State', 'Estimated State'}, 'Location', 'West')

title 'Half Circle Rotation Test'


% print -dpng -r500 RotTest2.png
