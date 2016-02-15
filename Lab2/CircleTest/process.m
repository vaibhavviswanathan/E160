
clear
clearvars -global
close all
load rotationtestdata


global timestamp wheelDispL wheelDispR xm ym trot;

hf = figure(2);
set(hf,'Units','Points');
set(hf,'Position',[650,550,350,300]);

h1 = plot(xm,ym,'*', 'Color', [     0    0.4470    0.7410]);
hold on
% quiver(xm, ym, 0.5*cos(Angledegrees/180*pi), -0.5*sin(Angledegrees/180*pi));
a = quiver(xm,ym,cos(trot),sin(trot), 0.2);
set(a,'Color', h1.Color);

inds = [26, 256, 620, 920, 1230, 1550, 1900];
xj = xJag(inds);
yj = yJag(inds);
tj = tJag(inds);
h2 = plot(xj,yj,'*', 'Color', [0.8500    0.3250    0.0980]);
a = quiver(xj,yj,cos(tj),sin(tj), 0.2);
set(a,'Color', h2.Color);
a = plot(xJag,yJag);
set(a,'Color', h2.Color);

legend([h1,h2],{'True State', 'Estimated State'}, 'Location', 'Best')

xlabel 'x [m]'
ylabel 'y [m]'

grid on

legend([h1,h2],{'True State', 'Estimated State'}, 'Location', 'Best')

title 'Full Circle Rotation Test'

%%
niter = 100;
R0 = 0.089;
b0 = 0.242;
mmse = objfun([b0,R0,R0]);

argsmin = Inf;
minmmse = Inf;

x0 = [b0,R0,R0];
lb = 0.8*[b0,R0,R0];
ub = 1.2*[b0,R0,R0];

[args_lb,minmmse_lb] = fminsearch(@objfun, lb);
[args_x0,minmmse_x0] = fminsearch(@objfun, x0);
[args_ub,minmmse_ub] = fminsearch(@objfun, ub);

%%

argsmin = args_lb;

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
    a = exo(i);
    b = eyo(i);
    x0 = xo(i);
    y0 = yo(i);
    t=-pi:0.01:pi;
    x=x0+a*cos(t);
    y=y0+b*sin(t);
    plot(x,y, '--', 'Color', h2.Color)
end

xlabel 'x [m]'
ylabel 'y [m]'

grid on

legend([h1,h2],{'True State', 'Optimized State'}, 'Location', 'Best')

title 'Full Circle Rotation Test'

% print -dpng -r500 RotTest.png
