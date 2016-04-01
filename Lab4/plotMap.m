function [ h ] = plotMap( h )
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

h = figure(h);

x1 = [3.38 + 5.79 + 3.55 /2 , -3.38 - 5.79 - 3.55 / 2];
y1 = [2.794, 2.794];

x2 = [-3.55/2, -3.55/2];
y2 = [0.0, -2.74];

x3 = -x2;
y3 = y2;

x4 = [3.55/2, 3.55 / 2 + 5.79];
y4 = [0,0];

x5 = -x4;
y5 = y4;

x6 = [-3.55/2, -3.55/2-3.05];
y6 = [-2.74, -2.74];

x7 = -x6;
y7 = y6;

x8 = [5.03 / 2, -5.03/2];
y8 = [-2.74 - 2.31, -2.74 - 2.31];

C = [  0.6350    0.0780    0.1840];

hold on
plot(x1,y1,'Color',C);
plot(x2,y2,'Color',C);
plot(x3,y3,'Color',C);
plot(x4,y4,'Color',C);
plot(x5,y5,'Color',C);
plot(x6,y6,'Color',C);
plot(x7,y7,'Color',C);
plot(x8,y8,'Color',C);

grid on


end

