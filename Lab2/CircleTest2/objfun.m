function [ mmse ] = objfun( args )
%UNTITLED10 Summary of this function goes here
%   Detailed explanation goes here

global timestamp wheelDispL wheelDispR xm ym trot;

[x_int, y_int, t_int] = integrate_wheeldisps(timestamp,...
    wheelDispL, wheelDispR, args(1), args(2), args(3));

% inds = [1, 5, 19, 31, 44, 55, 69, 80, 92, 106, 115, 127, 139, 151, 163, 175, 187, 200]/0.053034300000000;
% inds = round(inds);
inds = [20, 140, 400, 600, 800, 1000, ...
    1250, 1450, 1650, 1900, 2100, 2300, ...
    2550, 2750, 2950, 3200, 3400, 3700];
inds = inds(1:14);
% [xx,yy, tt] = extract_stops(x_int,y_int,t_int);
xx = x_int(inds);
yy = y_int(inds);
tt = t_int(inds);


mmse = mean((xx-xm).^2 + (yy - ym).^2);


end

