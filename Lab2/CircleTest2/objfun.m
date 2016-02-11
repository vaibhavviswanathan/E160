function [ mmse ] = objfun( args )
%UNTITLED10 Summary of this function goes here
%   Detailed explanation goes here

global timestamp wheelDispL wheelDispR xm ym trot;

[x_int, y_int, t_int] = integrate_wheeldisps(timestamp,...
    wheelDispL, wheelDispR, args(1), args(2), args(3));

inds = [26, 256, 620, 920, 1230, 1550, 1900];
% [xx,yy, tt] = extract_stops(x_int,y_int,t_int);
xx = x_int(inds);
yy = y_int(inds);
tt = t_int(inds);


mmse = mean((xx-xm).^2 + (yy - ym).^2 +((tt - trot) * args(2)).^2);

end

