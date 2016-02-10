function [ xx,yy,tt ] = integrate_wheeldisps( time, wdL, wdR, b, RL, RR )
%integrate_wheeldisps Integragtes the wheel displacements to estimate
%position given params

tt = zeros(size(time+1));
xx = tt;
yy = xx;

wdR = wdR*RR/0.089;
wdL = wdL*RL/0.089;

dist_traveled = (wdR + wdL)/2;
angle_traveled = (wdR - wdL)/(2*b);

for i = 2:length(tt)
    xx(i) = xx(i-1) + dist_traveled(i)*cos(tt(i-1) + angle_traveled(i));
    yy(i) = yy(i-1) + dist_traveled(i)*sin(tt(i-1) + angle_traveled(i));
    tt(i) = tt(i-1) + angle_traveled(i);
    if tt(i) > pi
        tt(i) = tt(i) - 2*pi;
    elseif tt(i) < -pi
        tt(i) = tt(i) + 2*pi;
    end
    
end





end

