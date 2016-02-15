function [ xx,yy,tt,ex,ey,et ] = integrate_wheeldisps( time, wdL, wdR, b, RL, RR )
%integrate_wheeldisps Integragtes the wheel displacements to estimate
%position given params

tt = zeros(size(time+1));
xx = tt;
yy = xx;
ex = xx;
ey = xx;
et = xx;

eRR = 0.02;
eRL = eRR;
eb = 0.02;

wdR = wdR*RR/0.089;
wdL = wdL*RL/0.089;

esR = wdR/RR * eRR;
esL = wdL/RL * eRL;

dist_traveled = (wdR + wdL)/2;
eDs = 0.5*sqrt(esL.^2 + esR.^2);

angle_traveled = (wdR - wdL)/(2*b);
eDt = sqrt( (1/(2*b))^2 * (esL.^2 + esR.^2) +...
    ((wdR - wdL)/(2*b^2)).^2 * eb^2);

for i = 2:length(tt)
    xx(i) = xx(i-1) + dist_traveled(i)*cos(tt(i-1) + angle_traveled(i));
    yy(i) = yy(i-1) + dist_traveled(i)*sin(tt(i-1) + angle_traveled(i));
    tt(i) = tt(i-1) + angle_traveled(i);
    if tt(i) > pi
        tt(i) = tt(i) - 2*pi;
    elseif tt(i) < -pi
        tt(i) = tt(i) + 2*pi;
    end
    
    et(i) = sqrt( et(i-1)^2 + eDt(i)^2 );
    ex(i) = sqrt( ex(i-1)^2 + cos(tt(i))^2 * eDs(i)^2 + ...
        dist_traveled(i)^2*sin(tt(i))^2*et(i)^2);
    ey(i) = sqrt( ey(i-1)^2 + sin(tt(i))^2 * eDs(i)^2 + ...
        dist_traveled(i)^2*cos(tt(i))^2*et(i)^2);
    
    
end





end

