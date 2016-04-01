function [ output_args ] = plotConfidenceEllipse( xV, yV, xyCV, x0, y0 )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

Sigma = [xV, xyCV; xyCV, yV];
[V,D] = eig(Sigma);
D = [D(1,1),D(2,2)];
if(D(1) > D(2))
    L1 = D(1);
    L2 = D(2);
    v = V(:,1);
else
    L1 = D(2);
    L2 = D(1);
    v = V(:,2);
end

a = 2*sqrt(5.991*L1);
b = 2*sqrt(5.991*L2);
phi = atan2(v(2),v(1));

theta_grid = linspace(0,2*pi);

% the ellipse in x and y coordinates 
ellipse_x_r  = a*cos( theta_grid );
ellipse_y_r  = b*sin( theta_grid );

%Define a rotation matrix
R = [ cos(phi) sin(phi); -sin(phi) cos(phi) ];

%let's rotate the ellipse to some angle phi
r_ellipse = [ellipse_x_r;ellipse_y_r]' * R;

% Draw the error ellipse
plot(r_ellipse(:,1) + x0,r_ellipse(:,2) + y0,'-',...
    'Color', [0.4660    0.6740    0.1880])




end

