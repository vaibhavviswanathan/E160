function [ lambdas ] = getLambdas( xV,yV,xyV )
%UNTITLED7 Summary of this function goes here
%   Detailed explanation goes here


lambdas = zeros(size(xV));
for i = 1:length(xV)
    Sigma = [xV(i), xyV(i); xyV(i), yV(i)];
    [~,D] = eig(Sigma);
    lambdas(i) = max(max(D));

end
end

