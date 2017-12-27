function [ xprime, yprime, thetaprime ] = sample_motion_model_velocity( u,x )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
deltaT = 0.5;
alpha = [0.01 0.01 0.01 0.01 0.0005 0.0005];
vvar = alpha(1)*abs(u(1))+alpha(2)*abs(u(2));
wvar = alpha(3)*abs(u(1))+alpha(4)*abs(u(2));
vbar = u(1)+normrnd(0,vvar);
wbar = u(2)+normrnd(0,wvar);
gbar = normrnd(0,alpha(5)*abs(u(1))+alpha(6)*abs(u(2)));
xprime = x(1)-((vbar/wbar)*sin(x(3)))+((vbar/wbar)*sin(x(3)+(wbar*deltaT)));
yprime = x(2)+((vbar/wbar)*cos(x(3)))-((vbar/wbar)*cos(x(3)+(wbar*deltaT)));
thetaprime = x(3)+(wbar*deltaT)+(gbar*deltaT);
end

