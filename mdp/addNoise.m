function [ thetaN,thetadotN ] = addNoise(theta,thetadot)
%addNoise Generates a noise and adds it to simulated theta ans thetdot
%   A random vector is generated with the given mean and covariance matrix.
%   This random vector of sixe 1X2 is added to theta and theadot.
mu = [0 0];
sigma = [0.01 0;0 0.001];
noise = mvnrnd(mu,sigma,1);
thetaN = theta + noise(1);
thetadotN = thetadot + noise(2);
%fprintf('addNoise(%d,%d):The values of theta and theta dot after adding noise:%d , %d\n',theta,thetadot,thetaN,thetadotN);
end

