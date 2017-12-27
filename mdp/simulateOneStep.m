function [ thetaN,thetadotN ] = simulateOneStep( theta,thetaDot,deltaT,u )
%simulateOneStep - This calculates theta and thetadot at N using euler
%integration
%   This is based on the fact that for the given function
%   dx/dt = f(x,u), we know that dx/dt = (x at (t+h) - x at t)/h
%   Hence (x at (t+h)) = (x at t) + h*f(x,u)
    
    J = 1;gamma = 1;l = 1;m = 1;g = 9.8;
    
    
    thetaN = theta + thetaDot*deltaT;
    thetadotN = thetaDot + (((m*g*l*sin(theta))/J)-((gamma*thetaDot)/J)+((l*cos(theta)*u))/J)*deltaT;
    
    %fprintf('simulateOneStep(%d,%d,%d,%d):The values of theta and theta dot after adding noise:%d , %d\n',theta,thetaDot,deltaT,u,thetaN,thetadotN);
end

