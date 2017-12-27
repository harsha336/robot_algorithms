function [ thetaN,thetadotN ] = simulateSequence(thetain,thetadotin,u,deltaT)
%simulateSequence Compute the thetaN and thetadotN after a number of N
%steps.
%   The simulateOneStep and addNoise functions are used iteratively to
%   get thetaN and thetadotN
%-----------------------------------------------------------------------
%               Reusing the previous lab assignment code
%               The for loop is not required here
%-----------------------------------------------------------------------

    steps = length(u);
    sequence = zeros(steps,2);
    for k = 1:steps
        [theta,thetadot] = simulateOneStep(thetain,thetadotin,deltaT,u);
        [theta,thetadot] = addNoise(theta,thetadot);
        sequence(k,:) = double([theta thetadot]);
    end
    thetaN = sequence(steps,1);
    thetadotN = sequence(steps,2);
    
    %plot(x,sequence(:,1),x,sequence(:,2));
end

