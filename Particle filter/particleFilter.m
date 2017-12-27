function [chinew,flag] = particleFilter( chi,u,z,m,w )
% This function contains the particle filter algorithm.
chinewbar = zeros( 3,m );
for i = 1:m
    % The state motion model is applied to find the next state of each of
    % the particles.
    [chinewbar(1,i),chinewbar(2,i),chinewbar(3,i)] = sample_motion_model_velocity(u,chi(:,i));
    % The probability returned by the sensor model is used to assign
    % weights to each of the particles
    w(i) = w(i)*beam_range_finder_model(z,chinewbar(:,i));
end

chinew = zeros(3,m);
% neff for adaptive resampling is used.
%neff = 1/sum(w.^2);
if sum(w) > 0
    % As observed from the outputs neff is generally small and hence
    % resampling is almost always done.
    for i = 1:m
        w = w./sum(w);
        index = find(rand <= cumsum(w),1);
        chinew(:,i) = chinewbar(:,index);
    end
    flag = 1;
elseif sum(w) == 0
    disp("It is not less than the value");
    chinew = [ randi([-4 6],1,m); randi([-3 10],1,m); randi([0 7],1,m)];
    flag = 0;
end

end

