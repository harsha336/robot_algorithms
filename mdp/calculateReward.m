function [ reward ] = calculateReward( low_bound,upper_bound,mu_inter )
%Calculate the reward for all future states for which the expectation of the 
% returns are calulated using r(s,a). The reward multiplied by getting the
% reward for a particular future state is sent back to the main mdp
% function.

pr = calculateProb(low_bound,upper_bound,mu_inter);
fid  = fopen('Rewards.txt','a');
fprintf(fid,'Prob for [%d,%d] = %d\n',low_bound(1),upper_bound(1),pr);
fclose(fid);
r = 0;
if ( low_bound(1)< -pi/4 || upper_bound(1) > pi/4 )
    r = -1;
end
reward = r*pr;
fid  = fopen('Rewards.txt','a');
fprintf(fid,'Reward for [%d,%d] = %d\n',low_bound(1),upper_bound(1),reward);
fclose(fid);
end

