function simulatePendulum()
%This is only a wrapper function to call the main mdp function. As the
%question required a function simulatePendulum which needed to be the entry
%point.

% Calling mdp with only 6 discrete intervals. Out of the 15 times the
% sequence was completed till 500 steps for 12 times. The remaining 3 times
% the range of steps completed was 250-350. The time taken to compute the
% policy is ~20 min.
mdp(6);

% mdp was called with discrtization up till 10 state spaces. This took
% ~2 hours. Hence not using mdp(10).
end

