function [ prob ] = calculateProb( Xl,Xu,mu_inter )
%Calculates the probability of the future state lying in a particular
%region.

sigma = [0.01 0;0 0.001];
prob = mvncdf(Xl,Xu,mu_inter,sigma);
%fid = fopen('Probabilities.txt','a');
%fprintf(fid,'Probability of interval [%d,%d]-[%d,%d] with mean [%d,%d] is %d\n',Xl(1),Xl(2),Xu(1),Xu(2),mu_inter(1),mu_inter(2),prob);
%fclose(fid);
end