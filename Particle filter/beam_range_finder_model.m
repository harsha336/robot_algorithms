function q = beam_range_finder_model( z,x )
% This is the sensor model. p(z|x) is calculated based on the sensor
% readings. Note: The sensor readings seems to be different from the one
% used in the previous lab as the probability and in turn the weights are
% generally very low. The state model seems to be different and hence the
% sensor readings should also have been different.

q = 1;
xi = [5 4 -3];
yi = [5 7 2];

for k = 1:3
    zstar = sqrt( (x(1)-xi(k))^2+(x(2)-yi(k))^2 ); 
    %zstar_bar = normrnd(zstar,sqrt(1));
    phit = normpdf(z(k),zstar,sqrt(0.001));
    q = q*phit;
end

end

