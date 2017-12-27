mu = zeros(10,2);
sigma = [0.4 0;0 0.2];
hold('on');
for i = 1:5
    simulateSequence(i,i+2,i+1,i*5,u,sigma);
end