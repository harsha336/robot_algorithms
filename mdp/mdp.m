function [policy] = mdp(interval)

gamma = 0.95;

set_size = interval + 1;

% Discretizing theta space. Having the lower bound and upper bound of theta
% in each theta space.
theta = zeros(set_size);
theta_period = (pi/4)/(interval/2);
theta_state = zeros(set_size,2);
theta(1) = -pi/4;
theta_state(1,:) = [-Inf -pi/4];
for i = 2:set_size
    theta(i) = theta(i-1) + theta_period;
    theta_state(i,:) = [theta(i-1) theta(i)];
end
theta_state(i+1,:) = [theta(i) +Inf];

state_space = i+1;

% Discretizing thetadot space. Having the lower bound and upper bound of theta
% in each thetadot space.
thetadot = zeros(set_size);
thetadot_period = (2*pi/3)/(interval/2);
thetadot(1) = -2*pi/3;
thetadot_state = zeros(set_size,2);
thetadot_state(1,:) = [-Inf -2*pi/3];
for i = 2:set_size
    thetadot(i) = (thetadot(i-1) + thetadot_period);
    thetadot_state(i,:) = [thetadot(i-1) thetadot(i)];
end
thetadot_state(i+1,:) = [thetadot(i) +Inf];

% Discretizing forces.
u = zeros(set_size);
for i=1:interval/2 
    u(i) = -i*3;
end
for i=interval/2+1:interval 
    u(i) = (i-((interval/2)))*3;
end
u(i+1) = 0;

% Calculating the values here. Having a matrix for values corresponding to
% each theta and thetadot space.
value = zeros(state_space,state_space);
omega = 0;
count = 0;

disp('Calculating optimal values...');
disp('Please wait!');

% Restricting to two counts as the number of times value optimization done
% did not improve the efficiency by a great margin.
while ((omega < 1) && (count < 2))
    
    for i = 1:state_space
        for j = 1:state_space
            prev_value = value(i,j);
            inter_value = zeros(set_size);
            % Applying action 'a' (force) to each state space
            for k = 1:set_size
                
                % Apply action for the current state and find the mean of the
                % next states
                
                if ( i == 1 )
                    calc_theta = theta_state(i,2)-((pi/4)/(interval/2))/2;
                elseif ( i == state_space )
                    calc_theta = theta_state(i,1)+((pi/4)/(interval/2))/2;
                else
                    calc_theta = (theta_state(i,1)+theta_state(i,2))/2;
                end
                
                if (j == 1 )
                    calc_thetadot = thetadot_state(j,2)-((2*pi/3)/(interval/2))/2;
                elseif ( j == state_space)
                    calc_thetadot = (thetadot_state(j,1)+(2*pi/3)/(interval/2))/2;
                else
                    calc_thetadot = (thetadot_state(j,1)+thetadot_state(j,2))/2;
                end
                [thetaN,thetadotN] = simulateOneStep(calc_theta,calc_thetadot,0.1,u(k));
                mu_inter = [thetaN,thetadotN];
                value_star = 0;
                rsa = 0;
                % Calculating the probability p(s'|s,a) including the -Inf
                % and +Inf range as well.
                for l = 1:state_space
                    for m = 1:state_space
                        prob_low_bound = [theta_state(l,1) thetadot_state(m,1)];
                        prob_upper_bound = [theta_state(l,2) thetadot_state(m,2)];
                        pr = calculateProb(prob_low_bound,prob_upper_bound,mu_inter);
                        rsa = rsa + calculateReward(prob_low_bound,prob_upper_bound,mu_inter);
                        value_star = value_star + pr*value(l,m);
                    end
                end
                value_sum = rsa + gamma*value_star;
                inter_value(k) = value_sum;
            end
            
            %Calculating the max of each action
            max = 1;
            for n = 1:interval
                if (inter_value(n) > inter_value(max))
                    max = n;
                end
            end
            value(i,j) = inter_value(max);
            
            fid = fopen('values.txt','a');
            fprintf(fid,'Values of [%d,%d] - %d',i,j,value(i,j));
            fclose(fid);
            
            delta = abs(prev_value - value(i,j));
            if omega<delta
                omega = delta;
            end 
            fid = fopen('Omega.txt','a');
            fprintf(fid,'Omega value = %d\n',omega);
            fclose(fid);
        end
    end
    count = count + 1;
end

% Calculating the optimal policy here. A matrix policy used to hold the
% action to be applied for each interval.
disp('Calculating optimal policy...');
disp('Please wait!');

policy = zeros(state_space,state_space);
for i = 1:state_space
    for j = 1:state_space
        max = -99999;
        for k = 1:set_size
            
            % Apply action for the current state and find the mean of the
            % next states
            
            if ( i == 1 )
                calc_theta = theta_state(i,2)-((pi/4)/(interval/2))/2;
            elseif ( i == state_space )
                calc_theta = theta_state(i,1)+((pi/4)/(interval/2))/2;
            else
                calc_theta = (theta_state(i,1)+theta_state(i,2))/2;
            end
            if (j == 1 )
                calc_thetadot = thetadot_state(j,2)-((2*pi/3)/(interval/2))/2;
            elseif ( j == state_space)
                calc_thetadot = thetadot_state(j,1)+((2*pi/3)/(interval/2))/2;
            else
                calc_thetadot = (thetadot_state(j,1)+thetadot_state(j,2))/2;
            end
            
            [thetaN,thetadotN] = simulateOneStep(calc_theta,calc_thetadot,0.1,u(k));
            sum_state = 0;
            rsa = 0;
            mu_inter = [thetaN thetadotN];
            for l = 1:state_space
                for m = 1:state_space
                    prob_low_bound = [theta_state(l,1) thetadot_state(m,1)];
                    prob_upper_bound = [theta_state(l,2) thetadot_state(m,2)];
                    rsa = rsa + calculateReward(prob_low_bound,prob_upper_bound,mu_inter);
                    pr = calculateProb(prob_low_bound,prob_upper_bound,mu_inter);
                    sum_state = sum_state + pr*value(l,m);
                end
            end
            inter_sum_value = rsa + gamma*sum_state;
            if (inter_sum_value > max)
                max = inter_sum_value;
                policy(i,j) = k; 
            end
        end
        
        fid = fopen('Policy.txt','a');
        fprintf(fid,'Policy of [%d,%d] = %d\n',i,j,policy(i,j));
        fclose(fid);
    end
end

while 1
    prompt = 'Do you want to generate a sequence?[Y/N]';
    choice = input(prompt,'s');
    if (strcmp(choice,'N'))
        break;
    end
    
    prompt = 'Input the theta value(Balances only with 0):';
    theta_inp = input(prompt);
    if isempty(theta_inp)
        continue;
    end
    
    prompt = 'Input thetadot value(Balances only with 0:)';
    thetadot_inp = input(prompt);
    if isempty(thetadot_inp)
        continue;
    end
    
    steps = 1;
    sequence = zeros(500,2);
    
    % Iterating for 500 times until theta is not out of bounds. 
    while (theta_inp > -pi/4 && theta_inp<pi/4 && steps <= 500)
        pol_ind1 = 0;
        pol_ind2 = 0;
        for i=1:state_space
            if (theta_inp >= theta_state(i,1) && theta_inp < theta_state(i,2))
                pol_ind1 = i;
            end
        end
    
        for i=1:state_space
            if (thetadot_inp >= thetadot_state(i,1) && thetadot_inp < thetadot_state(i,2))
                pol_ind2 = i;
            end
        end
        if (~pol_ind1 || ~pol_ind2)
            fprintf('Theta is out of bounds [%d], Thetadot is out of bounds [%d]\n',~pol_ind1,~pol_ind2);
            break;
        else
            k = policy(pol_ind1,pol_ind2);
            force = u(k);
            [new_theta,new_thetadot] = simulateSequence(theta_inp,thetadot_inp,force,0.1);
            fprintf('Action %d applied on [%d %d] to get [%d %d]\n',force,theta_inp,thetadot_inp,new_theta,new_thetadot);
            theta_inp = new_theta; thetadot_inp = new_thetadot;
            sequence(steps,:) = [theta_inp force];
            if (theta_inp < -pi/4 || theta_inp > pi/4)
                fprintf('Pole travelled out of bounds!\n');
            end
        end
        steps = steps +1;
        pause(1);
    end
    
    fid = fopen('action.txt','a');
    for i=1:steps-1
        fprintf(fid,'Action %d taken to get %d\n',sequence(i,2),sequence(i,1));
    end
    
    fclose(fid);
    figure
    plot(sequence(:,1));
    hold on
    plot(sequence(:,2));
    title('Graph of theta and actions')
    xlabel('steps');
    ylabel('theta and actions');
    legend('theta','action');
end
end
            