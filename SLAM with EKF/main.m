function main( )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
init_simulator();
fg = load('swap.mat');
disp(fg);
alpha_rot = 1.5e2;
alpha_str = 8e2;
num_lan = 8;
mu = [0 0 0];
sigma = [0.01 0 0;0 0.01 0;0 0 0.001];
mu_prev = [0 0]';
N = 0;
sequence = [0 0];
f = figure('Visible','off','Position',[360,500,450,285]);

% Construct the components.
hup    = uicontrol('Style','pushbutton',...
             'String','Move forward','Position',[315,220,70,25],...
             'Callback',@move_up_callback);
hdown  = uicontrol('Style','pushbutton',...
             'String','Move backward','Position',[315,180,70,25],...
             'Callback',@move_down_callback);
hleft = uicontrol('Style','pushbutton',...
             'String','Move left','Position',[315,135,70,25],...
             'Callback',@move_left_callback);
hright = uicontrol('Style','pushbutton',...
             'String','Move right','Position',[315,90,70,25],...
             'Callback',@move_right_callback);
hroam = uicontrol('Style','pushbutton',...
             'String','Find On Auto','Position',[315,65,70,25],...
             'Callback',@simulate_callback);
hrotstep = uicontrol('Style','pushbutton',...
             'String','Rotate stepwise','Position',[315,40,70,25],...
             'Callback',@rotate_spiral);


ha = axes('Units','pixels','Position',[50,60,200,185]);
align([hup,hdown,hleft,hright,hroam,hrotstep],'Center','None');

% Initialize the UI.
% Change units to normalized so components resize automatically.
f.Units = 'normalized';
ha.Units = 'normalized';
hup.Units = 'normalized';
hdown.Units = 'normalized';
hright.Units = 'normalized';
hleft.Units = 'normalized';
hroam.Units = 'normalized';
hrotate.Units = 'normalized';
hrotstep.Units = 'normalized';
plotKnownData();
f.Visible = 'on';

    function plotKnownData()
        %seq(1,:) = mu_prev(1:2);
        %seq(2,:) = mu(1:2);
        %mu_cur = [mu(1) mu(2)];
        %disp("oooooooooooooooooooooooooooooooo");
        %disp(sequence);
        %disp("oooooooooooooooooooooooooooooooo");
        plot(sequence(:,1),sequence(:,2),'b--D');
        hold on;
        xlim([-30 30]);
        ylim([-30 30]);
        %mu_prev = mu_cur;
        %disp("Plotting pose:");disp(mu(1:2));disp(sigma(1:2,1:2));
        temp_sig = sigma(1:2,1:2)
        temp_sig(2,1) = temp_sig(1,2);
        plotErrorEllipsoid(mu(1:2),temp_sig);
        if N>0
            for k = 1:N
                plotErrorEllipsoid(mu(3+2*(k-1)+1:3+2*(k-1)+2),sigma(3+2*(k-1)+1:3+2*(k-1)+2,3+2*(k-1)+1:3+2*(k-1)+2));
                %disp("Plotting for:");disp(mu(3+2*(k-1)+1:3+2*(k-1)+2));disp(sigma(3+2*(k-1)+1:3+2*(k-1)+2,3+2*(k-1)+1:3+2*(k-1)+2));
            end
        end
        hold off;
        %plotLandmarks();
    end

%     function plotLandmarks()
%         figure;
%         x = zeros(N);
%         y = zeros(N)
%         for i=1:N
%             x_land(i) = mu(3+(2*i)-1);
%             y_land(i) = mu(3+(2*i));
%         plot(mu(3:3+2*N

    function move_up_callback(~,~)
        u = [1 0];
        move_robot(1,0);
        l = get_landmarks();
        %z = z';
        processLandmarks(l,u);
    end

    function move_down_callback(~,~)
        u = [0 -pi];
        move_robot(0,-pi);
        l = get_landmarks();
        %z = z';
        processLandmarks(l,u);
    end

    function move_left_callback(~,~)
        u = [0 pi/2];
        move_robot(0,pi/2);
        l = get_landmarks();
        processLandmarks(l,u);
    end

    function rotate_spiral(~,~)
        move_spiral();
        back_to_center();
    end

    function move_right_callback(~,~)
        u = [0 -pi/2];
        move_robot(0,-pi/2);
        l = get_landmarks();
        processLandmarks(l,u,plan);
    end

    function move_custom(theta)
        u = [0 theta];
        move_robot(0,theta);
        l = get_landmarks();
        processLandmarks(l,u);
    end

    function processLandmarks(l,u)
        disp("##############################################");
        sw = load('swap.mat');
        disp(sw);
        
        
        N = (length(mu) - 3)/2;
        N_prev = N;
        if ( isempty(l) )
            disp("No sensor readings!");
            [mu,sigma] = ekf_update_wo_pred( mu,sigma,u,N );
        else
            disp("Sensor readings present");
            disp("Sensor values after moving robot is:");
            z = [l.d; l.b];
            disp(z);
            [mu,sigma] = ekf_slam(mu,sigma,u,z,N);
        end
        mu = mu';
        sequence = [sequence;mu(1:2)];
        N = (length(mu) - 3)/2;
        disp("Number of landmarks:");
        disp(N);
        if N ~= N_prev
            disp("New landmark found!");
            %pause;
        end
        disp("State Vector is:");disp(mu);
        %disp(sigma);
        [size_mu,dummy] = size(mu);
        disp("///////////////////////////");
        sw = load('swap.mat');
        disp(sw);
        disp(size_mu);disp(dummy);
%         if (dummy > 11)
%             pause;
%         end
        disp("///////////////////////////");
        disp("##############################################");
        plotKnownData();
    end

    function simulate_callback(~,~)
        cover = 0;
        while cover < 8
            move_custom(-mu(3));
            move_custom(cover*pi/4);
            move_straight();
            back_to_center();
            cover = cover + 1;
            
            
        end
    end

    function move_spiral()
        simulate = 1;
        move_steps = 0;
        while simulate
            move_steps = move_steps+2;
            bucket =move_steps;
            while bucket > 0
                move_up_callback();
                bucket = bucket - 1;
                var = det(sigma(1:3,1:3));
                disp(var);
                if var > 0.002
                    simulate = 0;
                    disp("I am lost: ");disp(bucket);disp(mu(1));
                    break;
                end
                pause(0.1);
            end
            move_left_callback();
        end
    end

    function move_straight()
        simulate = 1;
        move_steps = 0;
        while simulate
            move_steps = move_steps+2;
            bucket =move_steps;
            while bucket > 0
                move_up_callback();
                bucket = bucket - 1;
                var = det(sigma(1:3,1:3));
                disp(var);
                if var > 0.002
                    simulate = 0;
                    disp("I am lost: ");disp(bucket);disp(mu(1));
                    break;
                end
                pause(0.1);
            end 
        end
    end

    function back_to_center()
          trial = 0;
          while trial < 5
              trial=trial+1;
              disp("Came here as I am lost!");
              
              rotate_robot = atan2(mu(2),mu(1)) - wrapToPi(mu(3)) -pi;
              move_custom(rotate_robot);
              Xl = [-1 -1];
              Xu = [1 1];
             count = 0;
             while (mvncdf(Xl,Xu,mu(1:2),[sigma(1,1) 0;0 sigma(2,2)])<0.95 && count<10)
                move_up_callback();
                count = count + 1;
                var = det(sigma(1:3,1:3));
                disp(var);
                if var < 0.000002
                    break;
                end
                pause(0.1);
            end
          end
    end

end


