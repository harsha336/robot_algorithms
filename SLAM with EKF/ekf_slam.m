function [ mu,sigma ] = ekf_slam( mu,sigma,u,z,N )
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
%alpha = 5e2;
alpha = 1e3;
deltaT = 1;
model_noise = [0.01 0 0;0 0.01 0;0 0 0.001];
sensor_noise = [0.01 0;0 0.01];

Fx = horzcat(eye(3),zeros(3,2*N));

%x = x + ut * deltaT * cos(theta) + sqrt(varx)*randn(1);
%y = y + ut * deltaT * sin(theta)+ sqrt(varx)*randn(1);
%theta = theta + ur*deltaT+ sqrt(vart)*randn(1);

mu_model_vec = [u(1)*cos(mu(3))*deltaT; 
    u(1)*sin(mu(3))*deltaT;
    u(2)*deltaT ];

mubar = mu' + (Fx'*mu_model_vec);

sigma_model_vec = [0 0 -u(1)*sin(mu(3))*deltaT;
    0 0 u(1)*cos(mu(3))*deltaT;
    0 0 0];
G = eye(3+2*N,3+2*N) + (Fx' * sigma_model_vec * Fx);
sigmabar = (G*sigma*G') + (Fx'*model_noise*Fx);
[size_sig_r,size_sig_c] = size(sigmabar);
kallman_sum = zeros(size_sig_r,size_sig_c);

N_bkp = N;
count = 1;

for zi = z
    disp("99999999999999999999999999999999");
    zi(2) = wrapToPi(zi(2));
    mubar(3+2*N+1) = mubar(1) + zi(1) * cos(wrapToPi(zi(2)+mubar(3)));
    mubar(3+2*N+2) = mubar(2) + zi(1) * sin(wrapToPi(zi(2)+mubar(3)));
    disp(zi);disp("in");disp(z);
    
    size_temp = size(sigmabar,2);
    sigmabar = vertcat(sigmabar,zeros(2,size_temp));
    kallman_sum = vertcat(kallman_sum,zeros(2,size_temp));
    inf_mat = [999999 0;0 999999];
    zero_mat = [0 0;0 0];
    sigmabar = horzcat(sigmabar,vertcat(zeros(size_temp,2),inf_mat));
    kallman_sum = horzcat(kallman_sum,vertcat(zeros(size_temp,2),zero_mat));
    
    N = N + 1;
    H = [];
    
    disp("mubar computed:");disp(mubar);
    man_dis = zeros(1,N);
    for k = 1:N
        
        del(1) = (mubar(3+2*(k-1)+1) - mubar(1));
        del(2) = (mubar(3+2*(k-1)+2) - mubar(2));
        q = del * del';
        bear_ang = atan2(del(2),del(1)) - mubar(3);
        %if bear_ang > pi+pi/15 || bear_ang < -pi-pi/15
        zcap(:,k) = [sqrt(q);wrapToPi(atan2(del(2),del(1)) - mubar(3))];
        %else
        %    zcap(:,k) = [sqrt(q);(atan2(del(2),del(1)) - mubar(3))];
        %end
        zcap_ww(:,k) = [sqrt(q);(atan2(del(2),del(1)) - mubar(3))];
        Fxk = vertcat(horzcat(eye(3),zeros(3,2*k-2),zeros(3,2),zeros(3,2*N-2*k)),horzcat(zeros(2,3),zeros(2,2*k-2),eye(2),zeros(2,2*N-2*k)));
        inter_H = [-sqrt(q)*del(1) -sqrt(q)*del(2) 0 sqrt(q)*del(1) sqrt(q)*del(2);
            del(2) -del(1) -q -del(2) del(1)];
        Hk = (1/q)*inter_H*Fxk;
        H = [H;Hk];
        temp_man_var = (Hk*sigmabar*Hk') + sensor_noise;
        disp("Difference is:");
        disp(zi-zcap(:,k));
        z_diff = [zi(1) - zcap(1,k);wrapToPi(zi(2) - zcap(2,k))];
        disp("Index diff is:");
        disp(z_diff);
        man_dis(1,k) = abs((z_diff)'*inv(temp_man_var)*(z_diff));
        man_var(2*(k-1)+1:2*(k-1)+2,:) = temp_man_var;
    end
    var_eg = eig(sigma(1:3,1:3));
    %var_eg = eig(sigma(1:3,1:3));
    var_e = sum(var_eg);
    disp("eigen value");disp(var_e);
    var_dist = var_e*alpha;
    disp("Variance det");
    disp(var_dist);
    man_dis(1,N) = var_dist;
    disp("Taking alpha as:");disp(man_dis(1,N));
    disp("Priinting all the distances:");
    disp(man_dis(1,:));
    %disp("Value of N before:");disp(N);
    [minval,ji] = min(man_dis);
%     if N>ji
%         N = N_bkp;
%         mubar = mubar(1:3+2*N);
%         sigmabar = sigmabar(1:3+2*N,1:3+2*N); 
%     end
%     if N_bkp<ji
%         kallman_sum = [kallman_sum;zeros(2,2)];
%     end
    
    N = max(N_bkp,ji);
    mubar = mubar(1:3+2*N);
    sigmabar = sigmabar(1:3+2*N,1:3+2*N);
    kallman_sum = kallman_sum(1:3+2*N,1:3+2*N);
    %disp("Value of N after:");disp(N);
    disp("Values of z and zcap:");disp(zi);disp(zcap(:,ji));
    disp("Printing all values of z here:");disp(zcap);
    disp("Printing unwrapped z here:");disp(zcap_ww);
    disp("Something going on here!");
    
    Hji = H(2*(ji-1)+1:2*(ji-1)+2,1:3+2*N);
    man_var_ji = man_var(2*(ji-1)+1:2*(ji-1)+2,:);
    %disp("Hji");disp(Hji);disp("man_var_ji:");disp(man_var_ji);disp("sigmabar:");disp(sigmabar);
    K = sigmabar*Hji'*inv(man_var_ji);
    %disp("Size of kallman_sum:");disp(size(kallman_sum));
    %disp("Size of K:");disp(size(K));
    %disp("Size of H:");disp(size(H));
    z_diff_fin = [zi(1)-zcap(1,ji);wrapToPi(zi(2)-zcap(2,ji))];
    mubar = mubar + K*(z_diff_fin);
    kallman_sum = kallman_sum + K*Hji;
    sigmabar = sigmabar - K*Hji*sigmabar;
    disp("Printing mubar here!");disp(mubar);
    disp("99999999999999999999999999999999");
    
    count = count + 1;
end

mubar(3) = wrapToPi(mubar(3));
sigmabar = sigmabar - kallman_sum*sigmabar;
mu = mubar;
sigma = sigmabar;

end

