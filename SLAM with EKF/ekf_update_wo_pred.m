function [ mu,sigma ] = ekf_update_wo_pred( mu,sigma,u,N )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
deltaT = 1;
varx = 0.01;
vart = 0.001;
model_noise = [0.01 0 0;0 0.01 0;0 0 0.001];

Fx = horzcat(eye(3),zeros(3,2*N));
%disp("Fx is:");
%disp(Fx);
%disp("Mu is:");
%disp(mu);
%disp("Sigma is:");
%disp(sigma);
%x = x + ut * deltaT * cos(theta) + sqrt(varx)*randn(1);
%y = y + ut * deltaT * sin(theta)+ sqrt(varx)*randn(1);
%theta = theta + ur*deltaT+ sqrt(vart)*randn(1);
mu_model_vec = [u(1)*cos(mu(3))*deltaT; 
    u(1)*sin(mu(3))*deltaT;
    u(2)*deltaT ];
%disp("Mu model vec is");
%disp(Fx'*mu_model_vec);
mubar = mu' + (Fx'*mu_model_vec);
%disp("Mu is:");
%disp(mubar);
sigma_model_vec = [0 0 -u(1)*sin(mu(3))*deltaT;
    0 0 u(1)*cos(mu(3))*deltaT;
    0 0 0];
G = eye(3+2*N,3+2*N) + (Fx' * sigma_model_vec * Fx);
sigmabar = (G*sigma*G') + (Fx'*model_noise*Fx);
mu = mubar;
sigma = sigmabar;

end

