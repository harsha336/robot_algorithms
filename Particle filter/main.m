function main( m )
% The main function which calls the particle filter algorithm.

w = zeros(m,1);
chi = [ randi([-4 6],1,m); randi([-3 10],1,m); randi([0 7],1,m)];
fid  = fopen('inputs.txt','r');
formatSpec = '%f %f';
sizeu = [2 Inf];
u = fscanf(fid,formatSpec,sizeu);
fclose(fid);
for i = 1:m
    w(i) = 1/m;
end
fid = fopen('sensor_readings.txt','r');
formatSpec = '%f %f %f';
sizez = [3 Inf];
z = fscanf(fid,formatSpec,sizez);
fclose(fid);
t = length(z);
figure;
for i = 1:t
    [newchi,flag] = particleFilter(chi,u(:,i),z(:,i),m,w);
    chi = newchi;
    if flag == 1
        plot(chi(1,:),chi(2,:));
        hold on;
    end
   
end
title('Pose of the robot at each time step');
figure;
plot(chi(1,:),chi(2,:),'o');
title('Final population of particles');
fin_pose = [mean(chi(1,:)) mean(chi(2,:)) mean(chi(3,:))];
disp(fin_pose);

end

