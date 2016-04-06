clc;
clear;


%% Parameters

L = 200;
N = 80;     % Number of robots
t_start = 0;    % seconds
t_end = 10;     % End time (seconds)
del_t = 0.5;
t = linspace(t_start, t_end, (t_end-t_start)/del_t);
alpha = 0.07;
beta = 0.05;
%strength for attraction and repulsion
Ca = 20;
Cr = 50;
%length for attraction and repulsion
la = 100;
lr = 2;
v = 0.3;
%mass of each robot
m = 1/N;
prefered_vel = sqrt(alpha/beta);


%% Initial position of robots


x = L.*rand(N,2);
theta = 2*pi.*rand(N,1);                % Initial heading (radians)
vel = [v.*cos(theta), v.*sin(theta)];   % Initial velocities (m/s)


%% simualation

t = 0;

while (t < t_end)
    
    for k= 1:N
        for l= 1:N
            if (k ~= l)
                U = -Ca.*exp(-abs(x(k) - x(l))./la) + Cr.* exp(-abs(x(k) - x(l))./lr); % ?
                a = (alpha - beta.*(abs(vel).^2) - m.*U.*sum(x(k) -x(l)));             % ?
                quiver(x(:,1),x(:,2),a(:,1),a(:,2),0.2)
                %plot(x(:,1),x(:,2), 'bo')
                pause(0.1)
            
            end
        end
    end
    
    t = t + del_t;

end
