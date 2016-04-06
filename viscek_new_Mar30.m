clc;
clear;


%% Parameters
L = 25;     % periodic boundary
N = 300;     % Number of robots
p = (N/L^2); % Density

t_start = 0;    % seconds
t_end = 10;     % End time (seconds)
del_t = 0.5;
t = linspace(t_start, t_end, (t_end-t_start)/del_t);
r = 1;           % interaction radius
v = 0.1;          % m/s
eta = 0.1;      % degrees
% theta_noise = unifrnd(-eta/2, eta/2);
theta_noise = -eta/2 + eta*rand(1);

%% Initial positions of the robots
%x = L.*rand(N,1);
%y = L.*rand(N,1);
% plot(x,y,'gd');
x = L.*rand(N,2);                       % Initial positions of the robots (m)
theta = 2*pi.*rand(N,1);                % Initial heading (radians)
% v = sin(theta)/cos(theta);
vel = [v.*cos(theta), v.*sin(theta)];   % Initial velocities (m/s)
% plot(x(:,1),x(:,2),'bo');
% quiver(x(:,1), x(:,2), vel(:,1), vel(:,2), 0.2);

%plot(x(:,1),theta,'.')




%% Run simulation
t = 0;
x_prev = x;
theta_prev =theta;
theta_avg = theta;
L_prev = L;
eta_prev = eta;
while (t < t_end)

%     L = 40;
    % Calculating density of robots
    p = (N/L^2);
    % BUG: Must not change L within single simulation
    L_prev = L - 1;
    L = L_prev;
    figure(10);
    avg_vel = (1/N.*v) *sqrt((sum(vel(:,1)))^2 + (sum(vel(:,2)))^2);
    plot(p,avg_vel,'b.');
    xlim([0, 10])
    ylim([0, 1])
    xlabel('density');
    ylabel('avg vel');
    hold on;
    

%     eta_prev = eta + 1 ;
%     eta = eta_prev;
%     figure(20);
%     plot(eta,avg_vel,'gs');
%     xlim([0, 10])
%     ylim([0, 1])
%     xlabel('eta');
%     ylabel('avg_vel');
%     hold on;
%     

    x_pos = zeros(N,2);
    %x_pos = zeros(N,2);
    average = zeros(N,1);
    
    for j = 1 : N  % for a robot
        %disp(j)
        for k = 1 : N % remaining robots
            %disp(k)
            d = sqrt((x(k,1) - x(j,1))^2 + ((x(k,2) - x(j,2))^2)); % distance between robots
            if (d < r) %&& ( j ~= k)
                % creates an array of robots within given radius
                x_pos(k,1) = x(k,1);
                x_pos(k,2) = x(k,2);
                %disp(x_pos)
                
            end
        end
        %disp(x_pos)
        
        l = find(x_pos(1:N) > 0);  % returns position of elements in array
        % If robots in neighborhood
%        theta_avg(j) = theta(j);
        if(~isempty(l))
            theta_average = mean(theta(l));
%             for z = 1:length(l)
%                 average(z) = theta(l(z)); % stores theta value of coressponding robots
%             end
%             theta_average = (sum(average)./length(l));

            % Find new value of theta for robot j
            theta_avg(j) = theta_average;
            theta(j) = (theta_average + theta_noise); % theta value for all robots within radius r
            % Use theta value to obtain velocity components
            %vel(j,:) = [v.*cos(theta(j)), v.*sin(theta(j))];
            vel = [v.*cos(theta(l)), v.*sin(theta(l))];
            x(l) = x_prev(l) + vel(l).*del_t;
            %x = x_prev + vel.*del_t;
%             if (x < 0)
%                 x = x + L;
%             else
%                 if (x > L)
%                     x = x - L;
%                 end
%             end
            %x_prev = x;
        end
        %disp(x)
        %figure(1)
        %x = x_prev + vel.*del_t;
        %plot(x(:,1),x(:,2),'bo');
        
        % Periodic boundary condition
         for m=1:N
             for n=1:2
                 if (x(m,n) < 0)
                     x(m,n) = x(m,n) + L;
                 else
                     if (x(m,n) > L)
                         x(m,n) = x(m,n) - L;                        
                     end
                 end
             end
         end
%             if (x >=  L)
%                x = x - L;
%             end
%         end
        %x = x_prev + vel.*del_t;
        figure(1);
        quiver(x(:,1), x(:,2), vel(:,1), vel(:,2), 0.3);
        xlim([0, L])
        ylim([0, L])
        pause(0.1)
        x_prev = x;
        theta_prev = theta;
        %end
       
%         figure(5);
%         plot(x_pos(:,1),theta_avg,'.')
%         ylim([-2*pi, 2*pi])
%         
    end
    pause(0.01)
    t = t + del_t;
    disp(t)
%     disp(abs_vel)
%     %    for i = 1:N
%     abs_vel = (1/N.*v) *sqrt((sum(vel(:,1)))^2 + (sum(vel(:,2)))^2);
%     figure(20)
%     plot(t,abs_vel,'bo')
%         %hold on
%   end
    
end

% for i = 1:N
% abs_vel = (1/N.*v) *sqrt((sum(vel(:,1)))^2 + (sum(vel(:,2)))^2);
% plot(p,abs_vel,'bo')
% hold on
% end


 


