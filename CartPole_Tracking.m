clearvars
close all
clc
format bank

%% Description
% Model Based Control System design for trajectory following of a Cart with
% an Inverted Pole.

% Both kinds of actuation are considered

% 1). Both cart and pole are actuated - actBoth
% 2). Only Cart is actuated - actCart

% Since it is a tracking problem, thus linearization of the system can't be
% assumed using small angle approximation. Model based control helps in
% converting the non-linear dynamic equations to linearized Error Dynamics
% equations.

%% System Parameters
% Cart mass
M = 10;%kg

% Pole mass
m = 2;%kg

% Pole length
L = 4;%meters

% Acceleration due to gravity
g = 9.81; %m/s^2

% Time Step Size
dt = 0.01;

% Time Vector
t_span = [0:dt:100];

% Initial Conditions
w_0 = [0;0.5;0;0];

%% Disturbance force on the control system
% variable for deciding the type of disturbance force
dist = 'Static'; % None/Impulse/Harmonic/Static

switch dist
    case 'None'
        % No external disturbance force on the system
        f_dist = zeros(2,1,length(t_span));
        
    case 'Impulse'
        % The disturbance force is modelled as an unit impulse function force
        % applied horizontally on the cart
        % Resembles a hand flicking the cart
        % There is no disturbance torque given to the pole joint
        
        % Time instant at which force is applied (seconds)
        a = 5;
        
        % Magnitude of disturbance force
        F = 200;
        f_dist = ImpulseForce(t_span,a,F,dt);
        
    case 'Harmonic'
        % Harmonic Disturbance Force
        for i = 1:length(t_span)
            f_dist(:,i) = zeros(2,1);
            f_dist(1,i) = 5*sin(10*t_span(i));
        end
        
    case 'Static'
        for i = 1:length(t_span)
            f_dist(:,i) = zeros(2,1);
            f_dist(1,i) = 5;
        end
end

%% Desired trajectory
% This is the trajectory which has to be tracked by the control system
% Harmonic trjaectory desired for the pole joint angle and the cart should
% come to rest at its initial starting position

Case = 'Case1';

switch Case
    case 'Case1'
        % Pole joint angle desired trajectory
        posn_PoleDesired = sin(t_span);
        vel_PoleDesired = cos(t_span);
        
        % Cart desired trajectory
        posn_CartDesired = zeros(size(t_span));
        vel_CartDesired = zeros(size(t_span));
        
        % Desired General coordinate Accelerations
        acc_PoleDesired = -sin(t_span);
        acc_CartDesired = zeros(size(t_span));
        
        
    case 'Case2'
        % Pole joint angle desired trajectory
        posn_CartDesired = sin(t_span);
        vel_CartDesired = cos(t_span);
        
        % Cart desired trajectory
        posn_PoleDesired = zeros(size(t_span));
        vel_PoleDesired = zeros(size(t_span));
        
        % Desired General coordinate Accelerations
        acc_CartDesired = -sin(t_span);
        acc_PoleDesired = zeros(size(t_span));
        
end

for i = 1:length(t_span)
    desired_GenCoorAcc(:,i) = [acc_CartDesired(i);acc_PoleDesired(i)];
end

%% Variable for deciding type of actuation
% actBoth: both cart and pole are actuated
% actCart: only cart is actuated
act = 'actCart';

switch act
    
    %% Both cart and pole are actuated
    case 'actBoth'
        %% Controller Gains
        % PD Controller is made use of
        
        % Proportional Gain
        Kp = [2,0;0,2];
        
        % Derivative Gain
        Kd = [2,0;0,2];
        
        %% Solving the error dynamics equation
        % State Vector: [Cart Posn Error;Pole Angle Error;Cart Velocity Error;Pole
        % Angular Velocity Error]
        
        % Desired state vector at t = 0
        wd_0 = [posn_CartDesired(1);posn_PoleDesired(1);vel_CartDesired(1);vel_PoleDesired(1)];
        % Error Initial Condition
        e_0 = wd_0 - w_0;
        
        [t,e] = ode45(@(t,e)ErrorDynamics_SSR(t,e,Kp,Kd,m,M,L,dist,Case),t_span,e_0);
        e = e';
        
        % State Error Weighing Matrix
        A_error = [zeros(size(Kp)),eye(size(Kp));
            -Kp , -Kd];
        
        % Closed Loop Eigen-Values (Error Stability)
        ClosedLoop_Poles = eig(A_error)
        
        
        % State Errors
        posn_CartError = e(1,:);
        posn_PoleError = e(2,:);
        vel_CartError = e(3,:);
        vel_PoleError = e(4,:);
        
        %% Computing the current positions and velocities
        % Error = Desired - Current
        % Current = Desired - Error
        
        posn_CartCurrent = posn_CartDesired - posn_CartError;
        vel_CartCurrent = vel_CartDesired - vel_CartError;
        
        posn_PoleCurrent = posn_PoleDesired - posn_PoleError;
        vel_PoleCurrent = vel_PoleDesired - vel_PoleError;
        
        %% Coefficient Matrices of non-linear EOM of the system
        % X-> Generalized Coordinates Vector
        % M(X)X_ddot + C(X,X_dot)X_dot + K(X) = Q
        
        for ii = 1:length(t_span)
            
            % Mass Matrix
            M_mat(:,:,ii) = [M + m, m*L*cos(posn_PoleCurrent(ii))/2;
                m*L*cos(posn_PoleCurrent(ii))/2, (m*(L)^2)/3];
            
            % Damping Matrix
            C_mat(:,:,ii) = [0,(-m*L*sin(posn_PoleCurrent(ii))*vel_PoleCurrent(ii))/2;
                (-m*L*sin(posn_PoleCurrent(ii))*vel_PoleCurrent(ii))/2, 0 ];
            
            % Stiffness Matrix
            K_mat(:,:,ii) = [0 ; (-m*g*L*sin(posn_PoleCurrent(ii)))/2];
            
            % Actuator Effort Matrix
            u_act(:,ii) = M_mat(:,:,ii)*(Kp*[e(1,ii);e(2,ii)] + Kd*[e(3,ii);e(4,ii)] + desired_GenCoorAcc(:,ii)) + ...
                C_mat(:,:,ii)*[vel_CartCurrent(ii);vel_PoleCurrent(ii)] + K_mat(:,:,ii) - f_dist(:,ii);
        end
        
        %% Plotting Results
        %% Response of Control System
        figure
        hold on
        plot(t_span,posn_CartCurrent,'linewidth',2)
        plot(t_span,posn_CartDesired,'-.','linewidth',2)
        plot(t_span,posn_CartError,'linewidth',2)
        legend('Current','Desired','Error')
        xlabel('Time(s)')
        title('Cart Position (meters)')
        grid on
        
        
        figure
        hold on
        plot(t_span,posn_PoleCurrent,'linewidth',2)
        plot(t_span,posn_PoleDesired,'-.','linewidth',2)
        plot(t_span,posn_PoleError,'linewidth',2)
        legend('Current','Desired','Error')
        xlabel('Time(s)')
        title('Pole Position (rad)')
        grid on
        
        %% Actuator Effort
        figure
        hold on
        plot(t_span,u_act(1,:),'Linewidth',2)
        plot(t_span,u_act(2,:),'linewidth',2)
        legend('Cart Actuation (N)','Pole Actuation (N-m)')
        grid on
        
        %% Current State Vector
        for i = 1:length(t_span)
            w(i,:) = [posn_CartCurrent(i);posn_PoleCurrent(i);vel_CartCurrent(i);vel_PoleCurrent(i)];
        end
        
    %% Only Cart is actuated 
    case 'actCart'
        %% Controller Gains
        % PD Controller is made use of
        
        % Proportional Gain
        Kp = [1,-0.1;-0.5,1];
        
        % Derivative Gain
        Kd = [1,-0.5;-0.1,1];
        
        [t,w] = ode45(@(t,w)ClosedLoopDynamics_NL(t,w,m,M,g,L,Kp,Kd,dist,Case),t_span,w_0);  
        
        %% State errors 
        % Cart Position Error
        e(:,1) = posn_CartDesired' - w(:,1);
        e(:,2) = posn_PoleDesired' - w(:,2);
        e(:,3) = vel_CartDesired' - w(:,3);
        e(:,4) = vel_PoleDesired' - w(:,4);
        
        %% Responses
        figure
        hold on
        plot(t_span,w(:,1),'linewidth',2)
        plot(t_span,e(:,1),'linewidth',2)
        plot(t_span,posn_CartDesired,'-.','color','k','linewidth',2)
        grid on
        legend('Current','Error','Desired')
        
        
        figure
        hold on
        plot(t_span,w(:,2),'linewidth',2)
        plot(t_span,e(:,2),'linewidth',2)
        plot(t_span,posn_PoleDesired,'-.','color','k','linewidth',2)
        grid on
        legend('Current','Error','Desired')
        
        %% Actuator Effort
        % Only one actuator is activated
        for ii = 1:length(t_span)
            %% Non-Linear dynamic coefficient matrices
            % Mass Matrix
            M_mat(:,:,ii) = [M + m, m*L*cos(w(ii,2))/2;
                m*L*cos(w(ii,2))/2, (m*(L)^2)/3];
            
            % Damping Matrix
            C_mat(:,:,ii) = [0,(-m*L*sin(w(ii,2))*w(ii,4))/2;
                (-m*L*sin(w(ii,2))*w(ii,4))/2, 0 ];
            
            % Stiffness Matrix
            K_mat(:,:,ii) = [0 ; (-m*g*L*sin(w(ii,2)))/2];
            
            % Actuator Effort Matrix
            u_act(:,ii) = [1,0]*(M_mat(:,:,ii)*(Kp*[e(ii,1);e(ii,2)] + Kd*[e(ii,3);e(ii,4)] + desired_GenCoorAcc(:,ii)) + ...
                C_mat(:,:,ii)*[w(ii,3);w(ii,4)] + K_mat(:,:,ii) - f_dist(:,ii));
        end
        
end
        
