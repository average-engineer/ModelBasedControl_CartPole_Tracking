function [dw_dt] = ClosedLoopDynamics_NL(t,w,m,M,g,L,Kp,Kd,disturbance,desired)

% Creates the state space model of the non-linear dynamic equation of
% inverted Cart Pole System with only one actuation
% This dynamic formulation is made for the tracking problem where the cart
% and the pole have to follow a desired trajectory

% State vector: [w1;w2;w3;w4]
% w1: Cart Position (meters)
% w2: Pole angle (radians)
% w3: Cart Velocity (m/s)
% w4: Pole angular velocity (rad/s)


% Desired trajectory
switch desired
    case 'Case1'
        Xd = [0;sin(t)];
        Xd_dot = [0;cos(t)];
        Xd_ddot = [0;-sin(t)];
        
    case 'Case2'
        Xd = [sin(t);0];
        Xd_dot = [cos(t);0];
        Xd_ddot = [-sin(t);0];
end


% Disturbance Force
switch disturbance
    case 'None'
        f = [0;0];
        
    case 'Impulse'
        if t<5
            f = [0;0]; 
        elseif t>=5 && t<= 5 + (1/5)
            f = [200;0];
        else
            f = [0;0];
        end
        
    case 'Harmonic'
        f = [5*sin(10*t);0];
        
    case 'Static'
        f = [5;0];       
end

% Non-Linear dynamic coefficient matrices
% Mass Matrix
M_mat = [M + m,(m*L*cos(w(2)))/2;
        (m*L*cos(w(2)))/2,(m*(L)^2)/3];

% Damping matrix
C_mat = [0,(-m*L*sin(w(2))*w(4))/2;
        (-m*L*sin(w(2))*w(4))/2,0];
    
% Stiffness matrix
K_mat = [0;(-m*g*L*sin(w(2)))/2];

% Modified coefficient matrices for single actuation
M1 = [1,0;0,0]*M_mat;
K1 = [0,0;0,1]*K_mat;
C1 = [0,0;0,1]*C_mat + [1,0;0,0]*M_mat*Kd;

% Partitioning state vectors into general coordinates
X = [w(1);w(2)];
X_dot = [w(3);w(4)];

dwdt_12 = X_dot;

dwdt_34 = (M_mat\eye(size(M_mat)))*(M1*(Xd_ddot + Kd*Xd_dot + Kp*Xd - Kp*X) - C1*X_dot - K1 - [1,0;0,0]*f);

dw_dt = [dwdt_12;dwdt_34];
    
end