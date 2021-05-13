function [de_dt] = ErrorDynamics_SSR(t,e,Kp,Kd,disturbance)

%% Description
% Function for state space modelling of the Error Dynamics of a trajectory
% following control system

% e: state error vector (desired state vector - current state vector)
% de_dt: first differential of state error vector wrt time
% A_e: State Error Weighing Matrix
% B_e: Control Cost Matrix
% f: External Disturbance force on the control system
% de_dt = A_e.e + B_e.f --> STATE SPACE MODEL OF ERROR DYNAMICS

% State Error Weighing Matrix
A_e = [zeros(size(Kp)),eye(size(Kp));
        -Kp , -Kd];
    
% Control Cost Matrix
B_e = [zeros(size(Kp));eye(size(Kp))];

% Disturbance Force

switch disturbance
    case 'None'
        f = [0;0];
        
    case 'Impulse'
        if t<5
            f = [0;0]; 
        elseif t>=5 && t<= 5 + (1/5)
            f = [5;0];
        else
            f = [0;0];
        end
        
    case 'Harmonic'
        f = [5*sin(10*t);0];
        
    case 'Static'
        f = [5;0];
        
end
        
% Error Dynamics State Space Equation
de_dt = A_e*e + B_e*f;
end