function F_Impulse = ImpulseForce(t,a,F,dt)

% Function for representing any arbrtitrary force as impulse force
% t: simulation time vector
% dt: time step size
% a: time instant at which the impulse is experienced
% F: the magnitude of the impulse felt
% The function is unit impulse function where the duration of the impulse
% force is the inverse of the impulse force magnitude

% The element in time vector corresponding to start of impulse (a)
for i = 1:length(t)
    if t(i) == a;
        inst = i;
        break;
    end
end 

% Duration of the distrubance
dur = 1/F;

% Number of time steps in simuation time vector
dur_num = ceil(dur/dt);

for ii = 1:length(t)    
    F_Impulse(:,ii) = zeros(2,1);
    if ii >= inst && ii <= inst + dur_num
        F_Impulse(1,ii) = F;
    end
end

end