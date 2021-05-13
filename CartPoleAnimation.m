function CartPoleAnimation(w,L)

% Function for animating the response of the Cart Pole System
% xc and yc are considered the centre of the cart box
% xb and yb are considered the free end coordinates of the Pole
% yc is considered zero
% xc is the same as the horizontal position of the cart
% w is the calculated state vector of the system
% L is the pole length

yc = 0;

figure
for i = 1:max(size(w))
    xc = w(i,1); % Horizontal Position of Cart
    xb = xc + L*sin(w(i,2)); % assuming the angle is positive when the pole is towards the right side (positive X axis)
    yb = yc + L*cos(w(i,2)); % Inverted Pole
    
    
    hold on
    % Drawing the cart box
    h1 = drawCart(xc,yc,0.4,2,'r');
    % Drawing the pole
    h2 = plot([xc,xb],[yc,yb],'r','Linewidth',2);
    
    % Pausing for some time before starting the next instant (frame rate)
    pause(0.01)
    
    % deleting each plot/drawing for this instant
    delete(h1)
    delete(h2)
    
    axis([-5,5,-5,5])
    grid on
end
end