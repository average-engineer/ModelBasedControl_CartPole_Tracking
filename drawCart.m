function[h1] = drawCart(xc,yc,height,width,color)

% draws a 2D rectangular cart
% xc and yc are the centre points of the said cart 

% the corners of the cart box coordinates are:
% (x1,y1), (x2,y2), (x3,y3), (x4,y4)

x1 = xc - (width/2);
x4 = x1;
x2 = xc + (width/2);
x3 = x2;

y1 = yc - (height/2);
y2 = y1;
y3 = yc + (height/2);
y4 = y3;

% vector of X coordinates
x = [x1,x2,x3,x4,x1];

% vector of Y coordinates
y = [y1,y2,y3,y4,y1];

h1 = plot(x,y,color,'Linewidth',2);
end