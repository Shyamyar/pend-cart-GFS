function ydot=InvPendulumSSModel(t,y,F,theta_des)

% This defines the dynamic model for the inverted pendulum-cart based on
% the EOMs. If you are applying this approach for a different application,
% this file needs to be modified.

M = 0.5;
m = 0.2;
l = 0.5;
%Resolving the components of the state-vector(y). 
% y = [theta; theta_dot; position of cart; velocity of cart]
y1=y(1);
y2=y(2);
y3=y(3);
y4=y(4);


%Fuzzy logic controller:
% u = 0; %dummy - used just to see that the open-loop model is correct
% fis1 = readfis('InvPendCartClass.fis');

%State-space model equations:
y1dot = y2;
y3dot = y4;
y2dot = (-F*cos(y1) + m*l*(y2^2)*sin(y1)*cos(y1) + (M+m)*9.8*sin(y1))/(m*l*(cos(y1)^2)-(4*l/3)*(M+m));
y4dot = ((4*l/3)*y2dot - 9.8*sin(y1))/cos(y1);

ydot=[y1dot;y2dot;y3dot;y4dot];