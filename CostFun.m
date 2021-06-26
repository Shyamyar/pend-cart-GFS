function cost = CostFun(vec,fis1)  % This function computes the cost value for each individual.

% vec refers to the individual vector from GA's population defined by GA.
% This function is evaluated for each individual during every generation.

warning('off')
theta_des = 0;  % Desired angle: Inverted position means theta = 0 rad.

t0=0;   % Start time of simulation
dt = 0.01;    % Step size
tmax=10;   % End time

X0=[pi/3;0;0;0];   % Initial condition for the four states

vec(:,28:36) = round(vec(:,28:36));

%%% Assign GA individual to the corresponding fis paramters
fis1.input(1).mf(1).params = sort(vec(1:3));
fis1.input(1).mf(2).params = sort(vec(4:6));
fis1.input(1).mf(3).params = sort(vec(7:9));

fis1.input(2).mf(1).params = sort(vec(10:12));
fis1.input(2).mf(2).params = sort(vec(13:15));
fis1.input(2).mf(3).params = sort(vec(16:18));

fis1.output(1).mf(1).params = sort(vec(19:21));
fis1.output(1).mf(2).params = sort(vec(22:24));
fis1.output(1).mf(3).params = sort(vec(25:27));

fis1.rule(1).consequent = vec(28);
fis1.rule(2).consequent = vec(29);
fis1.rule(3).consequent = vec(30);
fis1.rule(4).consequent = vec(31);
fis1.rule(5).consequent = vec(32);
fis1.rule(6).consequent = vec(33);
fis1.rule(7).consequent = vec(34);
fis1.rule(8).consequent = vec(35);
fis1.rule(9).consequent = vec(36);

options = odeset('RelTol',1e-6);

Yhist = X0';
Thist = t0;
tf = 0.5;
while tf<=(tmax+0.0001)
    F = evalfis([wrapToPi(Yhist(end,1)-theta_des),Yhist(end,2)],fis1);
    [Tout,Yout]=ode45(@(t,y) InvPendulumSSModel(t,y,F, theta_des),t0:dt:tf,X0);
    Yhist = [Yhist;Yout(1:end-1,:)];
    Thist = [Thist;Tout(1:end-1)];
    
    t0 = tf;
    tf = tf+0.5;
    X0 = Yout(end,:);
end
SI = stepinfo(Yhist(:,1),Thist);
% cost = SI.SettlingTime + 10*abs(Yhist(end,1)-theta_des);  % Cost is a combination of settling time and steady state error, since we need to minimize both. The objective is get the pendulum to the inverted position quickly.
cost = abs(Yhist(end,1)-theta_des);