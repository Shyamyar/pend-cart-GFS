clear all

%% Initialization
%Time parameters:
dt=1e-2;
t0=0;
% tf=10;

X0=[pi/3;0;0;0];
theta_des = 0;

%% Run simulation:
% options = odeset('RelTol',1e-6);

% [Tout,Yout]=ode45(@InvPendulumSSModel,t0:dt:tf,X0);
tf = 0.5;
Thist = t0;
Yhist = X0';
chosen_fis = readfis('TrainedInvPendCart2.fis');
while tf<9.9
    F = evalfis([wrapToPi(Yhist(end,1)-theta_des),Yhist(end,2)],chosen_fis);
    [Tout,Yout]=ode45(@(t,y) InvPendulumSSModel(t,y,F, theta_des),t0:dt:tf,X0);
    Yhist = [Yhist;Yout(1:end-1,:)];
    Thist = [Thist;Tout(1:end-1)];
    
    t0 = tf;
    tf = tf+0.5;
    X0 = Yout(end,:);
end

Tout = Thist;
Yout = Yhist;

%% Animation of the inverted pendulum
cur_pos = 0;
dTout = [0;diff(Tout)];
lc = 1;
LR = 1;

figure()
axis([-5 5 -5 5])
for i = 1:10:size(Tout,1)
    h1 = rectangle('Position',[cur_pos 0 lc 0.2]);
    hold on
    xy = [cur_pos+0.5*lc 0.2; cur_pos+0.5*lc-LR*sin(Yout(i,1)) 0.2+LR*cos(Yout(i,1))];
    h2 = plot(xy(:,1),xy(:,2),'b-','LineWidth',2);
    xlim([cur_pos+(lc/2)-5 cur_pos+(lc/2)+5])
    pause(0.2)
    delete(h1)
    delete(h2)
    cur_pos = Yout(i,3)-(lc/2);
end

%% Plot results
figure()
subplot(2,1,1),plot(Tout,Yout(:,1),Tout,Yout(:,2))
legend('\theta','\thetadot','Location','best')
xlabel('Time[sec]')
ylabel('\theta and \thetadot')
grid
subplot(2,1,2),plot(Tout,Yout(:,4));legend('Velocity of the cart')
xlabel('Time[sec]')
ylabel('u[m/s]')
grid

