%% Part 4 with Initial Position Y(0) = [-0.7765 ; 0 ; 0.045] and changing percentage variance values
clear all; %#ok<CLALL>
clc;
%% Initialization
%First we initialize the required Initial and Final time 
tinitial = 0;
tstep = 0.01;
tfinal = 40;
%niter and num values have been initialized for use in the loop below
niter=1;
num = 1;
%Kd and Kp are the values for the PD controller that have been chosen randomly 
Kd = [36;30;40];
Kp = [18;15;20];
%Initializing matrices for position and velocity errors
XPosError = zeros(1,400);
YPosError = zeros(1,400);
ZPosError = zeros(1,400);
XVelError = zeros(1,400);
YVelError = zeros(1,400);
ZVelError = zeros(1,400);
%Now we use assign the values that have been provided
omega = pi/4;
%omega = pi/2;
Yinitial = [-0.7765 ; 0 ; 0.045];
Yinitialdot = [0;0;0];
%The joint value has been obtained by performing Inverse Kinematics on the initial position values 
q = [0.1932  ; -2.5912  ;  0.6370]; 
qdot = [0;0;0]; 
[Yd, Yddot, Ydddot] = DesiredTrajectory(omega,0);
%error dynamics (Derivation in Report)
e = Yd - Yinitial;
edot = Yddot - Yinitialdot;
%u is the control input
u = Ydddot + Kd.*edot + Kp.*e ; 
%Torque will be obtained by using sending the u value to the non linear feedback model
Torque = NonLinearFeedbackModel(u,q,qdot);
loopcheck = niter*tstep;
%Percentage Variance
percent = 2;
R = IkineAndVar(percent);
%% Loop for obtaining the joint values to move from initial position to final position
while loopcheck < tfinal
    %Robot Dynamics block to obtain the q and qdot values
    [T,y]=ode45(@(t,y) RobotDynamicsModelVar(y,Torque,R),[tinitial tstep],[q,qdot]);
    q = y(length(y),1:3)';
    qdot = y(length(y),4:6)';
    %The Forward Kinematics model will return the current position and velocity
    [Y,Ydot,x] = ForwardKinematicsModel(q,qdot,num);
    [Yd,Yddot,Ydddot] = DesiredTrajectory(omega,(loopcheck));
    %The error dynamics can be obtained by using the current and desired position and velocity values
    e = Yd - Y;
    edot = Yddot - Ydot;
    %The control input will be calculated using the previously obtained
    %values along with the chosen Kp and Kd values
    u = Ydddot + Kd.*edot + Kp.*e;
    %The non linear feedback model will return the Torque value using the
    %control input
    Torque = NonLinearFeedbackModel(u,q,qdot);
    niter = niter + 1;
    loopcheck = niter*tstep;
    %The values for the position error as well as velocity error will be
    %then iteratively added
    XPosError(niter) = e(1);
    YPosError(niter) = e(2);
    ZPosError(niter) = e(3);
    XVelError(niter) = edot(1);
    YVelError(niter) = edot(2);
    ZVelError(niter) = edot(3);
end
%% Plotting the required error values over time
%Axis values need to be changed in order to obtain correct zoomed in values
t=0:tstep:(tfinal-tstep);
figure(1)
subplot(2,1,1)
plot(t,XPosError,'r',t,YPosError,'b',t,ZPosError,'g')
axis([0 tfinal -5 5])
line('XData', [0 50], 'YData', [0 0], 'LineStyle', '-', ...
    'LineWidth', 1, 'Color',[0.2 0.4 1.0])
title('Plot of Position Errors along the 3 axes (Zoomed out)')
xlabel('Time')
ylabel('Position error' )
subplot(2,1,2)
plot(t,XPosError,'r',t,YPosError,'b',t,ZPosError,'g')
axis([0 tfinal -0.05 0.05])
line('XData', [0 50], 'YData', [0 0], 'LineStyle', '-', ...
    'LineWidth', 1, 'Color',[0.2 0.4 1.0])
title('Plot of Position Errors along the 3 axes (Zoomed in)')
xlabel('Time')
ylabel('Position error' )

figure(2)
subplot(2,1,1)
plot(t,XVelError,'r',t,YVelError,'b',t,ZVelError,'g')
axis([0 tfinal -5 5])
line('XData', [0 50], 'YData', [0 0], 'LineStyle', '-', ...
    'LineWidth', 0.5, 'Color',[0.0 0.4 0.4])
title('Plot of Velocity Errors along the 3 axes (Zoomed out)')
xlabel('Time')
ylabel('Velocity error' )
subplot(2,1,2)
plot(t,XVelError,'r',t,YVelError,'b',t,ZVelError,'g')
axis([0 tfinal -0.05 0.05])
line('XData', [0 50], 'YData', [0 0], 'LineStyle', '-', ...
    'LineWidth', 0.5, 'Color',[0.0 0.4 0.4])
title('Plot of Velocity Errors along the 3 axes (Zoomed in)')
xlabel('Time')
ylabel('Velocity error' )