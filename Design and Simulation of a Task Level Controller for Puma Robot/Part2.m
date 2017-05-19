%% Part 2 - Implementing the Non Linear Controller
clear all; %#ok<CLALL>
clc;
%% With u value [1;0;0]
%% Initialization
% The value for u will be selected using the values provided
u = [1;0;0];
% The Joint value and joint velocity value have been assumed for this implementation
q = [0;0;0]; 
qdot = [0;0;0]; 
% The initial and final time along with the time step have also been randomly selected 
tinitial = 0;
tstep = 0.01;
tfinal = 20;
num =15;
% The matrices below have been initialized for the X,Y and Z position values
Xval = zeros(num,1);
Yval = zeros(num,1);
Zval = zeros(num,1);
Y = [Xval;Yval;Zval];
%% Position Values
for i=1:num
    % The torque will be calculated using the Non Linear feedback model
    Torque = NonLinearFeedbackModel(u,q,qdot);
    % The ode45 function will return the q and qdot values required to perform forward kinematics
    [t,y]=ode45(@(t,y) RobotDynamicsModel(y,Torque),[tinitial tstep],[q,qdot]);
    q = y(length(y),1:3)';
    qdot = y(length(y),4:6)';
    % The forward kinematics model is used to obtain the current position and velocity values 
    [Y,Ydot,x] = ForwardKinematicsModel(q,qdot,num);
    Xval(i) = Y(1);
    Yval(i) = Y(2);
    Zval(i) = Y(3);
end
%% Plotting the X, Y and Z position values
figure(1)
subplot(2,2,1)
plot(x,Xval,'r',x,Yval,'g',x,Zval,'b');
legend('X','Y','Z');
title('Position along X, Y and Z axes for u = [1;0;0]');
xlabel('Time /s');
ylabel('Position /m');



%% With u value [0;1;0]
%% Initialization
% The value for u will be selected using the values provided
u = [0;1;0];
% The Joint value and joint velocity value have been assumed for this implementation
q = [0;0;0]; 
qdot = [0;0;0]; 
% The initial and final time along with the time step have also been randomly selected 
tinitial = 0;
tstep = 0.01;
tfinal = 20;
num =15;
% The matrices below have been initialized for the X,Y and Z position values
Xval = zeros(num,1);
Yval = zeros(num,1);
Zval = zeros(num,1);
Y = [Xval;Yval;Zval];
%% Position Values
for i=1:num
    % The torque will be calculated using the Non Linear feedback model
    Torque = NonLinearFeedbackModel(u,q,qdot);
    % The ode45 function will return the q and qdot values required to perform forward kinematics
    [t,y]=ode45(@(t,y) RobotDynamicsModel(y,Torque),[tinitial tstep],[q,qdot]);
    q = y(length(y),1:3)';
    qdot = y(length(y),4:6)';
    % The forward kinematics model is used to obtain the current position and velocity values 
    [Y,Ydot,x] = ForwardKinematicsModel(q,qdot,num);
    Xval(i) = Y(1);
    Yval(i) = Y(2);
    Zval(i) = Y(3);
end
%% Plotting the X, Y and Z position values
subplot(2,2,2)
plot(x,Xval,'r',x,Yval,'g',x,Zval,'b');
legend('X','Y','Z');
title('Position along X, Y and Z axes for u = [0;1;0]');
xlabel('Time /s');
ylabel('Position /m');



%% With u value [0;0;1]
%% Initialization
% The value for u will be selected using the values provided
u = [0;0;1];
% The Joint value and joint velocity value have been assumed for this implementation
q = [0;0;0]; 
qdot = [0;0;0]; 
% The initial and final time along with the time step have also been randomly selected 
tinitial = 0;
tstep = 0.01;
tfinal = 20;
num =15;
% The matrices below have been initialized for the X,Y and Z position values
Xval = zeros(num,1);
Yval = zeros(num,1);
Zval = zeros(num,1);
Y = [Xval;Yval;Zval];
%% Position Values
for i=1:num
    % The torque will be calculated using the Non Linear feedback model
    Torque = NonLinearFeedbackModel(u,q,qdot);
    % The ode45 function will return the q and qdot values required to perform forward kinematics
    [t,y]=ode45(@(t,y) RobotDynamicsModel(y,Torque),[tinitial tstep],[q,qdot]);
    q = y(length(y),1:3)';
    qdot = y(length(y),4:6)';
    % The forward kinematics model is used to obtain the current position and velocity values 
    [Y,Ydot,x] = ForwardKinematicsModel(q,qdot,num);
    Xval(i) = Y(1);
    Yval(i) = Y(2);
    Zval(i) = Y(3);
end
%% Plotting the X, Y and Z position values
subplot(2,2,3)
plot(x,Xval,'r',x,Yval,'g',x,Zval,'b');
legend('X','Y','Z');
title('Position along X, Y and Z axes for u = [0;0;1]');
xlabel('Time /s');
ylabel('Position /m');