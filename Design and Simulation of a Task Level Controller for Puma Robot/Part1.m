%% Part 1 - Simulate Robot Dynamics Model
clear all; %#ok<CLALL>
clc;
%% Initialization
% Since neither the joint values nor the Initial position have not been
% given for this part, we can use our own values for q and qdot 
q = [0;0;0]; 
qdot = [0;0;0]; 
% The initial and final time values along with the time step can also be selected by us
tinitial = 0;    
tstep = 0.01;
tfinal = 10;
% The Torque values being tested are as shown below
%Torque = [1;0;0]; 
Torque = [0;1;0];
%Torque = [0;0;1];
%% Robot Dynamics Model is solved using the ode45 function and the q and qdot values are returned
% The robot dynamics function which is one of the parameters in the ode45 function is used to obtain the qddot value 
[t,y]=ode45(@(t,y) RobotDynamicsModel(y,Torque),[tinitial tstep],[q,qdot]);
%% Plotting Joint Angle Values
% The following values can be plotted if required.
% figure(1)
% plot(t,y(:,1),'r',t,y(:,2),'g',t,y(:,3),'b');
% legend('q1','q2','q3');
% title('Joint Angle Values');
% xlabel('Time /s');
% ylabel('Angle / rad');
% %% Plotting Joint Velocities
% figure(2)
% plot(t,y(:,4),'r',t,y(:,5),'g',t,y(:,6),'b');
% legend('q1dot','q2dot','q3dot');
% title('Joint Velocities');
% xlabel('Time /s');
% ylabel('Angular velocity / rad/s');
