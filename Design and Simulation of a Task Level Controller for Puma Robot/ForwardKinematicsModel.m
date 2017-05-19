%% Forward Kinematics Model
function [Y,Ydot,x] = ForwardKinematicsModel(q,qdot,num)
%% Values from the DH Table
a = [0 0.4318 -0.0203 0]; 
d = [0 0.1491 0 0.4331];  
x = 0:num-1;

%% Assigning the joint angle values
q1 = q(1);
q2 = q(2);
q3 = q(3);

% Position Values 
h1 = a(3)*cos(q1)*cos(q2+q3) + d(4)*cos(q1)*sin(q2+q3) + a(2)*cos(q1)*cos(q2) - d(2)*sin(q1);
h2 = a(3)*sin(q1)*cos(q2+q3) + d(4)*sin(q1)*sin(q2+q3) + a(2)*sin(q1)*cos(q2) + d(2)*cos(q1);
h3 = - a(3)*sin(q2+q3) + d(4)*cos(q2+q3) - a(2)*sin(q2);
Y = [h1; h2; h3];
%Jacobian Matrix values are converted to double precision values 
J = double([(127*cos(q2 + q3)*sin(q1))/6250-(2159*cos(q2)*sin(q1))/5000 - (14909*cos(q1))/100000 - (43307*sin(q2 + q3)*sin(q1))/100000, (43307*cos(q2 + q3)*cos(q1))/100000 - (2159*cos(q1)*sin(q2))/5000 + (127*sin(q2 + q3)*cos(q1))/6250, (43307*cos(q2 + q3)*cos(q1))/100000 + (127*sin(q2 + q3)*cos(q1))/6250 ;(2159*cos(q1)*cos(q2))/5000 - (14909*sin(q1))/100000 - (127*cos(q2 + q3)*cos(q1))/6250 + (43307*sin(q2 + q3)*cos(q1))/100000, (43307*cos(q2 + q3)*sin(q1))/100000 - (2159*sin(q1)*sin(q2))/5000 + (127*sin(q2 + q3)*sin(q1))/6250, (43307*cos(q2 + q3)*sin(q1))/100000 + (127*sin(q2 + q3)*sin(q1))/6250; 0,(127*cos(q2 + q3))/6250 - (43307*sin(q2 + q3))/100000 - (2159*cos(q2))/5000,(127*cos(q2 + q3))/6250 - (43307*sin(q2 + q3))/100000]);
 
Ydot = J*qdot;
end
