%% Inverse Kinematics for q(0) and qdot(0) and the variance for part 4

function [R] = IkineAndVar(per)
%% Finding the Inverse Kinematics
function F = kinsolve1(q)
    a = [0 0.4318 -0.0203 0]; 
    d = [0 0.1491 0 0.4331]; 
    Y = [-0.7765 ; 0 ; 0.045];
    F(1) = a(3)*cos(q(1))*cos(q(2)+q(3)) + d(4)*cos(q(1))*sin(q(2)+q(3)) + a(2)*cos(q(1))*cos(q(2)) - d(2)*sin(q(1)) - Y(1);
    F(2) = a(3)*sin(q(1))*cos(q(2)+q(3)) + d(4)*sin(q(1))*sin(q(2)+q(3)) + a(2)*sin(q(1))*cos(q(2)) + d(2)*cos(q(1)) - Y(2);
    F(3) = - a(3)*sin(q(2)+q(3)) + d(4)*cos(q(2)+q(3)) - a(2)*sin(q(2)) - Y(3);
end
Kinout1 = @kinsolve1;
qa = [0,0,0];
qinitial1 = fsolve(Kinout1,qa);
function F = kinsolve2(q)
    a = [0 431.8/1000 -20.32/1000 0]; 
    d = [0 149.09/1000 0 433.07/1000]; 
    Y = [-0.5 ; -0.1 ; 0];
    F(1) = a(3)*cos(q(1))*cos(q(2)+q(3)) + d(4)*cos(q(1))*sin(q(2)+q(3)) + a(2)*cos(q(1))*cos(q(2)) - d(2)*sin(q(1)) - Y(1);
    F(2) = a(3)*sin(q(1))*cos(q(2)+q(3)) + d(4)*sin(q(1))*sin(q(2)+q(3)) + a(2)*sin(q(1))*cos(q(2)) + d(2)*cos(q(1)) - Y(2);
    F(3) = - a(3)*sin(q(2)+q(3)) + d(4)*cos(q(2)+q(3)) - a(2)*sin(q(2)) - Y(3);
end
Kinout2 = @kinsolve2;
qb = [pi/4,-pi/4,pi/4];
qinitial2 = fsolve(Kinout2,qb);
%% The Variance factor depending on the percentage variance
R = rand(3,3);
for i = 1:3
    for j = 1:3
        if R(i,j) >= 0.5
            R(i,j) = (1 + (per/100));
        else
            R(i,j) = (1 - (per/100));
        end
    end
end
end

