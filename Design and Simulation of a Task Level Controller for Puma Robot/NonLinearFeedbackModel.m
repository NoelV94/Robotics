%% Non Linear Feedback
%Torque = D(q)*J^(-1)*u - D(q)*J^(-1)J'q' + C(q,q')q' + g(q)

function Torque = NonLinearFeedbackModel(u,q,qdot)
% Assigning q and qdot values
q1 = q(1);
q2 = q(2);
q3 = q(3);
q1dot = qdot(1);
q2dot = qdot(2);
q3dot = qdot(3);

%Jacobians 
J11 = 0.02032*cos(q2 + q3)*sin(q1) - 0.4318*cos(q2)*sin(q1) - 0.14909*cos(q1) - 0.43307*sin(q2 + q3)*sin(q1);
J12 = 0.43307*cos(q2 + q3)*cos(q1) - 0.4318*cos(q1)*sin(q2) + 0.02032*sin(q2 + q3)*cos(q1);
J13 = 0.43307*cos(q2 + q3)*cos(q1) + 0.02032*sin(q2 + q3)*cos(q1);
J21 = 0.4318*cos(q1)*cos(q2) - 0.14909*sin(q1) - 0.02032*cos(q2 + q3)*cos(q1) + 0.43307*sin(q2 + q3)*cos(q1);
J22 = 0.43307*cos(q2 + q3)*sin(q1) - 0.4318*sin(q1)*sin(q2) + 0.02032*sin(q2 + q3)*sin(q1);
J23 = 0.43307*cos(q2 + q3)*sin(q1) + 0.02032*sin(q2 + q3)*sin(q1);
J31 = 0;
J32 = 0.02032*cos(q2 + q3) - 0.43307*sin(q2 + q3) - 0.4318*cos(q2);
J33 = 0.02032*cos(q2 + q3) - 0.43307*sin(q2 + q3);
J = [J11 J12 J13; J21 J22 J23; J31 J32 J33];

%Jdot
Jdot11 = 0.02032*cos(q2 + q3)*cos(q1)*q1dot - 0.02032*sin(q2 + q3)*sin(q1)*(q2dot+q3dot) + 0.4318*sin(q2)*sin(q1)*q2dot - 0.4318*cos(q2)*cos(q1)*q1dot + 0.14909*sin(q1)*q1dot - 0.43307*cos(q2 + q3)*sin(q1)*(q2dot+q3dot) - 0.43307*sin(q2 + q3)*cos(q1)*q1dot;
Jdot12 = -0.43307*sin(q2 + q3)*cos(q1)*(q2dot + q3dot) - 0.43307*cos(q2 + q3)*sin(q1)*q1dot + 0.4318*sin(q1)*sin(q2)*q1dot - 0.4318*cos(q1)*cos(q2)*q2dot + 0.02032*cos(q2 + q3)*cos(q1)*(q2dot + q3dot) - 0.02032*sin(q2 + q3)*sin(q1)*q1dot;
Jdot13 = -0.43307*sin(q2 + q3)*cos(q1)*(q2dot + q3dot) - 0.43307*cos(q2 + q3)*sin(q1)*q1dot  + 0.02032*cos(q2 + q3)*cos(q1)*(q2dot + q3dot) - 0.02032*sin(q2 + q3)*sin(q1)*q1dot;
Jdot21 = -0.4318*sin(q1)*cos(q2)*q1dot - 0.4318*cos(q1)*sin(q2)*q2dot - 0.14909*cos(q1)*q1dot + 0.02032*sin(q2 + q3)*cos(q1)*(q2dot + q3dot) + 0.02032*cos(q2 + q3)*sin(q1)*q1dot  + 0.43307*cos(q2 + q3)*cos(q1)*(q2dot + q3dot)  - 0.43307*sin(q2 + q3)*sin(q1)*q1dot;
Jdot22 = -0.43307*sin(q2 + q3)*sin(q1)*(q2dot + q3dot) + 0.43307*cos(q2 + q3)*cos(q1)*q1dot  - 0.4318*cos(q1)*sin(q2)*q1dot  - 0.4318*sin(q1)*cos(q2)*q2dot + 0.02032*cos(q2 + q3)*sin(q1)*(q2dot + q3dot) + 0.02032*sin(q2 + q3)*cos(q1)*q1dot;
Jdot23 = -0.43307*sin(q2 + q3)*sin(q1)*(q2dot + q3dot) + 0.43307*cos(q2 + q3)*cos(q1)*q1dot + 0.02032*cos(q2 + q3)*sin(q1)*(q2dot + q3dot) + 0.02032*sin(q2 + q3)*cos(q1)*q1dot;
Jdot31 = 0;
Jdot32 =  -0.02032*sin(q2 + q3)*(q2dot + q3dot) - 0.43307*cos(q2 + q3)*(q2dot + q3dot) + 0.4318*sin(q2)*q2dot;
Jdot33 = -0.02032*sin(q2 + q3)*(q2dot + q3dot) - 0.43307*cos(q2 + q3)*(q2dot + q3dot);

Jdot = [Jdot11 Jdot12 Jdot13; Jdot21 Jdot22 Jdot23; Jdot31 Jdot32 Jdot33];


%D(q) matrix
d11 = 2.4574 + 1.7181*cos(q2)*cos(q2) + 0.4430*sin(q2+q3)*sin(q2+q3) - 0.0324*cos(q2)*cos(q2+q3) - 0.0415*cos(q2+q3)*sin(q2+q3) + 0.9378*cos(q2)*sin(q2+q3);
d12 = 2.2312*sin(q2) - 0.0068*sin(q2+q3) - 0.1634*cos(q2+q3);
d13 = -0.0068*sin(q2+q3) - 0.1634*cos(q2+q3);
d21 = d12;
d22 = 5.1285 + 0.9378*sin(q3) - 0.0324*cos(q3);
d23 = 0.4424 + 0.4689*sin(q3) - 0.0162*cos(q3);
d31 = d13;
d32 = d23;
d33 = 1.0236;
D = [d11 d12 d13; d21 d22 d23; d31 d32 d33];

%C matrix
c111 = 0;
c121 = 0.0207 - 1.2752*cos(q2)*sin(q2) + 0.4429*cos(q3)*sin(q3) - 0.8859*sin(q2)*sin(q3)*sin(q2+q3) + 0.0325*cos(q2)*sin(q2+q3) + 0.4689*cos(q2)*cos(q2+q3) - 0.4689*sin(q2)*sin(q2+q3) - 0.0461*cos(q2+q2) - 0.0415*cos(q2+q3)*cos(q2+q3) - 0.0163*sin(q3);
c131 = 0.0207 + 0.4429*cos(q2)*sin(q2) + 0.4429*cos(q3)*sin(q3) - 0.8859*sin(q2)*sin(q3)*sin(q2+q3) + 0.0163*cos(q2)*sin(q2+q3) + 0.4689*cos(q2)*cos(q2+q3) - 0.0415*cos(q2+q3)*cos(q2+q3);
c211 = c121;
c221 = 1.8181*cos(q2) + 0.1634*sin(q2+q3) - 0.0068*cos(q2+q3);
c231 = 0.1634*sin(q2+q3) - 0.0068*cos(q2+q3);
c311 = c131;
c321 = c231;
c331 = 0.1634*sin(q2+q3) - 0.0068*cos(q2+q3);
c112 = - c121;
c122 = 0;
c132 = 0;
c212 = c122;
c222 = 0;
c232 = 0.4689*cos(q3) + 0.0162*sin(q3);
c312 = 0;
c322 = c232;
c332 = 0.4689*cos(q3) + 0.0162*sin(q3);
c113 = - c131;
c123 = - c132;
c133 = 0;
c213 = c123;
c223 = - c232;
c233 = 0;
c313 = c133;
c323 = c233;
c333 = 0;

c11 = c111*q1dot + c211*q2dot + c311*q3dot;
c12 = c121*q1dot + c221*q2dot + c321*q3dot;
c13 = c131*q1dot + c231*q2dot + c331*q3dot;
c21 = c112*q1dot + c212*q2dot + c312*q3dot;
c22 = c122*q1dot + c222*q2dot + c322*q3dot;
c23 = c132*q1dot + c232*q2dot + c332*q3dot;
c31 = c113*q1dot + c213*q2dot + c313*q3dot;
c32 = c123*q1dot + c223*q2dot + c323*q3dot;
c33 = c133*q1dot + c233*q2dot + c333*q3dot;

C = [c11 c12 c13; c21 c22 c23; c31 c32 c33];

%g matrix
g1 = 0;
g2 = - 48.5564*cos(q2) + 1.0462*sin(q2) + 0.3683*cos(q2+q3) - 10.6528*sin(q2+q3);
g3 = 0.3683*cos(q2+q3) - 10.6528*sin(q2+q3);
g = [g1 ; g2 ; g3];

Torque = D*(inv(J))*u - D*(inv(J))*Jdot*qdot + C*qdot + g;
end