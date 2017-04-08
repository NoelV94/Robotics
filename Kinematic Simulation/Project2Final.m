clc;
clear all;                            %#ok<CLSCR>
input1    = dlmread('arm');
input2    = dlmread('trajectory');
n         = input1(1,1);              %%Number of Joints
Dpf       = input1(1,2);              %%Damping Factor
m         = input2(1,1);              %%Number of points
[r,c]     = size(input1);
Fwd       = zeros(c,r-1);
pos       = zeros(3,n);

x         = 1;
j         = 2;
nxt       = 2;
p         = 1;


for z     = 1:1:r-1                  %%loop for joint angle and link values 
l(x)      = input1(j,1);             %%Length of link
theta(x)  = input1(j,2);             %%Theta Value
x         = x+1;
j         = j+1;
end

%%Start Main Loop

for Npts = 1:1:m

DeltaTheta = ones(1,n);

while norm(DeltaTheta) >= 0.064
     lp2       = 1;
    while 1000 >= lp2
        
%%Fwd Kin Loop    
for FWDLp = 1:1:1
y         = 1;
a         = 1;
thetat(y) = theta(1);                    
XX(y)     = l(y)*cos(thetat(y));
YY(y)     = l(y)*sin(thetat(y)); 
Fwd(:,y)  = [XX(y),YY(y)];

for FKLOOP= 1:1:n-1                        
y         = y+1;
thetat(y) = thetat(a)+ theta(1+a);
XX(y)     = XX(a) + l(y)*cos(thetat(y));
YY(y)     = YY(a) + l(y)*sin(thetat(y));
Fwd(:,y)  = [XX(y),YY(y)];
a         = a+1;
end
end

Xdes     = [input2(nxt,1);input2(nxt,2);0];  %%X Desired
Xact     = [Fwd(1,r-1) ;Fwd(2,r-1) ;0];      %%X Actual
DeltaX   = Xdes - Xact;                      %%DeltaX

%%Jacobian
clc;
JCB       = zeros(c+1,n);
ai        = [0;0;1];
v         = 1;
for JCLOOP  = 1:1:n
pos(:,p)  = [XX(v);YY(v);0];
JCB(:,v)  = cross(ai,pos(:,p));              %%Jacobian Matrix
v         = v+1;
p         = p+1;
end

%%DLS
JCBtps    = transpose(JCB);                  %%Jacobian Transpose
DLSinv   = (inv((JCBtps*JCB)+((Dpf)^2)*eye(n,n)))*(JCBtps);
DeltaTheta(1,:)= (DLSinv*DeltaX);
theta     = theta + DeltaTheta;
lp2       = lp2 + 1;
end
end
newang(Npts,:)= theta;
nxt         = nxt+1;
end
dlmwrite( 'C:\Users\Noel\Desktop\P2 Final\angles',newang,'delimiter','\t','precision','%0.4f');
