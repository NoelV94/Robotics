clc
clear all;

input=dlmread('object.key');
key_frames = input(1,1);                 
total_frames = input(1,2);
[r,c] = size(input);
a=zeros(1,4);
b=zeros(key_frames,4);
i=1;
for d=2:3:r-3              %%Form Rotation matrices from the input file and convert rotation to quaternion
    R = [input(d,1) input(d,2) input(d,3);input(d+1,1) input(d+1,2) input(d+1,3);input(d+2,1) input(d+2,2) input(d+2,3)];
    a=R_to_Q(R);
    b(i,:)=a;
    i=i+1;
end
key_q=b;                   %%Form a single matrix of quaternion values and perform quaternion interpolation
x0=Q_interpolation(key_q,key_frames,total_frames);
h=1;
z=0;
for g=0:1:total_frames;    %%The interpolated values are assembled in matrix form and converted into a rotation matrix
    R2=x0(h,:);
    k=Q_to_R(R2);
    x(1+z:3+z,:)=k;
    h=h+1;
    z=z+3;
end

clc;
input=dlmread('object.key');
%%X value for splines
i=1;
a1=0;
a0=input(2,c);
d=((total_frames)/(key_frames-1));
j=2;
m=5;
for v=1:1:(key_frames-1)
    a2=input(m,c)-a0-a1;
    for u=0:(1/d):1
     a0=input(j,c);
     e=(a2*(u^2))+(a1*u)+a0;
     x(i,c)=e;
     i=i+3;
    end
    j=j+3;
    m=m+3;
    a1=(2*a2)+a1;
end
%%Y value for splines
i=2;
a1=0;
a0=input(3,c);
d=((total_frames)/(key_frames-1));
j=3;
m=6;
for v=1:1:(key_frames-1)
    a2=input(m,c)-a0-a1;
    for u=0:(1/d):1
     a0=input(j,c);
     e=(a2*(u^2))+(a1*u)+a0;
     x(i,c)=e;
     i=i+3;
    end
    j=j+3;
    m=m+3;
    a1=(2*a2)+a1;
end
%%Z value for splines
i=3;
a1=0;
a0=input(4,c);
d=((total_frames)/(key_frames-1));
j=4;
m=7;
for v=1:1:(key_frames-1)
    a2=input(m,c)-a0-a1;
    for u=0:(1/d):1
     a0=input(j,c);
     e=(a2*(u^2))+(a1*u)+a0;
     x(i,c)=e;
     i=i+3;
    end
    j=j+3;
    m=m+3;
    a1=(2*a2)+a1;
end

dlmwrite('object.traj',total_frames,'delimiter','');
dlmwrite('object.traj',x,'-append','delimiter','\t','precision','%0.4f');


