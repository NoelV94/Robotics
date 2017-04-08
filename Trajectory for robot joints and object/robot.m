clc
clear all;

input=dlmread('robot.key');
key_frame=input(1,1);
total_frame=input(1,2);
a = total_frame;
b = key_frame-1;          %%number of splines
[r,c] = size(input);      %%Number of columns is equal to number of joints
x=zeros((total_frame),c); %%Final matrix with .ang file values 
d = (a/b);                %%Distributing frames between the splines
e = 1;
f = 2;
i = 3;
j = 1;
for k = 1:1:c                %%Loop for each joint of the robot
  for l = 1:1:4              %%Loop for each spline
    for u=0:(1/d):1          %%Loop for frames in each spline
    P1=input(f,j);    
    P2=input((f+2),j);  
    D1=input(i,j);    
    D2=input((i+2),j);    
    h1=2*(u.^3)-3*(u.^2)+1;
    h2=(-2*(u.^3)+3*(u.^2));
    h3=(u.^3)-2*(u.^2)+u;
    h4=(u.^3)-(u.^2);
    y=(P1*h1)+(P2*h2)+(D1*h3)+(D2*h4);   
    x(e,j) = y;              %%Final Matrix containing .ang file values
    e=e+1;                
    end
    f=f+2;
    i=i+2;
  end
  f=2;
  i=3;
  e=1;
  j=j+1;
end
dlmwrite('robot.ang',a,'delimiter','');
dlmwrite( 'robot.ang',x,'-append','delimiter','\t','precision','%0.4f');
