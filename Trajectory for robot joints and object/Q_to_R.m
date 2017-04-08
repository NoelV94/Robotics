% Convert quaternion to rotation matrix
% Takes in a quaternion, returns corresponding rotation matrix

function R2=Q_to_R(Q)
    % n-column
    R2(1,1)=1-(2*(Q(2)*Q(2)+Q(3)*Q(3)));
    R2(2,1)=2*(Q(1)*Q(2)+Q(4)*Q(3));
    R2(3,1)=2*(Q(1)*Q(3)-Q(4)*Q(2));
    
    % o-column
    R2(1,2)=2*(Q(1)*Q(2)-Q(4)*Q(3));
    R2(2,2)=1-(2*(Q(1)*Q(1)+Q(3)*Q(3)));
    R2(3,2)=2*(Q(2)*Q(3)+Q(4)*Q(1));
    
    % a-column
    R2(1,3)=2*(Q(1)*Q(3)+Q(4)*Q(2));
    R2(2,3)=2*(Q(2)*Q(3)-Q(4)*Q(1));
    R2(3,3)=1-(2*(Q(1)*Q(1)+Q(2)*Q(2)));
end