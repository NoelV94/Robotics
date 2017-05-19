%% Desired Trajectory
function [yd, yddot, ydddot] = DesiredTrajectory(omega,t)
%Values were provided
R = 0.25;
yd = [-0.866*R*cos(omega*t)-0.56 ;R*sin(omega*t) ; 0.5*R*cos(omega*t)-0.08];
%yddot and ydddot were found by differentiating yd with respect to time
yddot = [(433*R*omega*sin(t*omega))/500 ; R*omega*cos(t*omega); -(R*omega*sin(t*omega))/2]; 
ydddot = [ (433*R*omega^2*cos(t*omega))/500; -R*omega^2*sin(t*omega); -(R*omega^2*cos(t*omega))/2];
end
