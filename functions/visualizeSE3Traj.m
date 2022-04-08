function [ output_args ] = visualizeSE3Traj( trajectorySE3, color)
% Plots trajectory in SE(3) format
% arguments: 	
%   trajectorySE3 - input trajectory
%   color - color of the trajectory

trajLength = size(trajectorySE3,3);
path = zeros(trajLength,3);
for i=1:trajLength
    path(i,1) = trajectorySE3(1,4,i);
    path(i,2) = trajectorySE3(2,4,i);
    path(i,3) = trajectorySE3(3,4,i);
end
plot3(path(:,1),path(:,2),path(:,3), 'Color', color,'LineWidth',2);
axis equal

end

