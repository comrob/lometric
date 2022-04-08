function [ xyzTraj ] = trajSE3ToXYZ( homogTraj )
%Transforms SE(3) trajectory to [x; y; z;]
    trajLength = size(homogTraj,3);
    xyzTraj = zeros(3, trajLength);
    for i=1:trajLength
        xyzTraj(:,i) = homogTraj(1:3,4,i);
    end
end

