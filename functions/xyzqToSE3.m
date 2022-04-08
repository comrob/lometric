function [ trajSE3, usedPoses] = xyzqToSE3( trajTXYZQ )
% This function transforms trajectory from format [t, x, y, z, qw, qx, qy qz] to SE(3)
% usedPoses - if pose of the old trajectory was used or not (  trajSE3 = SE3(trajTXYZQ(usedPoses,:)) )

tLength = size(trajTXYZQ,1);
trajSE3 = zeros(4,4, tLength);
tmpPose = zeros(4,4);
tmpPose(4,4) = 1;
usedPoses = zeros(tLength,1);

outPoseIndex = 1;
for i=1:tLength
    tmpPose(1:3, 1:3)= quat2rotm(trajTXYZQ(i,5:8));
    tmpPose(1:3, 4) =  trajTXYZQ(i,2:4)';
    
    if 0==isnan(sum(sum(tmpPose)))  && 0==isinf(sum(sum(tmpPose))) 
        trajSE3(:,:,outPoseIndex) = tmpPose;
        usedPoses(outPoseIndex,1) = i;
        outPoseIndex = outPoseIndex + 1;
    end
end

%cut zero matrices in the output
trajSE3(:,:,outPoseIndex:end) = [];
usedPoses(outPoseIndex:end,:) = [];

end

