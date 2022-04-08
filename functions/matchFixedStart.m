function [ matchedEstimate ] = matchFixedStart(groundTruth, estimate, matchPoseCount)
%This method matches trajectory estimate to the ground truth using SVD
%This version use start collocation
%matchPoseCount - count of the points at the begining

trajLength = size(groundTruth,3);
trajOnes = ones(1,trajLength);

SE3groundTruth = trajSE3ToXYZ(groundTruth);
SE3estimate = trajSE3ToXYZ(estimate);

%calculate translation
centroidGT = SE3groundTruth(:,1)';
centroidT =  SE3estimate(:,1)';

%put trajectories to [0 0 0]
SE3groundTruth = SE3groundTruth-kron(trajOnes, centroidGT');
SE3estimate = SE3estimate-kron(trajOnes, centroidT');

%seek rotation
[U S V] = svd(SE3estimate(:,1:matchPoseCount)*SE3groundTruth(:,1:matchPoseCount)');
R = V*U';

if det(R)<0
V(:,3) = -V(:,3);
R = V*U';
end

%apply transformations
matchedEstimate = estimate;
for i=1:trajLength
	matchedEstimate(1:3,4,i) = matchedEstimate(1:3,4,i)-centroidT';
end

rotationTransform = zeros(4,4);
rotationTransform(4,4) = 1;
rotationTransform(1:3,4) = centroidGT';
rotationTransform(1:3,1:3) = R;
for i=1:trajLength
	matchedEstimate(:,:,i) = rotationTransform*matchedEstimate(:,:,i);
end

end
