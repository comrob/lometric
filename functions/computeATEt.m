function [ statsATE ] = computeATEt(groundTruth, estimate)
%computes mean and rms of ATEt from SE(3) trajectories

    trajLength = size(estimate,3);
	ATEt = zeros(trajLength,1);
	ATEinconsistency =0;
	for i=1:trajLength
		ATE = groundTruth(:,:,i)\estimate(:,:,i);
		diffVector = estimate(1:3,4,i)-groundTruth(1:3,4,i);
		ATEt(i,1) = sqrt(diffVector'*diffVector);
		ATECheck = sqrt((ATE(1:3,4)')*ATE(1:3,4)); 
		ATEinconsistency = ATEinconsistency + abs(ATEt(i)-ATECheck);
	end
	ATEinconsistency = ATEinconsistency/trajLength;
        if ATEinconsistency > 0.00001
            fprintf(2,'ERROR: ATE inconsistency detected!  mean of ATEinc: %f\n',ATEinconsistency);
        end	

	meanATEt = sum(ATEt)/trajLength;
	rmsATEt = sqrt(((ATEt')*ATEt)/trajLength);
    statsATE = ErrorStats( min(ATEt), max(ATEt), meanATEt, rmsATEt);
end
