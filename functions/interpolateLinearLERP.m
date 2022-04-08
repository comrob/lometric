function [ gt_interp, estim_cut ] = interpolateLinearLERP( groundTruth, estim, varargin )
% This interpolates ground truth to obtain corresponding points for the trajectory estimate
% all arg variants:
%   ( groundTruth, estim )
%   ( groundTruth, estim, timeConstrain )

MAX_POINT_TIME_DISTANCE = 1000; % [s] in case of the interpolation, check if the time
                                %      is close enough to one of border
                                %      times, otherwise skip the point

if size(varargin,2) == 1
    MAX_POINT_TIME_DISTANCE = varargin{1};                               
end                     

%% go through all lines of the estim
estLength = size(estim,1);
gt_interp = zeros(size(estim));
estim_cut = zeros(size(estim));  %not every point of the estimate is used during the interpolation

rowIndex = 1;
for i=1:estLength
    nearPoints = findNearestTimesInteg(groundTruth(:,1), estim(i,1));
    testErr = size(nearPoints);
    if testErr(1,2) ==2 % we can also test
        % test if the point is close enough
        if  (MAX_POINT_TIME_DISTANCE -(estim(i,1)-groundTruth(nearPoints(1,1),1))) > 0 || (MAX_POINT_TIME_DISTANCE -(groundTruth(nearPoints(1,2),1)-estim(i,1))) > 0
            estim_cut(rowIndex,:) = estim(i,:);
            gt_interp(rowIndex,1) = estim(i,1);
        
            if  rowIndex==1
                groundTruth(nearPoints(1,1),2:4);
                groundTruth(nearPoints(1,2),2:4);
            end
            
            h= (estim(i,1) - groundTruth(nearPoints(1,1),1))/(groundTruth(nearPoints(1,2),1) - groundTruth(nearPoints(1,1),1));
            % interpolate over x y z and quaternion - so it is lerp aproximation
            gt_interp(rowIndex,:) = groundTruth(nearPoints(1,1),:)+((groundTruth(nearPoints(1,2),:)-groundTruth(nearPoints(1,1),:))*h);
            rowIndex = rowIndex+1;
        end
    end
end

%cut off lines with zeros
gt_interp(rowIndex:end, :)= [];
estim_cut(rowIndex:end, :) = [];

end

function [Result_f] = findNearestTimesInteg(Matrix_f,value_f)
    %finds two indexes of neareast times to value_f in Matrix_f
    Result_f = zeros(1,2);

    arrayLength = size(Matrix_f,1);
    indCol = linspace(1,arrayLength,arrayLength)';
    Result_f(1,1) = max((Matrix_f(:,1)<=value_f).*indCol);
    [cvds, Result_f(1,2)] = max((Matrix_f(:,1)>value_f)-1);
    
    %indicate error
    if Result_f(1,1)<=0 || (Result_f(1,1) >= Result_f(1,2))
        Result_f = 0;
    end

end

