function [ estimMatchedSE3, gtSE3, statsATE ] = evaluateTrajectory( groundTruth, estim, matchType, varargin )
% This function match and compute precision of a given trajectory wrt to ground truth
% all args variants: 
%   ( groundTruth, estim, matchOption )
%   ( groundTruth, estim, matchOption, gtFormat, estFormat )
%   ( groundTruth, estim, matchOption, gtFormat, estFormat, interpTimeLimit )
%

% main args
%  groundTruth - input ground truth trajectory in default format t x y z qx qy qz qw, where [qx, qy, qz, qw] is orientation represented by quaternion 
%  estim - estimated trajectory in default format t x y z qx qy qz qw, or other format if specified
%  matchType - enum MatchType with possibilities: MatchType.None, MatchType.LeastSquares, MatchType.FixedStart

% optional args
%  gtFormat - format of the ground truth, options: TrajectoryFormat.SE3, TrajectoryFormat.TXYZQXQYQZQW, TrajectoryFormat.TXYZQWQXQYQZ
%  estFormat -format of the estimated trajectory
%  MAX_POINT_TIME_DISTANCE - time interpolation removes point, if it is further than this threshold from others

%% Inner parametrizaion, input preprocessing

% function constants
MIN_POINTS_REQUIRED = 5;        % if there is less points in the trajectory, no evaluation is provided
OVERLAP_MIN_WARNING = 0.75;     % if overlap between trajectories (in time domain) is lower -> show warning
MAX_POINT_TIME_DISTANCE = 1000; % [s] in case of the interpolation, check if the time
                                %      is close enough to one of border
                                %      times, otherwise skip it

inputFormatGT  = TrajectoryFormat.TXYZQXQYQZQW;   
inputFormatEST = TrajectoryFormat.TXYZQXQYQZQW;

% handle varargin
argCount = size(varargin,2)+3;
switch argCount
    case 3 % default parameters
        inputFormatGT  = TrajectoryFormat.TXYZQXQYQZQW;    %default format, change it by varargin
        inputFormatEST = TrajectoryFormat.TXYZQXQYQZQW;
    case 5 % parse input args - there are input formats
        inputFormatGT = varargin{1}; %parse_input_format_args(varargin{1}, TUM_FORMAT, CLASSIC_FORMAT, SE3_FORMAT, WRONG_FORMAT); % gt format
        inputFormatEST = varargin{2}; % parse_input_format_args(varargin{2}, TUM_FORMAT, CLASSIC_FORMAT, SE3_FORMAT, WRONG_FORMAT); % estimate format   
    case 6 % there is specification of time constrain as well
        inputFormatGT = varargin{1}; %parse_input_format_args(varargin{1}, TUM_FORMAT, CLASSIC_FORMAT, SE3_FORMAT, WRONG_FORMAT); % gt format
        inputFormatEST = varargin{2}; %parse_input_format_args(varargin{2}, TUM_FORMAT, CLASSIC_FORMAT, SE3_FORMAT, WRONG_FORMAT); % estimate format   
        MAX_POINT_TIME_DISTANCE = varargin{3};
end

%% ---------------------- Handle input coordinate format ---------------------------------

% do the input format conversions
if inputFormatGT == TrajectoryFormat.TXYZQXQYQZQW
    groundTruth = changeQuatFormatToDefault(groundTruth);
end
if inputFormatEST == TrajectoryFormat.TXYZQXQYQZQW
    estim = changeQuatFormatToDefault(estim);
end

% check right combination of input formats
if (inputFormatGT == TrajectoryFormat.SE3 || inputFormatEST == TrajectoryFormat.SE3 ) && (inputFormatGT ~= inputFormatEST)
    error('Error: Inconsistent input format of trajectories.\n');
end    

%% ---------------------- Time synchronization: transformation to SE(3) ------------------
if (inputFormatGT == TrajectoryFormat.SE3 && inputFormatEST == TrajectoryFormat.SE3 )  %skip the interpolation in case of SE3 input (already times-ynchronized)
    gtSE3 = groundTruth;
    estSE3 = estim;
else %time synchronization is needed
    
    % check trajectory overlap
    testTrajectoryOverlap( groundTruth, estim, OVERLAP_MIN_WARNING);
    
    % linear + LERP interpolation
    pointsBeforeInterpolation = size(estim,1);
    [ gt_interp, estim_cut ] = interpolateLinearLERP( groundTruth, estim, MAX_POINT_TIME_DISTANCE );
    pointDiscardedByInterpolation = (1-size(estim_cut,1)/pointsBeforeInterpolation);
    if pointDiscardedByInterpolation>1-OVERLAP_MIN_WARNING
        fprintf(2, 'Warning: interpolation discarded %.2f%s of trajectory points.\n', 100*pointDiscardedByInterpolation, '%');
    end
    
    % check if there is reasonable count of points after the interpolation
    if size(estim_cut,1) < MIN_POINTS_REQUIRED
        error('Error: Not enough points to evaluate trajectories.\n');
    end
    
    % normalize quaternions
    estim_cut(:,5:8) = quatNormalize( estim_cut(:,5:8) );
    gt_interp(:,5:8) = quatNormalize( gt_interp(:,5:8) );
    
    % transformation to SE3 coordinate
    pointsBeforeSE3 = size(estim_cut,1);
    [gtSE3,  validInds] = xyzqToSE3( gt_interp );  % homogenize ground truth
    estim_cut = estim_cut(validInds, :);             % cut out unsed poses

    [estSE3, validInds] = xyzqToSE3( estim_cut );  %do the same trick with the second trajectory 
    gtSE3 = gtSE3(:,:, validInds);
    
    %test if there is enough points to further evaluation
    pointDiscardedBySE3 = (1-size(estSE3,3)/pointsBeforeSE3);
    if pointDiscardedBySE3>0
        fprintf(2, 'Warning: transformation to SE(3) discarded %.2f%s of trajectory points.\n', 100*pointDiscardedBySE3, '%');
    end
    if size(estSE3,3) < MIN_POINTS_REQUIRED
        error('Error: Not enough points to evaluate trajectories.\n');
    end
end

%% ---------------------  Trajectory matching -------------------------

if matchType == MatchType.LeastSquares
    estimMatchedSE3 = matchLeastSquares(gtSE3, estSE3);
elseif matchType == MatchType.FixedStart
    estimMatchedSE3 = matchFixedStart(gtSE3, estSE3, size(estSE3,3));
else % none matching
    estimMatchedSE3 = estSE3;
end    

%% ---------------------  Error calculation -------------------------

% ATEt calculation
statsATE = computeATEt(gtSE3, estimMatchedSE3);

end

%% Minor support functions

function [ overlapPercentage ] = testTrajectoryOverlap( groundTruth, estim, showWarnThreshold )
% This computes percentage overlap of estimated trajectory in comparison
% with the ground truth (both trajectories have to be sorted)
% percentage is computed from point count
% input format: [t ? ? ? .... ], [t ? ? ?  ...] only time columns are used
% throwErrorLevel - shows warning message if overlap is low

    %How many points are between first and last point of the ground truth trajectory
    firstTime = groundTruth(1,1);
    lastTime = groundTruth(end,1);
    overlapPercentage = sum((firstTime <= estim(:,1)).* (lastTime >= estim(:,1)))/size(estim,1);
    
    if overlapPercentage < showWarnThreshold
        fprintf(2, 'Warning: Time overlap of the trajectory estimate and ground truth is low: %.2f%s.\n', overlapPercentage*100.0,'%');
    end
    
end

function [clasTraj] = changeQuatFormatToDefault(tumTraj) % input format: [t x y z qx qy qz qw ]
    clasTraj = [tumTraj(:,1:4), tumTraj(:,8), tumTraj(:,5:7)];
end

function [ quaterNorm ] = quatNormalize( quaterStrange )
% It computes the quaternion norm
% quaternion have to be in column format: [q1 q2 q3 q4]
    quaterNorm = zeros(size(quaterStrange));
    for i=1:size(quaterStrange, 1)
        quaterNorm(i,:) = quaterStrange(i,:)/( quaterStrange(i,:)*(quaterStrange(i,:)')); 
    end
end
