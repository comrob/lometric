% This example shows localization evaluation by ATE using different
% alignments
close all;
clear all;

groundTruth = loadMatrixFromFile( 'example_gt.txt', 1);
trajEstimate = loadMatrixFromFile( 'example_est.txt', 1);

%% Evaluation
[ estimMatchedSE3_non, gtSE3_non, statsATE_non ] = evaluateTrajectory( groundTruth, trajEstimate, MatchType.None);
[ estimMatchedSE3_sq,  gtSE3_sq,  statsATE_sq  ] = evaluateTrajectory( groundTruth, trajEstimate, MatchType.LeastSquares);
[ estimMatchedSE3_sqf, gtSE3_sqf, statsATE_sqf ] = evaluateTrajectory( groundTruth, trajEstimate, MatchType.FixedStart);

fprintf('Alignment by total station (considered correct): \n');
fprintf('  AVG(ATE) = %.2f m, RMS(ATE) = %.2f m\n', statsATE_non.avgErr, statsATE_non.rmsErr);
fprintf('Least squares alignment: \n');
fprintf('  AVG(ATE) = %.2f m, RMS(ATE) = %.2f m\n', statsATE_sq.avgErr, statsATE_sq.rmsErr);
fprintf('Fixed start alignment (Proposed): \n');
fprintf('  AVG(ATE) = %.2f m, RMS(ATE) = %.2f m\n', statsATE_sqf.avgErr, statsATE_sqf.rmsErr);

%% Visualize trajectories and error
visualizeAlignment(groundTruth, estimMatchedSE3_non, estimMatchedSE3_sqf, estimMatchedSE3_sq);

