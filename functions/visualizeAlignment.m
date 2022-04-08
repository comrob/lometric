function [ out ] = visualizeAlignment(groundTruth, noAlignment, fixedStartAlignment, leastSquaresAlignment)
%This function creates plot with trajectories using different alignments

% visualization params
textFontS = 12;
textFontName = 'SansSerif';

sblue   = '#0170C2';
cblue   = sscanf(sblue(2:end),'%2x%2x%2x',[1 3])/255;
sorange = '#ED7D31';
corange = sscanf(sorange(2:end),'%2x%2x%2x',[1 3])/255;
sgreen  = '#00AF52';
cgreen  = sscanf(sgreen(2:end),'%2x%2x%2x',[1 3])/255;
sred  = '#FB0102';
cred  = sscanf(sred(2:end),'%2x%2x%2x',[1 3])/255;
sfia  = '#7030A0';
cfia  = sscanf(sfia(2:end),'%2x%2x%2x',[1 3])/255;

% plot trajectories
figure(1);
set(gca,'FontSize',textFontS)
set(gca,'FontName',textFontName)
plot3(groundTruth(:,2),groundTruth(:,3),groundTruth(:,4),'b','Color', cblue, 'LineWidth',2);
hold on
visualizeSE3Traj( noAlignment, cfia )
visualizeSE3Traj( fixedStartAlignment, corange )
visualizeSE3Traj( leastSquaresAlignment,  cgreen )
legen1 = legend({'Ground truth','No alignment','Fixed start', 'Least squares'}, 'FontName', textFontName, 'Location', 'northwest','NumColumns',2 );
legen1.FontSize = textFontS;
legen1.Position = [0.1094 0.7101 0.7893 0.1012];
xlabel('x [m]', 'FontName', textFontName, 'FontSize',textFontS)
ylabel('y [m]', 'FontName', textFontName, 'FontSize',textFontS)
zlabel('z [m]', 'FontName', textFontName, 'FontSize',textFontS)
axis equal
view(0,90)

end
