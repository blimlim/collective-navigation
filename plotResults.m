%% Code for Johnston and Painter to plot relevant information. Called by Homing_Script.m.

figure(3); hold on; plot(tSave,xPosition(:,:,1)); title('Average X Position'); xlabel('Time'); ylabel('Average X Position'); box on;
figure(4); hold on; plot(tSave,clusterMeasure); title('Measure of Clustering'); xlabel('Time'); ylabel('Measure of Clustering'); box on;
figure(5); hold on; plot(tSave,distanceToGoal.all(2:end)); title('Average Distance to Goal'); xlabel('Time'); ylabel('Average Distance to Goal'); box on;
figure(6); hold on; plot(tSave,(meanNeighbours.all(2:end))./nIndividualsRemaining.all(2:end)); title('Average Proportion of Neighbours'); xlabel('Time'); ylabel('Average Proportion of Neighbours'); box on;
figure(7); hold on; plot(tSave,meanNeighbours.all(2:end)); title('Average Neighbours'); xlabel('Time'); ylabel('Average Neighbours'); box on;
figure(8); hold on; plot(tSave,nIndividualsRemaining.all(2:end)/nIndividualsStart); title('Proportion of Individuals Remaining'); xlabel('Time'); ylabel('Proportion of Individuals Remaining'); box on;
figure(9); hold on; plot(tSave,nIndividualsRemaining.all(2:end)); title('Number of Individuals Remaining'); xlabel('Time'); ylabel('Number of Individuals Remaining'); box on;
figure(10); hold on; plot(linspace(-pi,pi,nHistDirection-1),directionHist.all(2:end)'/sum(sum(directionHist.all(2:end)))); xlim([-pi pi]);  title('Distribution of Angles'); xlabel('Angle'); ylabel('Frequency'); box on;