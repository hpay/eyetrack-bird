function  plotSaccadeTest(saccades)
figure
subplot(2,3,1)
histogram(saccades.angular_speed, linspace(0,2000,50),'FaceColor','k','EdgeColor','none','FaceAlpha',1)
xlabel('Peak angular speed')
axis square

subplot(2,3,2)
histogram(saccades.time_stop - saccades.time_start, 0:.0033:.15,'FaceColor','k','EdgeColor','none','FaceAlpha',1);
xlabel('Total duration (s)')
axis square

subplot(2,3,3)
histogram(diff(saccades.time), linspace(0,1,50),'FaceColor','k','EdgeColor','none','FaceAlpha',1);
xlabel('Inter saccade interval (s)')
axis square

subplot(2,3,4)
histogram(saccades.angular_dist, linspace(0,100,50),'FaceColor','k','EdgeColor','none','FaceAlpha',1);
xlabel('Total distance (deg)')
axis square


mask = 1:1:length(saccades.time);
subplot(2,3,5)
scatter(saccades.angular_dist(mask), saccades.time_stop(mask) - saccades.time_start(mask) + rand(size(saccades.time(mask)))*.0033, 2, 'o','MarkerFaceColor','k','MarkerEdgeColor','none')
xlabel('Total distance (deg)')
ylabel('Total duration (s)')
axis square
xlim([0 150])

subplot(2,3,6)
scatter(saccades.angular_dist(mask), saccades.angular_speed(mask), 2, 'o','MarkerFaceColor','k','MarkerEdgeColor','none')
ylabel('Peak angular speed (deg/s)')
xlabel('Total distance (deg)')
axis square
ylim([0 2000])
fixticks
xlim([0 150])

shrink(1.5)
% figure; 
% scatter(saccades.angular_dist, (saccades.time_stop - saccades.time_start).*saccades.angular_speed,4,'.k')