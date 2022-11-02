% runEyetrackAll
% Old version -- wasn't keeping track of acrylic in front of calibration,
% results might be off?

pptDefaults
filepath = 'Z:\Hannah\behavior\dualcamera';
birds = {'ROS38', 'CHC37','TRQ180'};
% birds = {'CHC37'};
close all
angular_diff = @(v1,v2) acosd( dot(v1, v2, 2) ./...
    (sqrt(sum(v1.^2,2)).*sqrt(sum(v2.^2,2)))); % (DEG)


for ii = 1:length(birds)
    F = [];
    curr_bird = birds{ii};
    folders = dir(fullfile(filepath, [curr_bird '*']));
    for jj = 1:length(folders)
        results_filepath = fullfile(filepath, folders(jj).name,'results.mat' );
        if exist(results_filepath,'file')
            F_curr = getfield(load(results_filepath),'F');
            F_curr.folder = repmat(folders(jj).name, height(F_curr), 1);
            F_curr = F_curr(:,{'time_start','time_stop'...
                'duration','eye_ang','head_ang_all','head_ang_matched','eye_head_ang',...
                'std_eye','std_head','eye_coverage','R_eye_mask','deviation_from_mean'});
            
            
            if isempty(F)
                F = F_curr;
                sessions = 1;
            else
                F = [F; F_curr];
                sessions = sessions+1;
            end
        end
    end
    
    
    F.head_hang = atan2d(F.head_ang_matched(:,2), F.head_ang_matched(:,1)); % (DEG) horizontal angular position (azimuth)
    F.head_vang = 90-acosd(F.head_ang_matched(:,3));             % (DEG) vertical angular position (elevation - distance from horizontal)
    
    F.eye_hang = atan2d(F.eye_ang(:,2), F.eye_ang(:,1)); % (DEG) horizontal angular position (azimuth)
    F.eye_vang = 90-acosd(F.eye_ang(:,3));             % (DEG) vertical angular position (elevation - distance from horizontal)
    
    
    %     % Change eye_head_ang to be horizontal head angle
    %         F.eye_ang = F.eye_ang(:,1:2);
    %         F.head_ang_matched = F.head_ang_matched(:,1:2);
    %         F.head_ang_all = F.head_ang_all(:,1:2);
    
    max_std_eye = 0.01;
    max_std_head = 0.005; % Max from earlier: .01
    min_fix_coverage = 0.15; % (s) Changed from 0.1 5/29/2022
    
    fprintf('%s\n', curr_bird)
    
    % Mask based on head variability and sufficient eye data
    mask1 = F.std_head < max_std_head & F.eye_coverage >= min_fix_coverage & ~isnan(F.deviation_from_mean); % Mask based on head being stationary and which eye fixations have sufficient data
    F_head = F(mask1,:);
    F_head = sortrows(F_head,'std_eye','descend');
    F_eye = F_head(F_head.std_eye < max_std_eye,:);
    fprintf('Percent of fixations with std_eye>max_std_eye: %.1f%%\n', mean(F_head.std_eye>max_std_eye)*100)
    
    % Calculate angular differences and deviations from mean
    F_eye.eye_head_ang = angular_diff( F_eye.eye_ang,  F_eye.head_ang_matched);
    R_mean = nanmean(F_eye.eye_head_ang(F_eye.R_eye_mask));
    L_mean = nanmean(F_eye.eye_head_ang(~F_eye.R_eye_mask));
    F_eye.deviation_from_mean(F_eye.R_eye_mask) =  F_eye.eye_head_ang(F_eye.R_eye_mask) - R_mean;
    F_eye.deviation_from_mean(~F_eye.R_eye_mask) =  F_eye.eye_head_ang(~F_eye.R_eye_mask) - L_mean;
    
    
    % Give stats per bird
    fprintf('mean head-eye angle %.1f deg\n', ...
        nanmean([nanmean(F_eye.eye_head_ang(F_eye.R_eye_mask)), nanmean(F_eye.eye_head_ang(~F_eye.R_eye_mask))]))
    fprintf('stdev head-eye angle %.1f deg\n', nanstd(F_eye.deviation_from_mean))
    %     fprintf('95%% CI head-eye deviation %.1f to %.1f\n',  -std(F_eye.deviation_from_mean)*1.96, std(F_eye.deviation_from_mean)*1.96)
    %     fprintf('95th percentile range head-eye angle deviation %.2f to %.2f\n',  prctile(F_eye.deviation_from_mean, 2.5), prctile(F_eye.deviation_from_mean, 97.5))
    fprintf('%i fixations from %i sessions\n', height(F_eye),sessions)
    
    
    % Plot stdevs of eye and head
    %     figure; histogram(F.std_head,0:.0005:.03); xlabel('std head')
    %     title(curr_bird)
    %     shrink
    %     fixticks
    %     dashedline(max_std_head*[1 1], ylim)
    
    
    %     figure; histogram(F_head.std_eye,[0:.0005:.03 inf]); xlabel('std eye')
    %     title(curr_bird)
    %     shrink
    %     fixticks
    %     dashedline(max_std_eye*[1 1], ylim);
    
    figure; histogram(F_eye.deviation_from_mean,[-inf -25:25 inf], 'FaceColor','c')
    xtext = 10;
    ytext = max(ylim)*.7;
    text(xtext, ytext, sprintf('σ = %.1f deg',nanstd(F_eye.deviation_from_mean)))
    axis square
    xlabel('$\theta_{EH} - \overline{\theta}_{EH}$','Interpreter','latex')
    title(curr_bird)
    shrink
    fixticks
    xlim([-1 1]*22)
    
    %% Plot error by eye angle or head angle
    
    %{
figure; subplot(1,2,1)
plot(F_eye.eye_vang(F_eye.R_eye_mask), F_eye.deviation_from_mean(F_eye.R_eye_mask),'ok')
axis square
xlabel('eye elevation (deg)')
ylabel('R eye deviation from mean \theta')
subplot(1,2,2)
plot(F_eye.eye_vang(~F_eye.R_eye_mask), F_eye.deviation_from_mean(~F_eye.R_eye_mask),'ok')
xlabel('eye elevation (deg)')
ylabel('L eye deviation from mean \theta')
axis square
linkaxes

figure; subplot(1,2,1)
plot(F_eye.eye_hang(F_eye.R_eye_mask), F_eye.deviation_from_mean(F_eye.R_eye_mask),'ok')
axis square
xlabel('eye azimuth (deg)')
ylabel('R eye deviation from mean \theta')
subplot(1,2,2)
plot(F_eye.eye_hang(~F_eye.R_eye_mask), F_eye.deviation_from_mean(~F_eye.R_eye_mask),'ok')
xlabel('eye azimuth (deg)')
ylabel('L eye deviation from mean \theta')
axis square
    

%
figure; subplot(1,2,1)
plot(F_eye.head_vang(F_eye.R_eye_mask), F_eye.deviation_from_mean(F_eye.R_eye_mask),'ok')
axis square
xlabel('head elevation (deg)')
ylabel('R eye deviation from mean \theta')
subplot(1,2,2)
plot(F_eye.head_vang(~F_eye.R_eye_mask), F_eye.deviation_from_mean(~F_eye.R_eye_mask),'ok')
xlabel('head elevation (deg)')
ylabel('L eye deviation from mean \theta')
axis square
linkaxes

figure; subplot(1,2,1)
plot(F_eye.head_hang(F_eye.R_eye_mask), F_eye.deviation_from_mean(F_eye.R_eye_mask),'ok')
axis square
xlabel('head azimuth (deg)')
ylabel('R eye deviation from mean \theta')
subplot(1,2,2)
plot(F_eye.head_hang(~F_eye.R_eye_mask), F_eye.deviation_from_mean(~F_eye.R_eye_mask),'ok')
xlabel('head azimuth (deg)')
ylabel('L eye deviation from mean \theta')
axis square
    %}
    
    %
    %% Plot the distribution of saccade distances
    %{
    mask_eye_coverage = F.std_eye<=max_std_eye &  F.eye_coverage >= min_fix_coverage & ~isnan(F.eye_head_ang); % Mask based on which eye fixations have good data
    F.head_ang_all(~mask_eye_coverage,:) = NaN;
       
    angles_R = F.head_ang_all;
    angles_R(~F.R_eye_mask,:) = NaN;
    angles_L = F.head_ang_all;
    angles_L(F.R_eye_mask,:) = NaN;
    
    saccade_dist_R = angular_diff(angles_R(2:end,:), angles_R(1:end-1,:));
    saccade_dist_L = angular_diff(angles_L(2:end,:), angles_L(1:end-1,:));
    saccade_dist_all = [saccade_dist_R(~isnan(saccade_dist_R)); saccade_dist_L(~isnan(saccade_dist_L))];
    
    figure;
    histogram(saccade_dist_all,[0:1:100 inf], 'FaceColor','k'); hold on
    %      histogram(saccade_dist_R,[0:50 inf], 'FaceColor','r'); hold on
    %      histogram(saccade_dist_L,[0:50 inf], 'FaceColor','b'); hold on
    xtext = 30;
    ytext = max(ylim)*.8;
    text(xtext, ytext, sprintf('%.1f +- %.1f deg',nanmean(saccade_dist_all), nanstd(saccade_dist_all)))
    axis square
    xlabel('Saccade distance (deg)')
    title(curr_bird)
    shrink
    fixticks
    xlim([0 102])
    %}
    
    
    
end
%%
%{

    
end
%}

