function [E, p] = trackEye(filepath_eye,p, camfilename)

close all
disp(filepath_eye)

video1_filename = sprintf(camfilename, 1); 
video2_filename = sprintf(camfilename, 2);
% addpath(fullfile(fileparts(mfilename('fullpath')), 'src'));

% Load the high res video
cam1 = dir(fullfile(filepath_eye, video1_filename ));
cam2 = dir(fullfile(filepath_eye, video2_filename ));
vid1 = VideoReader(fullfile(cam1.folder, cam1.name));
vid2 = VideoReader(fullfile(cam2.folder,cam2.name));
N_eye = min(round(vid1.Duration*vid1.FrameRate), round(vid2.Duration*vid2.FrameRate));

fprintf('Frames: %i\n', N_eye)
frameRate = vid1.FrameRate;

%% Set up figure
figure;
% maximize()
plot_handles1 = [];
plot_handles1.ah = subplot(1,2,1);
apos =  [0.0125    0.025    0.45    0.95];
set(plot_handles1.ah,'Pos',apos);

plot_handles2 = [];
plot_handles2.ah = subplot(1,2,2);
set(plot_handles2.ah,'Pos',apos+[0.5 0 0 0]); drawnow

%% Plot an example frame to get pupil and iris thresholds
p(1).example_t = 7; % Change current time if needed
pause(0.001);
I1 = single(rgb2gray(read(vid1, round(p(1).example_t*frameRate))));
I2 = single(rgb2gray(read(vid2, round(p(1).example_t*frameRate))));

% Run pupil and CR detection in each image
[pupil1_temp,cr1_temp, ~, ~, points1] = detectPupilCR(I1, p(1).edge_thresh0, p(1));
[pupil2_temp,cr2_temp, ~, ~, points2] = detectPupilCR(I2, p(2).edge_thresh0, p(2));
plot_handles1 = detectPupilCRPlot(I1, p(1), plot_handles1, pupil1_temp, squeeze(cr1_temp), points1);
plot_handles2 = detectPupilCRPlot(I2, p(2), plot_handles2, pupil2_temp, squeeze(cr2_temp), points2);

% plot_handles1 = detectPupilCRPlot(imadjust(I1/255, [35 150]/255), p(1), plot_handles1, pupil1_temp, squeeze(cr1_temp), points1);
% plot_handles2 = detectPupilCRPlot(imadjust(I2/255, [32 150]/255), p(2), plot_handles2, pupil2_temp, squeeze(cr2_temp), points2);


disp('Adjust pupil and iris intensities and quit/restart if needed')

%%
% keyboard

clear I1 I2 % to save memory

%% Init
if ~exist(fullfile(filepath_eye,'eye.mat'),'file')
    E = [];
    E.pupil1 = NaN(N_eye,5);
    E.cr1 = NaN(N_eye,3, p(1).nCRs);
    E.pupil2 = NaN(N_eye,5);
    E.cr2 = NaN(N_eye,3, p(1).nCRs);
    E.resid1 = NaN(N_eye,1);
    E.resid2 = NaN(N_eye,1);  
    E.points_fraction1 = NaN(N_eye,1);
    E.points_fraction2 = NaN(N_eye,1);
    ii_start = 1;
else
    old = load(fullfile(filepath_eye,'eye.mat'));
    E = old.E;
    ii_start = find(~isnan(E.pupil1(:,1)),1, 'last')+1;
    p(1).pupil_start = old.p(1).pupil_start;
    p(2).pupil_start = old.p(2).pupil_start;
    clear old
end
%}

% Initialize eye results, different settings for each camera
edge_thresh1 = p(1).edge_thresh0;
edge_thresh2 = p(2).edge_thresh0;
bad_frame_count = 0;

%% Loop over camera frames
% tic
for ii = ii_start:N_eye
    
    % Load image
    I1 = single(rgb2gray(read(vid1,ii)));
    I2 = single(rgb2gray(read(vid2,ii)));
    
    % Run pupil and CR detection in each image
    [E.pupil1(ii,:),E.cr1(ii,:,:), edge_thresh1, E.resid1(ii), points1] = ...
        detectPupilCR(I1, max(p(1).min_edge_thresh+4,edge_thresh1+4), p(1));
    [E.pupil2(ii,:), E.cr2(ii,:,:), edge_thresh2, E.resid2(ii), points2] = ...
        detectPupilCR(I2, max(p(2).min_edge_thresh+4,edge_thresh2+4), p(2));
    
    plot_handles1 = detectPupilCRPlot(I1, p(1), plot_handles1, E.pupil1(ii,:), squeeze(E.cr1(ii,:,:)), points1);
    plot_handles2 = detectPupilCRPlot(I2, p(2), plot_handles2, E.pupil2(ii,:), squeeze(E.cr2(ii,:,:)), points2);
    
    % If either one fails, reset position of both
    if isnan(E.pupil1(ii,1)) || isnan(E.pupil2(ii,1))
        E.pupil1(ii,:) = NaN;  E.pupil2(ii,:) = NaN;
        bad_frame_count = bad_frame_count+1;
        edge_thresh1 = p(1).edge_thresh0;
        edge_thresh2 = p(2).edge_thresh0;
        if bad_frame_count>=p(1).max_bad_frames
            p(1).pupil_start = [NaN NaN];
            p(2).pupil_start = [NaN NaN];
        end
    else
        % Init pupil search from last location
        p(1).pupil_start = E.pupil1(ii,1:2);
        p(2).pupil_start = E.pupil2(ii,1:2);
        bad_frame_count = 0;
    end
    
    fprintf('%i.\n',ii)
    if mod(ii, 100)==0
        %         total_time = toc;
        %         fprintf('%.2f frames per second processing speed\n',ii/total_time);
        save(fullfile(filepath_eye,'eye.mat'),'E','p','ii')
    end
end
% total_time = toc;
% fprintf('% frames per second processing speed\n',N_eye/total_time);
save(fullfile(filepath_eye,'eye.mat'),'E','p')


%% Plot results
figure;
% maximize;
subplot(2,1,1); plot(E.pupil1(:,1:2)); title('Camera 1 pupil (x and y)')
subplot(2,1,2); plot(E.pupil2(:,1:2)); title('Camera 2 pupil (x and y)')
linkaxes

figure;
% maximize;
subplot(2,1,1); plot(E.cr1(:,1:2)); title('Camera 1 CR (x and y)')
subplot(2,1,2); plot(E.cr2(:,1:2)); title('Camera 2 CR (x and y)')
linkaxes