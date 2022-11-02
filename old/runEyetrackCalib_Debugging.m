p = getfield(load(fullfile(filepath, 'eye.mat')),'p');

% Load the high res video
video1_filename = 'cam1*.avi';
video2_filename = 'cam2*.avi';

cam1 = dir(fullfile(filepath, video1_filename ));
cam2 = dir(fullfile(filepath, video2_filename ));
vid1 = VideoReader(fullfile(cam1.folder, cam1.name));
vid2 = VideoReader(fullfile(cam2.folder,cam2.name));

    %% Load single image 
    ii = 1036
    ii = 676
    
    I1 = single(rgb2gray(read(vid1,ii)));
    I2 = single(rgb2gray(read(vid2,ii)));
  
    p(1).pupil_start = E.pupil1(ii,1:2);
    p(2).pupil_start = E.pupil2(ii,1:2);
    
figure; shrink([1 .6])
plot_handles1 = [];
plot_handles1.ah = subplot(1,2,1);
imagesc(I1); colormap gray; axis image
apos =  [0.0125    0.025    0.45    0.95];
set(plot_handles1.ah,'Pos',apos);

hold on
plot(p1(ii,1), p1(ii,2),'+r')

plot_handles2 = [];
plot_handles2.ah = subplot(1,2,2);
imagesc(I2); colormap gray; axis image
set(plot_handles2.ah,'Pos',apos+[0.5 0 0 0]); drawnow

hold on
plot(p2(ii,1), p2(ii,2),'+r')
% Run pupil and CR detection in each image
% option_crop_width = 200; % Set to crop size
% option_enhance_contrast = 1;
% [pupil1,cr1, edge_thresh1, mean_residual1, points1] = detectPupilCR(I1, p(1).edge_thresh0, p(1));
% [pupil2, cr2, edge_thresh2, mean_residual2,points2] = detectPupilCR(I2, p(2).edge_thresh0, p(2));
% plot_handles1 = detectPupilCRPlot(I1, p(1), plot_handles1, pupil1, cr1, points1,option_crop_width,option_enhance_contrast);
% plot_handles2 = detectPupilCRPlot(I2, p(2), plot_handles2, pupil2, cr2, points2,option_crop_width,option_enhance_contrast);

title(num2str(ii))

%% Check QTM residuals for example points
iis = [6557  6540  5815 3897 762 2087 2906 6645]';
positions = [1 2 2 2 3 3 3 3]';

tts = E.t(iis);
i_heads = tts*H.FrameRate;

figure
subplot(2,2,1) % Plot position 
scatter(positions, squeeze(H.RigidBodies.Positions(1,:,i_heads))'-mean(squeeze(H.RigidBodies.Positions(1,:,i_heads))'),'LineWidth',1.5);
xlim([0 4]); xlabel('Category'); ylabel('RigidBody Position');

% Plot angles
YPRs = squeeze(H.RigidBodies.RPYs); % If using custom definition in QTM, this should be the same as the result in H_euler
H_euler = table;
[H_euler.yaw, H_euler.pitch, H_euler.roll] = getYawPitchRollDeg(H_rotation);
subplot(2,2,2)
scatter(positions, table2array(H_euler(i_heads,:)),'LineWidth',1.5);
xlim([0 4]); xlabel('Category'); ylabel('RigidBody Yaw Pitch Roll');


% Plot residuals
subplot(2,2,3)
scatter(positions, squeeze(H.RigidBodies.Residual(i_heads)),'LineWidth',1.5)
xlim([0 4]); xlabel('Category'); ylabel('RigidBody Residual');
ylim([0 0.3])

% Plot resids for individual markers
subplot(2,2,4)
scatter(positions+randn(8,4)*.02, squeeze(H.Trajectories.Labeled.Data(:,4,i_heads))',40,'LineWidth',1.5)
xlim([0 4]); xlabel('Category'); ylabel('Markers Residuals');





