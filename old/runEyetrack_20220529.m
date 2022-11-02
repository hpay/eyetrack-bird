% clear;
% close all

data_root = 'Z:\Hannah\dualcamera';
% data_root = 'C:\dualcamera'

folders = {'ROS38_220308a', 'ROS38_220308b', 'ROS38_220308c'};
downsample_eye = 6;
folder_calib = 'calibration\220309';
% xlims = [1.1 14.7];
xlims = [0 12];
% xlims = [27.1 38];
% xlims = [51.1 53.2];

% xlims = [44.8 45.8];

% folders = {'CHC37_220310a', 'CHC37_220310b', 'CHC37_220310c', 'CHC37_220310d', 'CHC37_220310e'};
% downsample_eye = 6;
% folder_calib = 'calibration\220310';
% xlims = [0 50];
% 
% folders = {'TRQ180_220310a','TRQ180_220310b','TRQ180_220310c'};
% downsample_eye = 6;
% folder_calib = 'calibration\220310';
% xlims = [0 50];
% xlims = [8.7194   15.0952];

for ifolder = 1%1:length(folders)
    folder = folders{ifolder}

resume = 0 % Resume eye tracking using params below


filepath = fullfile(data_root, folder)
filepath_calib = fullfile(data_root, folder_calib)

pptDefaults
code_root = fileparts(which(mfilename));
% addpath('D:\hannah\Dropbox\alab\Code\project2\eye\helper')
addpath(fullfile(code_root,'helper'));

% Track eye
if ~exist(fullfile(filepath, 'eye.mat'),'file') || resume  %***
    p = [];
    p.radiiPupil = 25;      % (pixels)
    p.radiiCR = 5;          % (pixels)
    p.CRthresh = 10;        % (default 10) Threshold for subtracting background
    p.CRfilter = 3;         % (default 3, minimum 3). Set larger to detect imperfect circles
    p.CR_box_width = 100;   % (pixels) Crop image for faster CR search
    p.pupil_downsamp = 2;   % (pixels) factor to downsample for initial pupil search
    p.pupil_alpha = 2;  % Sensitivity to radial symmetry. 1 - slack, 3 - strict, ... and you can go higher
    p.minfeatures = .9;     % (fraction, [0 1]) min features required for starburst pupil detection
    p.smoothSigma = 1;      % (pixels) smooth image
    p.ellipse_residual_thresh = 1.3; % Max mean residual to accept for final ellipse fit
    p.max_radialSymTransform = -100;  % If pupil guesses are bad, move higher to skip more frames
    p.nCRs = 2;
    p.edge_thresh0 = 128;       % Initial guess, increase if pupil fit is too small, decrease to run faster
    p.max_bad_frames = 1;
    p.min_edge_thresh = 8;
    p.rerun = 0;                % Restart from the first frame
    p.plot_on = 1;
    p.debug_on = 0;
    p.pupil_intensity = 30; % Adjust based on images if needed
    p.iris_intensity = 80; % Adjust based on images if needed
    
    trackEyeFlea3(filepath,p);
end
Eraw = getfield(load(fullfile(filepath, 'eye.mat')),'E');
N_eye = size(Eraw.pupil1,1);

%% Load head
filename_qtm = 'head.mat'; % "front" and "back" trajectories

% Load head movement
H_raw = getfield(load(fullfile(filepath,filename_qtm)),'head');
try
    H = H_raw.Trajectories.Labeled.Data(:,1:3,:); % Last point is marker radius, ignore it
    back_ind = strcmp(H_raw.Trajectories.Labeled.Labels,'back'); 
    if find(back_ind)==1 % Move "back" marker to row 2
                H = cat(1, H(2,:,:), H(1,:,:));
    end
catch
    H = H_raw.Trajectories.Unidentified.Data(:,1:3,:); % Last point is marker radius, ignore it
    % Identify the front marker as the one that moves more, move it to row 1
    vel1 = nanmean(sqrt(sum(diff(H(1,:,:), [],3).^2, 2)));
    vel2 = nanmean(sqrt(sum(diff(H(2,:,:), [],3).^2, 2)));
    if vel2>vel1
        H = cat(1, H(2,:,:), H(1,:,:));
    end
end
N_head = H_raw.Frames;
t_head = (0:N_head-1)/H_raw.FrameRate;
dt_head = mean(diff(t_head));

t_eye = t_head(1:downsample_eye:N_eye*downsample_eye);
dt_eye = mean(diff(t_eye));
Eraw.t = t_eye(:);

fprintf('%i n head, %i n eye, ratio %.2f\n', N_head, N_eye, N_head/N_eye)

%% Plot raw results
figure; maximize
hs = subplot(3,1,1);
hh = plot(t_head, squeeze(H(1,:,:))'); hold on
plot(t_head, squeeze(H(2,1,:))','--', 'Color',hh(1).Color);
plot(t_head, squeeze(H(2,2,:))','--', 'Color',hh(2).Color);
plot(t_head, squeeze(H(2,3,:))','--', 'Color',hh(3).Color);
legend('x','y','z');
title('Head marker position, markers 1 and 2')
ylabel('mm')

hs(2) = subplot(3,1,2);
meanx = nanmean(Eraw.pupil1(:,1));
meany = nanmean(Eraw.pupil1(:,2));
hp = plot(t_eye, Eraw.pupil1(:,1)-meanx,'Color',hh(1).Color); hold on
hp(2) = plot(t_eye, Eraw.pupil1(:,2)-meany,'Color',hh(2).Color); hold on
hp(3:4) = plot(t_eye, bsxfun(@minus, Eraw.cr1(:,1:2,1), [meanx meany]),'k','LineWidth',.5); hold on
hp(5:6) = plot(t_eye, bsxfun(@minus,Eraw.cr1(:,1:2,2), [meanx meany]),'Color',.5*[1 1 1],'LineWidth',.5);
uistack(hp(3:end),'bottom')
legend(hp([1 2 3 5]), {'pupilx','pupily','CRa x and y','CRb x and y'});
title('Eye position, camera 1 (pixels)')
ylabel('pixels')

hs(3) = subplot(3,1,3);
meanx = nanmean(Eraw.pupil2(:,1));
meany = nanmean(Eraw.pupil2(:,2));
plot(t_eye,  bsxfun(@minus, Eraw.cr2(:,1:2,1), [meanx meany]),'k','LineWidth',.5); hold on
plot(t_eye,  bsxfun(@minus, Eraw.cr2(:,1:2,2), [meanx meany]),'Color',.5*[1 1 1],'LineWidth',.5);
plot(t_eye, Eraw.pupil2(:,1) - meanx,'Color',hh(1).Color); hold on
plot(t_eye, Eraw.pupil2(:,2) - meany,'Color',hh(2).Color); hold on
title('Eye position, camera 2 (pixels)')
ylabel('pixels')
xlabel('Time (s)')

for ii = 1:3 % make axes bigger
    set(hs(ii), 'Position', [hs(ii).Position(1) .3*(3-ii)+.1 hs(ii).Position(3)  .25])
end
fixticks
linkaxes(hs, 'x')

xlim(xlims)

%% Clean up eye tracking results
scale = 2;% scale = 1 normally % **** TEMP
E = cleanEye(Eraw, scale); 
plotEye(Eraw)
plotEye(E)

%% Get 3D vector describing the angle of the head
p_hbeak = squeeze(H(1,:,:))';
p_hback = squeeze(H(2,:,:))';
medfilt = 0.05; % (s) % 0.05 % same as for eye
medfilti = round(medfilt/dt_head);
p_hback = [medfilt1(p_hback(:,1), medfilti) medfilt1(p_hback(:,2), medfilti) medfilt1(p_hback(:,3), medfilti) ];
p_hbeak = [medfilt1(p_hbeak(:,1), medfilti) medfilt1(p_hbeak(:,2), medfilti) medfilt1(p_hbeak(:,3), medfilti) ];


%% Get 3D vector describing the angle of whichever eye is visible

% Get camera calibration and position of IRLEDs
square_size = 4; % (mm)
if ~exist(fullfile(filepath_calib,'calib.mat'),'file')
    C = calibrateFlea3(filepath_calib, square_size);
else
    C = getfield(load(fullfile(filepath_calib,'calib.mat')),'C');
end

% Convert pupil and CR from image to world coordinates and get gaze vector
p_cornea0= NaN(N_eye, 3);
p_pupil0 = NaN(N_eye, 3);
nanmask = ~isnan(E.pupil1(:,1)) & ~isnan(E.cr1(:,1,2)) & ~isnan(E.pupil2(:,1)) & ~isnan(E.cr2(:,1,2));
[p_cornea0(nanmask,:),p_pupil0(nanmask,:)] = estimatepandc(...
    C.stereo_params, C.p_light1, C.p_light2,...
    E.pupil1(nanmask,1:2),E.pupil2(nanmask,1:2),...
    E.cr1(nanmask,1:2,1),E.cr2(nanmask,1:2,1),E.cr1(nanmask,1:2,2),E.cr2(nanmask,1:2,2));

% nanmask = 12713
% estimatepandc(...
%     C.stereo_params, C.p_light1, C.p_light2,...
%     E.pupil1(nanmask,1:2),E.pupil2(nanmask,1:2),...
%     E.cr1(nanmask,1:2,1),E.cr2(nanmask,1:2,1),E.cr1(nanmask,1:2,2),E.cr2(nanmask,1:2,2))
%% Get eye vector
p_pupil = p_pupil0;
p_cornea = p_cornea0;

dist_p_c = sqrt(sum((p_pupil0 - p_cornea0).^2,2));
K = median(dist_p_c,'omitnan'); % (mm)

% Camera coordinates:
% First column: x axis of image (cols)
% Second column: y axis of image (rows, down in actual space)
% Third column: pointing along optical axis away from cameras
zc = p_cornea(:,3);
xc = p_cornea(:,1);
yc = p_cornea(:,2);
xp = p_pupil(:,1);
yp = p_pupil(:,2);
zp = zc - real(sqrt(K^2 - (xc-xp).^2 - (yc-yp).^2));
p_pupil(:,3) = zp;

% Exclude points where the pupil-ccc distance is too far off from the median
thresh_dist_p_c = 0.1; % Fraction away from median distance to exclude
mask_dist_p_c = abs(dist_p_c - K)/K> thresh_dist_p_c*scale | abs(imag(zp))>eps;
p_pupil(mask_dist_p_c,:) = NaN;
p_pupil = real(p_pupil);
p_cornea(mask_dist_p_c,:) = NaN;
dist_p_c(mask_dist_p_c) = NaN;
fprintf('Mean pupil-ccc distance: %.2f mm. mask_dist_p_c %.3f\n', nanmean(dist_p_c), mean(mask_dist_p_c))


% Plot pupil and CCC in camera world coordinates
figure;
h = plot3(p_pupil(:,1), p_pupil(:,2), p_pupil(:,3),'ok'); hold on
h(2) = plot3(p_cornea(:,1), p_cornea(:,2), p_cornea(:,3),'or'); hold on
% h(2) = plot3(p_pupil0(:,1), p_pupil0(:,2), p_pupil0(:,3),'+k'); hold on % Uncorrected

% Plot vector of eye
for ii = 1:N_eye
    if ~isnan(p_pupil(ii,:))
        plot3([p_pupil(ii,1) p_cornea(ii,1)],[p_pupil(ii,2) p_cornea(ii,2)], [p_pupil(ii,3) p_cornea(ii,3)],'k','LineWidth',.5);
    end
end

grid on ; axis equal; axis vis3d;
legend(h, 'Pupil (uncorrected)','Pupil (corrected','Center of corneal curvature')
xlabel('x'); ylabel('y'); zlabel('z - optical axis of cameras');
view([0 -40])



% Fill in small gaps 
temp = [];
temp.p_pupil =p_pupil;
temp.p_cornea =p_cornea;
maxgap = 0.1; % (s)
medfilt= 0.1; % (s)
maxgapi = round(maxgap/dt_eye);
temp = processStruct(temp, @(x) interp1gap(x, maxgapi));

% Preserve NaN masking at this point 
mask_temp = isnan(temp.p_pupil(:,1));

% median filter
medfilti = round(medfilt/dt_eye);
temp = processStruct(temp, @(x) medfilt1(x, medfilti,'omitnan','truncate'));

temp.p_pupil(mask_temp,:) = NaN;
temp.p_cornea(mask_temp,:) = NaN;
p_pupil = temp.p_pupil;
p_cornea = temp.p_cornea;




%% Get the common axes in the eye and head reference frames
% Provide folder containing axes.mat file from QTM and pair of images
% named 'cam1.bmp' and 'cam2.bmp' of an three-marker triangle
% (long edge = x axis, short edge = y axis, rotated 45 deg from x axis)


filepath_axes = fullfile(filepath_calib, 'axes');
if exist(fullfile(filepath_axes, 'axes_results.mat'),'file')
    A = getfield(load(fullfile(filepath_axes,'axes_results.mat'),'A'),'A');
    R_eye = A.R_eye;
    R_head = A.R_head;
    p0_eye = A.p0_eye;
    p0_head = A.p0_head;
    R_45 = A.R_45;
else
    p_axes = [];
    p_axes.thresh1 = 140;
    p_axes.thresh2 = 60;
    p_axes.RadiusRange = [22 26];
    p_axes.thresh = 10;
    p_axes.filter = 8;
    
    A  = calibrateAxes(filepath_axes, p_axes, C.stereo_params);
    keyboard
    save(fullfile(filepath_axes,'axes_results.mat'),'p_axes', 'A');
end


% %% Save axes calibration results

%% Calculate eye locations and vectors and head locations and vectors relative to this reference frame

% Align head points
p_hbeak_align = (p_hbeak - p0_head)*R_head*R_45;
p_hback_align = (p_hback - p0_head)*R_head*R_45;

% Get a unit vector pointing in the direction of the beak
v_head = (p_hbeak_align - p_hback_align)./sqrt(sum((p_hbeak_align - p_hback_align).^2, 2));

% Align eye points
p_pupil_align = (p_pupil - p0_eye)*R_eye*R_45;
p_cornea_align = (p_cornea - p0_eye)*R_eye*R_45;

% Get a unit vector pointing in the direction of the eye
v_eye = (p_pupil_align-p_cornea_align)./sqrt(sum((p_pupil_align-p_cornea_align).^2, 2));

%% Plot results
figure('Pos',[4    48   954   945]);
hs = subplot(2,1,1);
hp = plot(t_head, p_hbeak_align); hold on
plot(t_head, p_hback_align(:,1),'--','Color',hp(1).Color)
plot(t_head, p_hback_align(:,2),'--','Color',hp(2).Color)
plot(t_head, p_hback_align(:,3),'--','Color',hp(3).Color)
title('Beak and top of head coordinates (mm)')

hs(2) = subplot(2,1,2);
plot(t_eye, p_pupil_align); hold on
plot(t_eye, p_cornea_align,'k--')
title('Pupil & cornea world coordinates (mm)')

linkaxes(hs,'x')
xlim(xlims)
fixticks

%% Plot eye and head vectors in common reference frame
figure('Pos',[4    48   954   945]);
hs = subplot(2,1,1);
plot(t_head, v_head)
title('Head vector')


% TEMP filtering for plotting
v_eye2 = v_eye;
v_eye2(:,1) = medfilt1(v_eye(:,1),5);
v_eye2(:,2) = medfilt1(v_eye(:,2),5);
v_eye2(:,3) = medfilt1(v_eye(:,3),5);

% Not working: need to align times
% eye_head_ang= NaN(size(v_head));
% for ii= 1:size(v_eye2,1)
% eye_head_ang(ii,:) = angular_diff(v_eye2(ii,:), v_head(ii,:));
% end

hs(2) = subplot(2,1,2);
plot(t_eye, v_eye2)
title('Eye vector')

linkaxes(hs,'x')
xlim(xlims)
fixticks
xlabel('Time (s)')


%% PLOT horizontal angle
nsmooth = 5;


hang_eye = smooth(rad2deg(atan2(v_eye2(:,2), v_eye2(:,1))),nsmooth,'moving');
hang_head = smooth(rad2deg(atan2(v_head(:,2), v_head(:,1))),nsmooth,'moving');

vang_eye = smooth(asind(v_eye2(:,3)),nsmooth,'moving');
vang_head = smooth(asind(v_head(:,3)),nsmooth, 'moving');

hang_head_mean = mean(hang_head(t_head>xlims(1) & t_head<xlims(2)))-60;
hang_eye = hang_eye-hang_head_mean;
hang_head = hang_head-hang_head_mean;

figure('Pos',[4    48   954   945]);
hs = subplot(2,2,1);
plot(t_head, hang_head); hold on
title('Head angle  - horiz. (deg)')

hs(3) = subplot(2,2,2);
plot(t_head, vang_head)
title('Head angle  - elev. (deg)')



% 
hs(2) = subplot(2,2,3);
plot(t_eye, hang_eye); hold on
title('Eye angle - horiz. (deg)')
% 
hs(4) = subplot(2,2,4);
plot(t_eye, vang_eye,'-','Marker','.'); hold on
title('Eye angle - elev. (deg)')

linkaxes(hs,'x')
xlim(xlims)
fixticks
xlabel('Time (s)')


%%
figure('Pos',[4    48   954   945]);
hs = subplot(2,1,1);
yyaxis left
plot(t_head, hang_head); hold on
yyaxis right
plot(t_head, vang_head)
title('Head angle (deg)')

hs(2) = subplot(2,1,2);
yyaxis left
plot(t_eye, hang_eye); hold on
yyaxis right
plot(t_eye, vang_eye); hold on
title('Eye angle - elev. (deg)')

linkaxes(hs,'x')
xlim(xlims)
fixticks
xlabel('Time (s)')

%% Detect head saccades
plot_on = 1;
P_saccade.a = 20;       % (deg/s) Min angular head speed at saccade peak
P_saccade.b = 0.5;      % (norm) Max normalized angular speed at start and end of saccade
P_saccade.T = 0.5;      % (s) Exclude saccades within T seconds of beginning/end of file
P_saccade.max_duration = 0.25;       % (s) Max saccade duration, only excludes a few saccades
P_saccade.min_intersaccade = 0.15;   % (s) min peak distance
P_saccade.Fpass = 20; % (Hz) 25
P_saccade.FN = 2;
S = detectHeadSaccade(v_head, t_head,P_saccade, plot_on);
xlim(xlims)


%% For each head saccade, calculate the angular distance for the eye and head
angular_diff = @(v1,v2) acosd( dot(v1, v2, 2) ./...
    (sqrt(sum(v1.^2,2)).*sqrt(sum(v2.^2,2)))); % (DEG)

max_fix_dur = .5; % (s)
min_fix_coverage = 0.1; % (s)
max_std_eye = 0.01;
max_std_head = 0.01;

F = table();
F.time_start = [0; S.time_stop];
F.time_stop = [S.time_start; t_eye(end)];
F.ind_start_head = [1; S.ind_stop];
F.ind_stop_head = [S.ind_start; N_eye];
F.duration = F.time_stop - F.time_start;
F.eye_ang = NaN(height(F),3);
F.head_ang_all = NaN(height(F),3);
F.head_ang_matched = NaN(height(F),3);
F.eye_head_ang = NaN(height(F),1);
F.std_eye = NaN(height(F),1);
F.std_head = NaN(height(F),1);
% eye_mask = ~isnan(v_eye(:,1));
eye_inds = 1:length(v_eye);
for ii = 1:height(F)
    F.ind_start_eye(ii) = floor(F.time_start(ii)/dt_eye)+1;
    F.ind_stop_eye(ii) = floor(F.time_stop(ii)/dt_eye)+1;
    n_coverage = nnz(~isnan(v_eye(F.ind_start_eye(ii):F.ind_stop_eye(ii),1)));
    F.eye_coverage(ii) = n_coverage*dt_eye;
    
    curr_eye = v_eye(F.ind_start_eye(ii):F.ind_stop_eye(ii),:);
    F.std_eye(ii) = mean(nanstd(curr_eye));
    
    curr_head = v_head(F.ind_start_head(ii):F.ind_stop_head(ii),:);
    F.std_head(ii) = mean(nanstd(curr_head));
    
    
    curr_eye_inds = eye_inds(F.ind_start_eye(ii):F.ind_stop_eye(ii));
    curr_eye_inds = curr_eye_inds(~isnan(curr_eye(:,1)));
    
    % Exclude fixations where the head is actually moving or fixation is
    % too long
    if F.duration(ii) <= max_fix_dur && ...
            F.std_head(ii)<= max_std_head % F.std_eye(ii)<=max_std_eye &&    F.eye_coverage(ii) >= min_fix_coverage && ...
        
        % Take all head points
        F.head_ang_all(ii,:) = nanmean(v_head(F.ind_start_head(ii):F.ind_stop_head(ii),:));
        F.head_ang_all(ii,:) = F.head_ang_all(ii,:)/norm(F.head_ang_all(ii,:));
        
        % Take same head points as the eye is tracked at
        curr_head_inds = curr_eye_inds*downsample_eye;
        F.head_ang_matched(ii,:) = nanmean(v_head(curr_head_inds,:));
        F.head_ang_matched(ii,:) = F.head_ang_matched(ii,:)/norm(F.head_ang_matched(ii,:));
        
        % Find average eye vector
        F.eye_ang(ii,:) = nanmean(curr_eye);
        
        %         % Simulate 52 deg (average deviation twn eye and head) and add noise to
        %         % fake eye
        %         theta = 52;
        %         sigma = 0.008;
        %         F.eye_ang(ii,:) = rotateAbout(F.head_ang(ii,:), cross(F.head_ang(ii,:), F.eye_ang(ii,:)), theta);
        %         F.eye_ang(ii,:) =  F.eye_ang(ii,:)+mean(randn(n_coverage,3)*sigma); % add noise with sigma<--
        %
        F.eye_ang(ii,:) =  F.eye_ang(ii,:)/norm( F.eye_ang(ii,:));
        F.eye_head_ang(ii,:) = angular_diff(F.eye_ang(ii,:), F.head_ang_all(ii,:));
        %     F.head_ang(ii,:) = nanmean(v_head(F.ind_stop_head(ii)-1:F.ind_stop_head(ii),:)); % TEMP
    end
end


% Detect when the R or L eye is facing the cameras based on head angle
F.R_eye_mask =  F.head_ang_matched(:,1)>0; % since the head vector is roughly aligned with the beak, and the QTM calibration was aligned to the camera module frame
R_mean = nanmean(F.eye_head_ang(F.R_eye_mask));
L_mean = nanmean(F.eye_head_ang(~F.R_eye_mask));

% Calculate the deviation from mean eye-head angle over fixations
F.deviation_from_mean(F.R_eye_mask) = F.eye_head_ang(F.R_eye_mask) - R_mean;
F.deviation_from_mean(~F.R_eye_mask) = F.eye_head_ang(~F.R_eye_mask) - L_mean;


figure; histogram(F.std_eye,[0:.0005:.05 inf])
xlabel('Eye std')
ylabel('Fixations')
hold on
dashedline(max_std_eye*[1 1], ylim);

figure;
subplot(1,2,1)
histogram(F.deviation_from_mean(F.R_eye_mask),[-inf -18:1:18 inf])
xtext = 10;
ytext = max(ylim)*.7;
text(10, ytext, sprintf('σ = %.1f deg',nanstd(F.deviation_from_mean(F.R_eye_mask)) ))
ylabel('Fixations')
axis square
xlabel('$\theta_{EH}^R - mean(\theta_{EH}^R)$','Interpreter','latex')
title('Right eye')

subplot(1,2,2)
histogram(F.deviation_from_mean(~F.R_eye_mask),[-inf -18:1:18 inf])
text(xtext, ytext, sprintf('σ = %.1f deg',nanstd(F.deviation_from_mean(~F.R_eye_mask)) ))
axis square
xlabel('$\theta_{EH}^L - mean(\theta_{EH}^L)$','Interpreter','latex')
title('Left eye')
fixticks
linkaxes
xlim([-1 1]*20)

% SAVE RESULTS
save(fullfile(filepath,'results.mat'),'F','P_saccade')

F_mask = F;
F_mask(isnan(F.eye_head_ang),:)  = [];
F_mask = sortrows(F_mask,'deviation_from_mean'); 
F_mask = F_mask(:,{'time_start','time_stop'...%
    'duration','std_eye','eye_coverage','eye_head_ang','deviation_from_mean','R_eye_mask','head_vang','head_hang'});

%%  Plot eye-head angle over time
%{
figure;
yyaxis left
hp = plot(t_head,-v_head(:,1));
% hp = plot(t_head,v_head(:,:));
hold on;
ylabel('head_x')
dashedline(xlim,[0 0]);

% hp(2) = plot(F.time_start,(F.eye_head_ang-nanmin(F.eye_head_ang))*2/range(F.eye_head_ang)-1,'-or');
yyaxis right
hp(2) = plot(F.time_start,F.eye_head_ang,'-o');
plot(F.time_start(F.R_eye_mask),F.eye_head_ang(F.R_eye_mask),'ob');
plot(F.time_start(~F.R_eye_mask),F.eye_head_ang(~F.R_eye_mask),'or');
ylabel('θ_{eye-head} (deg)')
% legend(hp,{'h_x','θ_{EH}'},'box','off')
xlim([8 100])
fixticks
% mask_outliers = abs(S_mask.diff_angle) > 30 ;
%}





%% Make example video
% imaskh = round(xlims(1)/dt_head):downsample_eye:round(xlims(2)/dt_head);
% imaske = round(xlims(1)/dt_eye) + (0:length(imaskh)-1);
% fps = 1/dt_eye/2;
% write_on = 1;
% exampleVideo(fullfile(filepath,'example_halfspeed_badfixation.avi'), fps, p_hback_align(imaskh,:), p_hbeak_align(imaskh,:),...
%     p_cornea_align(imaske,:), p_pupil_align(imaske,:),write_on)

end