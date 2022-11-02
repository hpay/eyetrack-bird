% runEyetrackCalib
% Script to process camera calibration, run eye tracking, and store
% head/eye/beak calibration for use in gaze analysis
% 
% Hannah Payne 2022

close all

folder_eye_calib = 'HC09_221028b';
% folder_eye_calib = 'HC09_221028a';

% folder_eye_calib = 'HC08_221028a';
% folder_eye_calib = 'HC08_221028b';
% folder_eye_calib = 'HC08_221028c';

% folder_eye_calib = 'RBY47_221013';

% folder_eye_calib = 'SPP2_221013';

% AMB111/HC05 AFTER probe implant:
% folder_eye_calib = 'AMB111_220817c';

% Before probe implant:
% folder_eye_calib = 'AMB111_220717a';
% folder_eye_calib = 'IND102_220707a';
% folder_eye_calib = 'IND102_220630b';
% folder_eye_calib = 'IND102_220628';
% folder_eye_calib = 'IND102_220623c';
% folder_eye_calib = 'IND102_220623b';
% folder_eye_calib = 'IND102_220620b';
% folder_eye_calib = 'IND102_220620c';

downsample_eye = 6; % Set the downsampling rate at which the eye camera data was collected relative to QTM
data_root = 'Z:\Hannah\dualcamera';
camera_calib_root = 'Z:\Hannah\dualcamera\calibration';
camfilename = 'cam%i*.avi';

% pptDefaults
code_root = fileparts(which(mfilename));
% addpath('D:\hannah\Dropbox\alab\Code\project2\eye\helper')
addpath(fullfile(code_root,'src'));

filepath_eye = fullfile(data_root, folder_eye_calib)

temp = strfind(folder_eye_calib,'_')
folder_camera_calib = folder_eye_calib(temp+1:temp+6)
filepath_camera_calib = fullfile(camera_calib_root, folder_camera_calib)

% pptDefaults
code_root = fileparts(which(mfilename));
addpath(fullfile(code_root,'src'));

%% Get camera calibration
filepath_axes = fullfile(filepath_camera_calib, 'axes');
if ~exist(fullfile(filepath_camera_calib,'calib.mat'),'file') || ~exist(fullfile(filepath_camera_calib, 'axes.mat'),'file')
    calibrateCameras(filepath_camera_calib)
end
C = getfield(load(fullfile(filepath_camera_calib,'calib.mat')),'C');
A = getfield(load(fullfile(filepath_camera_calib,'axes.mat')), 'A');


%% Get the beak locations!
resume_beak =0;
if exist(fullfile(filepath_eye,'beak.mat'),'file')
    temp = load(fullfile(filepath_eye,'beak.mat'));
    p_beak1 = temp.p1; p_beak2 = temp.p2;
    if resume_beak % Add more beak points
        [p_beak1, p_beak2] = getBeak(filepath_eye, camfilename, p_beak1, p_beak2, temp.ii);
    end
else
    [p_beak1, p_beak2] = getBeak(filepath_eye, camfilename);
end

%% Load Qualisys rigid body
filename_qtm = 'qtm.mat'; % 6DOF
Q = getfield(load(fullfile(filepath_eye,filename_qtm)),'qtm');
N_head = Q.Frames;
t_head = (0:N_head-1)/Q.FrameRate;
dt_head = mean(diff(t_head));
p_rigidbody = squeeze(Q.RigidBodies.Positions(1,:,:))';
R_rigidbody = reshape(Q.RigidBodies.Rotations(1,:,:), [3,3,N_head]); % [X Y Z]

%% Track eye
if ~exist(fullfile(filepath_eye, 'eye.mat'),'file')     
    p = [];
    p.radiiPupil = 32;      % (pixels) 20
    p.radiiCR = [2 6];          % (pixels) [4 7]
    p.CRthresh = 10;        % (default 10) Threshold for subtracting background
    p.CRfilter = 8;         % (default 3, minimum 3). Set larger to detect imperfect circles
    p.CR_box_width = 100;   % (pixels) Crop image for faster CR search
    p.pupil_downsamp = 4;   % (pixels) factor to downsample for initial pupil search
    p.pupil_alpha = 2;      % Sensitivity to radial symmetry. 1 - slack, 3 - strict, ... and you can go higher
    p.minfeatures = .9;     % (fraction, [0 1]) min features required for starburst pupil detection
    p.smoothSigma = 3;      % (pixels) smooth image (2)
    p.ellipse_residual_thresh = 1.3; % Max mean residual to accept for final ellipse fit
    p.max_radialSymTransform = -40;  % If pupil guesses are bad, move lower to skip more frames (speeds up tracking but might miss more data)
    p.nCRs = 2;
    p.edge_thresh0 = 128;       % Initial guess, increase if pupil fit is too small, decrease to run faster
    p.max_bad_frames = 1;
    p.min_edge_thresh = 6;
    p.plot_on = 1;
    p.debug_on = 0;
    p.rerun = 1;                % 1 to restart from the first frame, 0 to continue from end
    p.pupil_start = [NaN NaN];
    p(2) = p(1); % Same settings for the second camera
    
    p(1).pupil_intensity = 20; % Adjust based on images if needed % 30 % 100/130
    p(1).iris_intensity = 70; % Adjust based on images if needed %80
    p(2).pupil_intensity = 20; % Adjust based on images if needed % 30 % 90/120
    p(2).iris_intensity = 70;  % Adjust based on images if needed %80
    
    trackEye(filepath_eye, p, camfilename); % TRACK THE PUPIL AND CR
end
Eraw = getfield(load(fullfile(filepath_eye, 'eye.mat')),'E');
N_eye = size(Eraw.pupil1,1);
dt_eye = dt_head*downsample_eye; % 1/fps
t_eye = (0:N_eye-1)'*dt_eye;
Eraw.t = t_eye;

if ~exist('xlims','var') || isempty(xlims); xlims = [0 Eraw.t(end)]; end

%% Clean up eye tracking results
scale = 1;
E = cleanEye(Eraw,scale);
plotEye(Eraw)
plotEye(E)

% E = Eraw%****TEMP

%% Convert pupil and corneal reflections from image to camera world coordinates
p_cornea0= NaN(N_eye, 3);
p_pupil0 = NaN(N_eye, 3);
nanmask = ~isnan(E.pupil1(:,1)) & ~isnan(E.cr1(:,1,2)) & ~isnan(E.pupil2(:,1)) & ~isnan(E.cr2(:,1,2));
[p_cornea0(nanmask,:),p_pupil0(nanmask,:)] = estimatepandc(...
    C.stereo_params, C.p_light1, C.p_light2,...
    E.pupil1(nanmask,1:2),E.pupil2(nanmask,1:2),...
    E.cr1(nanmask,1:2,1),E.cr2(nanmask,1:2,1),E.cr1(nanmask,1:2,2),E.cr2(nanmask,1:2,2));

%% Correction to ensure consistent length of pupil-center corneal curvature vectors

% TODO: compare to correction ALONG the axis of the eye, not in z axis
% of camera coordinate?

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
% figure;
% h = plot3(p_pupil(:,1), p_pupil(:,2), p_pupil(:,3),'ok'); hold on
% h(2) = plot3(p_cornea(:,1), p_cornea(:,2), p_cornea(:,3),'or'); hold on
% h(2) = plot3(p_pupil0(:,1), p_pupil0(:,2), p_pupil0(:,3),'+k'); hold on % Uncorrected
%
% % Plot vector of eye
% for ii = 1:N_eye
%     if ~isnan(p_pupil(ii,:))
%         plot3([p_pupil(ii,1) p_cornea(ii,1)],[p_pupil(ii,2) p_cornea(ii,2)], [p_pupil(ii,3) p_cornea(ii,3)],'k','LineWidth',.5);
%     end
% end
% 
% grid on ; axis equal; axis vis3d;
% legend(h, 'Pupil (uncorrected)','Pupil (corrected','Center of corneal curvature')
% xlabel('x'); ylabel('y'); zlabel('z - optical axis of cameras');
% view([0 -40])

%% Calculate eye locations/vectors and head locations/vectors relative to common axis reference frame

% Align eye points
p_pupil_common = (p_pupil - A.p0_eye)*A.R_eye*A.R_45; % A.p0_eye refers to dual camera measured in calibrateAxesBW
p_cornea_common = (p_cornea - A.p0_eye)*A.R_eye*A.R_45;

% Get a unit vector pointing in the direction of the eye
v_eye_common = (p_pupil_common-p_cornea_common)./sqrt(sum((p_pupil_common-p_cornea_common).^2, 2));

% Align head points
p_rigidbody_common = (p_rigidbody - A.p0_head)*A.R_head*A.R_45; % A.p0_head refers to dual camera measured in calibrateAxesBW

% Rotation matrix for the head rigid body
R_rigidbody_common = NaN(size(R_rigidbody));
for jj = 1:N_head
    R_rigidbody_common(:,1,jj) = R_rigidbody(:,1,jj)'*A.R_head*A.R_45;
    R_rigidbody_common(:,2,jj) = R_rigidbody(:,2,jj)'*A.R_head*A.R_45;
    R_rigidbody_common(:,3,jj) = R_rigidbody(:,3,jj)'*A.R_head*A.R_45;
end

% Downsample head to match eye 
temp = [];
temp.p = p_rigidbody_common;
temp.R = shiftdim(R_rigidbody_common,2);
temp = processStruct(temp, @(x) smooth(x, downsample_eye, 'moving'));
p_rigidbody_common_ds = temp.p(1:downsample_eye:N_eye*downsample_eye-1,:);
R_rigidbody_common_ds = shiftdim(temp.R(1:downsample_eye:N_eye*downsample_eye-1,:,:),1);


% p_rigidbody_common_ds = p_rigidbody_common(1:downsample_eye:N_eye*downsample_eye-1,:);
% R_rigidbody_common_ds = R_rigidbody_common(:,:,1:downsample_eye:N_eye*downsample_eye-1);

% % Fill in small gaps 
% maxgapi = round(maxgap/dt_eye);
% E = processStruct(E, @(x) interp1gap(x, maxgapi));
% 
% % Preserve NaN masking at this point 
% mask_temp = isnan(E.pupil1(:,1)) | isnan(E.pupil2(:,1)) | any(isnan(E.cr1(:,1,:)),3) | any(isnan(E.cr2(:,1,:)),3);
% 
% % median filter
% medfilti = round(medfilt/dt_eye);
% E = processStruct(E, @(x) medfilt1(x, medfilti,'omitnan','truncate'));
% 
% E.pupil1(mask_temp,:) = NaN;
% E.pupil2(mask_temp,:) = NaN;
% E.cr1(mask_temp,:,:) = NaN;
% E.cr2(mask_temp,:,:) = NaN;
% E.resid1(mask_temp,:) = NaN;
% E.resid2(mask_temp,:) = NaN;

%% Get eye points+vectors in local rigidbody frame. Eye = center of corneal curvature
p_eye_local = NaN(N_eye,3);
v_eye_local = NaN(N_eye,3);
for jj = 1:N_eye
    p_eye_local(jj,:) = R_rigidbody_common_ds(:,:,jj)'*(p_cornea_common(jj,:)-p_rigidbody_common_ds(jj,:))';
    v_eye_local(jj,:) = R_rigidbody_common_ds(:,:,jj)'*(v_eye_common(jj,:))';
end


%% Get beak in local rigidbody frame
p_beak = NaN(N_eye, 3);
[p_beak(~isnan(p_beak1(:,1)),:), reprojectionErrors] = triangulate(undistortPoints(p_beak1(~isnan(p_beak1(:,1)),:), C.stereo_params.CameraParameters1), undistortPoints(p_beak2(~isnan(p_beak2(:,1)),:), C.stereo_params.CameraParameters2),C.stereo_params);

% Convert from camera world coords to common world coords
p_beak_common = (p_beak - A.p0_eye)*A.R_eye*A.R_45;

% Convert from common world coords to local head coordinatesp_beak_local
p_beak_local_all = NaN(N_eye,3);
for jj = 1:N_eye
    p_beak_local_all(jj,:) = R_rigidbody_common_ds(:,:,jj)'*(p_beak_common(jj,:) - p_rigidbody_common_ds(jj,:))';
end
p_beak_local = nanmean(p_beak_local_all);

% Categorize as left and right eye 
divide_RL = 10;
mask_eye_R = p_eye_local(:,1)>divide_RL; % ***Update this automatically?

%% Get mask for eye positions that are out of range
% max_ant_eye_stds = 2;
max_eye_deviation = 1; % (mm)
getMax = @(x) nanmedian(x) + max_eye_deviation;
getMin= @(x) nanmedian(x) - max_eye_deviation;

% figure;
clf
 subplot(3,1,1)
histogram(p_eye_local(:,1), [-inf 0:.2:18 inf]); xlabel('p_eye_local (x coord/lateral pos, mm)','interp','none')
hold on; title('Lateral position')
dashedline(divide_RL, ylim,6,'Color','c');
dashedline(getMin(p_eye_local(mask_eye_R,1)), ylim,6,'Color','r');
dashedline(getMax(p_eye_local(mask_eye_R,1)), ylim,6,'Color','r');

dashedline(getMin(p_eye_local(~mask_eye_R,1)), ylim,6,'Color','r');
dashedline(getMax(p_eye_local(~mask_eye_R,1)), ylim,6,'Color','r');

subplot(3,1,2)
histogram(p_eye_local(:,2), [-inf 5:.2:12 inf]); xlabel('p_eye_local (y coord/anterior pos, mm)','interp','none')
hold on; title('Anterior position')
dashedline(getMin(p_eye_local(:,2)), ylim,6,'Color','r');
dashedline(getMax(p_eye_local(:,2)), ylim,6,'Color','r');


 subplot(3,1,3)
histogram(p_eye_local(:,3), [-inf -40:.2:-20 inf]); xlabel('p_eye_local (z coord/dorsal pos, mm)','interp','none')
hold on; title('Dorsal position')
dashedline(getMin(p_eye_local(:,3)), ylim,6,'Color','r');
dashedline(getMax(p_eye_local(:,3)), ylim,6,'Color','r');


% Mask Anterior
mask_lateral = NaN(length(p_eye_local),1);
mask_lateral(mask_eye_R) =  p_eye_local(mask_eye_R,1)>getMax(p_eye_local(mask_eye_R,1)) | p_eye_local(mask_eye_R,1)<getMin(p_eye_local(mask_eye_R,1));
mask_lateral(~mask_eye_R) =  p_eye_local(~mask_eye_R,1)>getMax(p_eye_local(~mask_eye_R,1)) | p_eye_local(~mask_eye_R,1)<getMin(p_eye_local(~mask_eye_R,1));

mask_anterior= p_eye_local(:,2)>getMax(p_eye_local(:,2)) | p_eye_local(:,2)<getMin(p_eye_local(:,2)) ;
mask_dorsal= p_eye_local(:,3)>getMax(p_eye_local(:,3)) | p_eye_local(:,3)<getMin(p_eye_local(:,3)) ;

mask_median_pos = mask_lateral | mask_anterior | mask_dorsal;
p_eye_local(mask_median_pos,:) = NaN;
v_eye_local(mask_median_pos,:) = NaN;

%% Store the mean vector and origin for each eye, plus the average beak location, in the head frame


% Get means for each eye
p_eye_R_local = nanmean(p_eye_local(mask_eye_R,:),1);
p_eye_L_local = nanmean(p_eye_local(~mask_eye_R,:),1);
v_eye_R_local = nanmean(v_eye_local(mask_eye_R,:),1); v_eye_R_local = v_eye_R_local/norm(v_eye_R_local);
v_eye_L_local = nanmean(v_eye_local(~mask_eye_R,:),1); v_eye_L_local = v_eye_L_local/norm(v_eye_L_local);

% Get the mean "head position": the mean of the two eyes
p_meaneye_local = mean([p_eye_R_local; p_eye_L_local]);

% OR get the intersection point of the two eye vectors:
A1 = p_eye_R_local;
A2 = p_eye_R_local+v_eye_R_local;
B1 = p_eye_L_local;
B2 = p_eye_L_local+v_eye_L_local;
nA = dot(cross(B2-B1,A1-B1)',cross(A2-A1,B2-B1)')';
nB = dot(cross(A2-A1,A1-B1)',cross(A2-A1,B2-B1)')';
d = dot(cross(A2-A1,B2-B1)',cross(A2-A1,B2-B1)')';
A0 = A1 + bsxfun(@times, nA./d, A2-A1);
B0 = B1 + bsxfun(@times, nB./d, B2-B1);
p_head_local = (A0+B0)/2; 


%% Get the mean "head vector": from mean of eyes to beak
vx_head_local = (p_beak_local-p_head_local)/norm(p_beak_local-p_head_local);

% Define a rotation matrix to convert from the local rigid body reference frame to the "head" (beak-eyes) reference frame.
vy_head_local = (p_eye_L_local - p_eye_R_local)/norm(p_eye_L_local-p_eye_R_local);
vy_head_local = vy_head_local - dot(vy_head_local, vx_head_local)*vx_head_local; % Get component of vy that is orthogonal to vx
vy_head_local= vy_head_local/norm(vy_head_local);
vz_head_local = cross(vx_head_local, vy_head_local);
% vz_head_local = null([vx_head_local, vy_head_local]); % TODO CHECK
% DIFF****************
R_head_local = [vx_head_local(:) vy_head_local(:) vz_head_local(:)]; 
% x = pointing from eye intersection to beak, y = pointing to the left, z = pointing up (airplane coords, so +pitch = upwards, +yaw = ?

% Save the results of this head/eye/beak calibration!! All measurements are
% relative to the QTM rigid body reference frame
H = [];
H.p_head = p_head_local(:);
H.p_beak = p_beak_local(:);
H.R_head = R_head_local; % This is the head coordinates (beak, left, up) relative to the rigid body ref frame
H.p_eye_R = p_eye_R_local(:);
H.p_eye_L = p_eye_L_local(:);
H.v_eye_R = v_eye_R_local(:);
H.v_eye_L = v_eye_L_local(:);
H.folder_eye_calib = folder_eye_calib;
H.folder_camera_calib = folder_camera_calib;

% Add local vectors of eye relative to head
H.v_eye_R_head = H.R_head'*H.v_eye_R;
H.v_eye_L_head = H.R_head'*H.v_eye_L;


% Get the eyes in local spherical coordinate relative to the
% eye-intersection -> beak vector
% theta_phi_L = theta_phi(H.v_eye_L_head)
% theta_phi_R = theta_phi(H.v_eye_R_head)

% Save calibration!
save(fullfile(data_root, folder_eye_calib, 'head_calibration.mat'),'-struct','H');

%% Save as a template for use with old data:
Ht = table;
Ht.folder = folder_eye_calib;
vx_template_local = (p_beak_local-p_meaneye_local)/norm(p_beak_local-p_meaneye_local);

% Define a rotation matrix to convert from the local rigid body reference frame to the "head" (beak-eyes) reference frame.
vy_template_local = (p_eye_L_local - p_eye_R_local)/norm(p_eye_L_local-p_eye_R_local);
vy_template_local = vy_template_local - dot(vy_template_local, vx_template_local)*vx_template_local; % Get component of vy that is orthogonal to vx
vy_template_local= vy_template_local/norm(vy_template_local);
vz_template_local = cross(vx_template_local, vy_template_local);
R_template_local = [vx_template_local(:) vy_template_local(:) vz_template_local(:)]; % Defined in rigid body ref frame
% x axis = mean of p_eyes to beak. y axis pointing left, z axis up

v_template_L = R_template_local'*H.v_eye_L ;  % left eye vector in this eye-mean reference frame
v_template_R = R_template_local'*H.v_eye_R ;  % right eye vector in this eye-mean reference frame
v_template_beak = R_template_local'*H.R_head(:,1); % beak vector (from eye-intersect to tip) in this eye-mean ref frame
Ht.p_head =  (R_template_local'*(p_head_local(:)-p_meaneye_local(:)))'; % eye-intersection relative to this eye-mean reference frame

theta_phi = @(v) [atan2d(v(2,:), v(1,:)); acosd(v(3,:)/norm(v))];

theta_phi_L = theta_phi(v_template_L);
theta_phi_R = theta_phi(v_template_R);
theta_phi_beak = theta_phi(v_template_beak);

% Take the average theta and phi of both eyes
theta_eye = (theta_phi_L(1)-theta_phi_R(1))/2;
phi_eye = mean([theta_phi_L(2) theta_phi_R(2)]);
all_phis = [theta_phi_L(2) theta_phi_R(2) theta_phi_beak(2)]
phi_eye_beak = mean(all_phis);

Ht.theta_beak = 0;
Ht.theta_eye = theta_eye;
Ht.phi_beak = phi_eye_beak;
Ht.phi_eye = phi_eye_beak;


save(fullfile(data_root, folder_eye_calib, 'head_calibration_template.mat'),'Ht');




%         [yaw, pitch, roll] = getYawPitchRollDeg(R_rigidbody); % TODO need
%         to rotate R axes so that yaw is relative to thead

%% Plot head and eye horiz angle in common global frame
% Get the position/rotation of the head in global coordinate frame
p_head = zeros(3,N_head);
R_head = zeros(3,3,N_head);
for ii = 1:3
    p_head = p_head + squeeze(R_rigidbody_common(:,ii,:))*H.p_head(ii);
    R_head(:,1,:) = R_head(:,1,:) + (R_rigidbody_common(:,ii,:))*H.R_head(ii,1);
    R_head(:,2,:) = R_head(:,2,:) + (R_rigidbody_common(:,ii,:))*H.R_head(ii,2);
    R_head(:,3,:) = R_head(:,3,:) + (R_rigidbody_common(:,ii,:))*H.R_head(ii,3);
end
p_head = p_head +  p_rigidbody_common';
p_head(:, isnan(p_rigidbody_common(:,1))) = NaN;
R_head(:,:,isnan(p_rigidbody_common(:,1))) = NaN;

[yaw_head, pitch_head, roll_head] = getYawPitchRollDeg(R_head);

% Get the eye position/vector in the global coordinate frame
yaw_eye= atan2d(v_eye_common(:,2), v_eye_common(:,1));
pitch_eye = squeeze(-asind(v_eye_common(:,3)));

% yaw_eye_smooth = smooth(interp1(t_eye,inpaint_nans(yaw_eye),  t_head,'linear'),'moving',3);
% pitch_eye_smooth = smooth(interp1(t_eye,inpaint_nans(pitch_eye),  t_head,'linear'),'moving',3);


xlims2 = [101 103.6]

% Plot head and eye example yaw
figure;  
plot(t_head, yaw_head,'k','LineWidth',1.5); ylabel('Yaw (°)'); hold on
% plot(t_head, yaw_eye_smooth,'c-');
plot(E.t, yaw_eye,'c.','MarkerSize',8);
% fixticks
plot(max(xlim) + [-1 0], -170+[0 0],'k')
plot(max(xlim) + [0 0], -170+[0 40],'k')
% axis off
xlim(xlims2)
legend('Head','Eye')


%% Plot head pos and speed example ** 
option_smooth = 1;
option_med_filt = 1;
% [ang_speed, ang_acc] = getAngSpeed(R_head, fps, option_smooth, option_med_filt);
% xlims2 = [121.23 125.23]
xlims2 = xlims
fps = 300
yaw_head_med_filt = medfilt1(yaw_head,7);
ang_speed = [0; smooth(abs(diff(yaw_head_med_filt)*fps),5,'moving')];

P_saccade.a = 60;       % (deg/s) Min angular head speed at saccade peak and at end
P_saccade.b = 0.5;      % (norm) Max normalized angular speed at start and end of saccade
P_saccade.T = 0.5;      % (s) Exclude saccades within T seconds of beginning/end of file
P_saccade.max_duration = 0.2;       % (s) Max saccade duration, only excludes a few saccades
P_saccade.min_intersaccade = 0.15;   % (s) min peak distance

S = detectHeadSaccadeRotm(R_head, t_head, P_saccade, 0);

figure;  
hs = subplot(2,1,1);
plot(t_head, yaw_head_med_filt,'k','LineWidth',1.5); ylabel('Yaw (°)'); hold on
plot(S.time, yaw_head_med_filt(S.ind), '+r');
fixticks
xlim(xlims2)

hs(2) = subplot(2,1,2);
plot(t_head, ang_speed,'k','LineWidth',1.5); ylabel('Speed (°/s)'); hold on
xlim(xlims2)
fixticks
linkaxes(hs,'x')

%% Plot local and eye-in-head angle - yaw is relative to rigid body frame
yaw_eye = atan2d(v_eye_local(:,2), v_eye_local(:,1));
% yaw_eye_smooth = smooth(interp1(t_eye,inpaint_nans(yaw_eye),  t_head,'linear'),'moving',3);

figure;  
plot(E.t, yaw_eye-nanmean(yaw_eye(~mask_eye_R)),'c.','MarkerSize',8); hold on
% plot(t_head, yaw_eye_smooth-nanmean(yaw_eye(~mask_eye_R)),'c-','MarkerSize',8); hold on

ylabel('Eye relative to rigid body (yaw)')
ylim([-20 20])
xlim(xlims2)
dashedline(xlim,[0 0]);

%% Plot local eye position in 3d
figure;
%     scatter3(p_eye_local(:,1),p_eye_local(:,2), p_eye_local(:,3),10, E.t); hold on;% Plot the raw data points
ds = 6; % downsample for plotting

scatter3(p_eye_local(1:ds:end,1),p_eye_local(1:ds:end,2), p_eye_local(1:ds:end,3),30, 'k.'); hold on;% Plot the raw data points
for ii = 1:ds:length(E.t)
    if ~isnan(p_eye_local(ii,1))
        plot3(p_eye_local(ii,1) + [0 v_eye_local(ii,1)],p_eye_local(ii,2)+ [0 v_eye_local(ii,2)], p_eye_local(ii,3)+ [0 v_eye_local(ii,3)],'k','LineWidth',.25);
    end
end

% Plot means
a = 6;
b = norm(p_beak_local-p_head_local);
msize = 60;
scatter3(p_head_local(1), p_head_local(2),p_head_local(3), msize, 'om','Filled')
scatter3(p_eye_R_local(1), p_eye_R_local(2), p_eye_R_local(3), msize, 'oc','Filled')
scatter3(p_eye_L_local(1), p_eye_L_local(2), p_eye_L_local(3), msize, 'oc','Filled')
plot3(p_eye_R_local(1)+[0 v_eye_R_local(1)*a], p_eye_R_local(2)+[0 v_eye_R_local(2)*a], p_eye_R_local(3)+[0 v_eye_R_local(3)*a], '-c','LineWidth',2) % Plot the mean local eye vectors
plot3(p_eye_L_local(1)+[0 v_eye_L_local(1)*a], p_eye_L_local(2)+[0 v_eye_L_local(2)*a], p_eye_L_local(3)+[0 v_eye_L_local(3)*a], '-c','LineWidth',2)

% Plot beak
mask = ~isnan(p_beak1(:,1));
colormap(jet)
scatter3(p_beak_local_all(mask,1),p_beak_local_all(mask,2), p_beak_local_all(mask,3),10^2,'.k'); hold on; % Plot the raw data points
scatter3(p_beak_local(:,1),p_beak_local(:,2), p_beak_local(:,3),40,'^m'); hold on;
plot3(p_head_local(1)+[0 vx_head_local(1)*b], p_head_local(2)+[0 vx_head_local(2)*b], p_head_local(3)+[0 vx_head_local(3)*b], '-m','LineWidth',2)

% Plot the local reference frame
%         plot3(a*[0 1 NaN 0 0 NaN  0 0], a*[0 0 NaN 0 1 NaN 0 0], a*[0 0 NaN 0 0 NaN 0 1],'k'); hold on

axis vis3d;     axis equal
% xlim([0 20]);  ylim([4 24]);   zlim([-50 -25])
% xlim([0 20]);  ylim([0 20]);   zlim([-45 -25])
xlabel('x');   ylabel('y'); zlabel('z')
set(gca,'Pos',[0 0 1 1])
% shrink([1 1.3])
view([0 0 1])
view([1 0 0])
grid on
% fixticks
% title(folder_eye_calib, 'Interp','none')


%% Plot local eye position in 2d (top down/zoomed in)
ds = 6; % downsample for plotting

figure;
subplot(1,2,1)
scatter(p_eye_local(1:ds:end,1),p_eye_local(1:ds:end,2),30, 'k.'); hold on;% Plot the raw data points
for ii = 1:ds:length(E.t)
    if ~isnan(p_eye_local(ii,1))
        plot(p_eye_local(ii,1) + [0 v_eye_local(ii,1)],p_eye_local(ii,2)+ [0 v_eye_local(ii,2)], 'k','LineWidth',.1);
    end
end

% Plot means
a = 6;
b = norm(p_beak_local-p_head_local);
msize = 60;
scatter(p_head_local(1), p_head_local(2), msize, 'om','Filled')
scatter(p_eye_L_local(1), p_eye_L_local(2), msize, 'oc','Filled')
plot(p_eye_R_local(1)+[0 v_eye_R_local(1)*a], p_eye_R_local(2)+[0 v_eye_R_local(2)*a], '-c','LineWidth',2) % Plot the mean local eye vectors
plot(p_eye_L_local(1)+[0 v_eye_L_local(1)*a], p_eye_L_local(2)+[0 v_eye_L_local(2)*a], '-c','LineWidth',2)

% Plot beak
mask = ~isnan(p_beak1(:,1));

colormap(jet)
scatter(p_beak_local_all(:,1),p_beak_local_all(:,2), 20^2,'.k'); hold on; % Plot the raw data points
scatter(p_beak_local(:,1),p_beak_local(:,2), 40,'^m'); hold on;
plot(p_head_local(1)+[0 vx_head_local(1)*b], p_head_local(2)+[0 vx_head_local(2)*b], '-m','LineWidth',2)

axis equal
xlim([3 7]);     ylim([7.5 11]);
xlabel('x');   ylabel('y');
grid on;   
% fixticks
title(folder_eye_calib, 'Interp','none')

subplot(1,2,2)
scatter(p_eye_local(1:ds:end,1),p_eye_local(1:ds:end,2),30, 'k.'); hold on;% Plot the raw data points
for ii = 1:ds:length(E.t)
    if ~isnan(p_eye_local(ii,1))
        plot(p_eye_local(ii,1) + [0 v_eye_local(ii,1)],p_eye_local(ii,2)+ [0 v_eye_local(ii,2)], 'k','LineWidth',.1);
    end
end

% Plot means
a = 6;
b = norm(p_beak_local-p_head_local);
msize = 60;
scatter(p_eye_R_local(1), p_eye_R_local(2), msize, 'oc','Filled')
scatter(p_eye_L_local(1), p_eye_L_local(2), msize, 'oc','Filled')
plot(p_eye_R_local(1)+[0 v_eye_R_local(1)*a], p_eye_R_local(2)+[0 v_eye_R_local(2)*a], '-c','LineWidth',2) % Plot the mean local eye vectors
plot(p_eye_L_local(1)+[0 v_eye_L_local(1)*a], p_eye_L_local(2)+[0 v_eye_L_local(2)*a], '-c','LineWidth',2)

% Plot beak
mask = ~isnan(p_beak1(:,1));

colormap(jet)
scatter(p_beak_local_all(:,1),p_beak_local_all(:,2), 20^2,'.k'); hold on; % Plot the raw data points
scatter(p_beak_local(:,1),p_beak_local(:,2), 40,'^m'); hold on;
plot(p_head_local(1)+[0 vx_head_local(1)*b], p_head_local(2)+[0 vx_head_local(2)*b], '-m','LineWidth',2)
axis equal
xlim([13.5 17.5]); ylim([7.5 11]);
xlabel('x');   ylabel('y');
grid on;    
% fixticks
title(folder_eye_calib, 'Interp','none')

%% Scatter plot of gaze in polar coords
figure; 

for which_eye  = [0 1]
    if which_eye == 1
        mask = mask_eye_R;
        theta_mean = atan2d(H.v_eye_R_head(2), H.v_eye_R_head(1));
        phi_mean = acosd(H.v_eye_R_head(3));
    else
        mask = ~mask_eye_R;
        theta_mean = atan2d(H.v_eye_L_head(2), H.v_eye_L_head(1));
        phi_mean = acosd(H.v_eye_L_head(3));
    end
    
    % Get local vectors of eye relative to head 
    v_eye_local_head = H.R_head'*v_eye_local';    
    theta = atan2d(v_eye_local_head(2,mask), v_eye_local_head(1,mask));
    phi = acosd(v_eye_local_head(3,mask));    
%     nanmean(v_eye_local_head(:,mask),2)
    
    subplot(1,2,which_eye+1)
    plot(theta-theta_mean, phi-phi_mean,'c.')
    axis equal
    a = 30; xlim([-a a]); ylim([-a a])
    grid on
    xlabel('Horizontal (°)')
    ylabel('Vertical (°)')
    
    std_thetas(which_eye+1) = nanstd(theta);
    std_phis(which_eye+1) = nanstd(phi);
    
    
end

overall_std = mean([std_thetas(:); std_phis(:)])

%% Detect head saccades
v_head = squeeze(R_rigidbody(:,1,:))'; % TO DO change this to allow detection of rotation in any axis!!!!!!

plot_on = 1;
P_saccade.a = 20;       % (deg/s) Min angular head speed at saccade peak
P_saccade.b = 0.5;      % (norm) Max normalized angular speed at start and end of saccade
P_saccade.T = 0.5;      % (s) Exclude saccades within T seconds of beginning/end of file
P_saccade.max_duration = 0.25;       % (s) Max saccade duration, only excludes a few saccades
P_saccade.min_intersaccade = 0.08;   % (s) min peak distance - chnged from 0.15
P_saccade.Fpass = 20; % (Hz) 25
P_saccade.FN = 2;
S = detectHeadSaccade(v_head, t_head,P_saccade, plot_on);
% xlim(xlims)

figure;
plot(p_rigidbody_common)
title('Head pos (common ref frame)')

%% For each head fixation,extract mean vector of the eye in the local head frame
angular_diff = @(v1,v2) acosd( dot(v1, v2, 2) ./...
    (sqrt(sum(v1.^2,2)).*sqrt(sum(v2.^2,2)))); % (DEG)
max_fix_dur = .5; % (s)
min_fix_coverage = 0.05; % (s)
max_std_eye = 0.1; % 0.01 % 0.05
max_std_head = 0.5; % 0.01

F = table();
F.time_start = [0; S.time_stop];
F.time_stop = [S.time_start; E.t(end)];
F.ind_start_head = [1; S.ind_stop];
F.ind_stop_head = [S.ind_start; N_eye];
F.ind_start_eye = floor(F.time_start/dt_eye)+1; % TODO: check +1?
F.ind_stop_eye = floor(F.time_stop/dt_eye)+1;
F.duration = F.time_stop - F.time_start;
F.eye_coverage = NaN(height(F),1);
F.std_eye = NaN(height(F),1);
F.std_head = NaN(height(F),1);
F.p_head_arena = NaN(height(F),3);
%     F.v_head_common = NaN(height(F),3);
F.p_eye_local = NaN(height(F),3);
F.v_eye_local = NaN(height(F),3);
F.eye_head_ang = NaN(height(F),1);

F(F.ind_stop_eye > N_eye,:) = [];
% eye_mask = ~isnan(v_eye(:,1));
% eye_inds = 1:length(v_eye_common);

for ii = 1:height(F)
    
    % Check coverage and std of eye
    n_coverage = nnz(~isnan(v_eye_common(F.ind_start_eye(ii):F.ind_stop_eye(ii),1)));
    F.eye_coverage(ii) = n_coverage*dt_eye;
    F.std_eye(ii) = mean(nanstd(p_eye_local(F.ind_start_eye(ii):F.ind_stop_eye(ii),:)));
    
    % Store head angle in QTM reference frame
    %         curr_head = p_rigidbody(F.ind_start_head(ii):F.ind_stop_head(ii),:);
    curr_head = p_rigidbody(F.ind_start_head(ii):F.ind_stop_head(ii),:);
    F.std_head(ii) = mean(nanstd(curr_head));
    F.p_head_arena(ii,:) = nanmean(curr_head);
    %         F.v_head_common(ii,:) = nanmean(v_rigidbody(F.ind_start_head(ii):F.ind_stop_head(ii),:));
    %         F.v_head_common(ii,:) = F.v_head_common(ii,:)/norm(F.v_head_common(ii,:));
    %
    % Exclude fixations where the head is moving, fixation is too long, or eye is not detected enough
    if F.duration(ii) <= max_fix_dur && F.std_head(ii)<= max_std_head &&  F.eye_coverage(ii) >= min_fix_coverage && F.std_eye(ii)<=max_std_eye
        
        %         % Simulate 52 deg (average deviation twn eye and head) and add noise to fake eye
        %         theta = 52;
        %         sigma = 0.008;
        %         F.eye_ang(ii,:) = rotateAbout(F.head_ang(ii,:), cross(F.head_ang(ii,:), F.eye_ang(ii,:)), theta);
        %         F.eye_ang(ii,:) =  F.eye_ang(ii,:)+mean(randn(n_coverage,3)*sigma); % add noise with sigma<--
        F.v_eye_local(ii,:) = nanmean(v_eye_local(F.ind_start_eye(ii):F.ind_stop_eye(ii),:));
        F.p_eye_local(ii,:) = nanmean(p_eye_local(F.ind_start_eye(ii):F.ind_stop_eye(ii),:));
        
        F.eye_head_ang(ii,:) = angular_diff([1 0 0], F.v_eye_local(ii,:));
        
    end
end

figure; subplot(2,1,1);
histogram(F.std_head,0:.01:.4); xlabel('std head')
subplot(2,1,2)
histogram(F.std_eye,0:.01:.4); xlabel('std eye')


Fcrop = F(~isnan(F.eye_head_ang),:);

% Detect when the R or L eye is facing the cameras based on head angle
Fcrop.R_eye_mask =  Fcrop.p_eye_local(:,1)>10;
Fcrop = sortrows(Fcrop,'R_eye_mask');

R_mean = nanmean(Fcrop.eye_head_ang(Fcrop.R_eye_mask));
L_mean = nanmean(Fcrop.eye_head_ang(~Fcrop.R_eye_mask));

% Calculate the deviation from mean eye-head angle over fixations
Fcrop.deviation_from_mean(Fcrop.R_eye_mask) = Fcrop.eye_head_ang(Fcrop.R_eye_mask) - R_mean;
Fcrop.deviation_from_mean(~Fcrop.R_eye_mask) = Fcrop.eye_head_ang(~Fcrop.R_eye_mask) - L_mean;

figure; histogram(Fcrop.deviation_from_mean,[-inf -20:2:20 inf], 'FaceColor','c')
xtext = 10;
ytext = max(ylim)*.7;
text(xtext, ytext, sprintf('σ = %.1f deg',nanstd(Fcrop.deviation_from_mean)))
axis square
xlabel('$\alpha_{EH} - \overline{\alpha}_{EH}$','Interpreter','latex')
ylabel('Number of fixations')
% title(folder_eye_calib,'Interp','none')
% shrink
% fixticks
xlim([-1 1]*22)

%% Plot the horizontal angle of the eye in local rigid body reference frame v. anterior position (in rigid body ref frame for now)

figure
subplot(1,2,2)
h_angle_eye_local = atan2d(v_eye_local(:,2), v_eye_local(:,1)); % in deg
h_angle_eye_local_R = atan2d(v_eye_R_local(2), v_eye_R_local(1)); % in deg
h_angle_eye_local_L = atan2d(v_eye_L_local(2), v_eye_L_local(1)); % in deg
% nanmean(h_angle_eye_local(mask_eye_R))

xlims1 = [7.5 9.5];
fprintf('Mean eye deviation from midline: %.1f deg\n', (h_angle_eye_local_L-h_angle_eye_local_R)/2)
plot(p_eye_local(mask_eye_R,2), h_angle_eye_local(mask_eye_R),'k.','MarkerSize',4);
xlabel('Anterior eye pos. (re: rigid body, mm)')
ylabel('Horizontal eye angle (deg)')
title('R eye')
xlim(xlims1); ylim([-20 20]+h_angle_eye_local_R)
grid on; axis square
subplot(1,2,1)
h_angle_eye_local = atan2d(v_eye_local(:,2), v_eye_local(:,1)); % in deg
plot(p_eye_local(~mask_eye_R,2), h_angle_eye_local(~mask_eye_R),'k.','MarkerSize',4);
xlabel('Anterior eye pos. (re: rigid body, mm)')
ylabel('Horizontal eye angle (deg)')
title('L eye')
xlim(xlims1); ylim([-20 20]+h_angle_eye_local_L)
grid on; axis square
% fixticks



%% Collect results here?
% ROS38: 6.1 (old, combined across sessions)
% CHC37: 8.7
% TRQ180: 6.1
% IND102_220707a:  5.6300
% AMB111: 6.5 + 4.7500 + 
 deviations_from_mean = [6.1 8.7 6.1 5.3 6.5 ]
 
 
 fprintf('Mean std of fixation angles %.2f+-%.2f deg, n = %i\n', mean(deviations_from_mean), std(deviations_from_mean)/sqrt(numel(deviations_from_mean)-1), numel(deviations_from_mean))

