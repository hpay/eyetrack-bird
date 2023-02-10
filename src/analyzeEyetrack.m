function [H, Ht, stats] = analyzeEyetrack(Eraw, p_pupil0, p_cornea0, p_beak,  Q_raw, A, downsample_eye,ploton)

% Params
% downsample_eye = 6; % Set the downsampling rate at which the eye camera data was collected relative to QTM
dir_figures = fullfile(fileparts(fileparts(which(mfilename))), 'results');
mkdir(dir_figures)

%% Load Qualisys rigid body
N_head = Q_raw.N_head;
fps_head = Q_raw.fps_head;
p_rigidbody = Q_raw.p_rigidbody;
R_rigidbody = Q_raw.R_rigidbody;


%% Get camera calibration
N_eye = size(Eraw.pupil1,1);
t_head = (0:N_head-1)/fps_head;
dt_eye = 1/fps_head*downsample_eye; % 1/fps
t_eye = (0:N_eye-1)'*dt_eye;
Eraw.t = t_eye;
xlims = [0 Eraw.t(end)];


%% Clean up eye tracking results
scale = 1;
[E, nanmask] = cleanEye(Eraw,scale);
plotEye(Eraw)
plotEye(E)

%% Correction to ensure consistent length of pupil-center corneal curvature vectors
p_pupil = p_pupil0;
p_cornea = p_cornea0;
p_pupil(nanmask,:) = NaN;
p_cornea(nanmask,:) = NaN;

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
% h(3) = plot3(p_pupil0(:,1), p_pupil0(:,2), p_pupil0(:,3),'+k'); hold on % Uncorrected
% 
% % Plot vector of eye
% for ii = 1:N_eye
%     if ~isnan(p_pupil(ii,:))
%         plot3([p_pupil(ii,1) p_cornea(ii,1)],[p_pupil(ii,2) p_cornea(ii,2)], [p_pupil(ii,3) p_cornea(ii,3)],'k','LineWidth',.5);
%     end
% end
% grid on ; axis equal; axis vis3d;
% legend(h, 'Pupil','Center of corneal curvature','Pupil (uncorrected)')
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

% Convert from camera world coords to common world coords
p_beak_common = (p_beak - A.p0_eye)*A.R_eye*A.R_45;

% Convert from common world coords to local rigidbody coordinates
p_beak_local_all = NaN(N_eye,3);
for jj = 1:N_eye
    p_beak_local_all(jj,:) = R_rigidbody_common_ds(:,:,jj)'*(p_beak_common(jj,:) - p_rigidbody_common_ds(jj,:))';
end
p_beak_local = mean(p_beak_local_all,'omitnan');
d_beak = sqrt(sum((p_beak_local_all-p_beak_local).^2,2));
beak_inds = find(~isnan(d_beak));
beak_dists = d_beak(~isnan(d_beak));
disp(table(beak_inds, beak_dists))

%% Determine orientation and categorize eyes as left and right 
p_head_local_temp = nanmean(p_eye_local,1); % very approximate
vx_head_local = (p_beak_local-p_head_local_temp)/norm(p_beak_local-p_head_local_temp);
flag_reverse = false;
if dot(vx_head_local, [0 1 0])<0
    flag_reverse = true;
end

divide_RL = 10;
if flag_reverse
    mask_eye_R = p_eye_local(:,1)<divide_RL; % ***Update this automatically?
else
    mask_eye_R = p_eye_local(:,1)>divide_RL; % ***Update this automatically?
end

%% Get mask for eye positions that are out of range
% max_ant_eye_stds = 2;
max_eye_deviation = 1; % (mm)
getMax = @(x) median(x,'omitnan') + max_eye_deviation;
getMin= @(x) median(x,'omitnan') - max_eye_deviation;

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
p_eye_R_local = mean(p_eye_local(mask_eye_R,:),1,'omitnan');
p_eye_L_local = mean(p_eye_local(~mask_eye_R,:),1,'omitnan');
v_eye_R_local = mean(v_eye_local(mask_eye_R,:),1,'omitnan'); v_eye_R_local = v_eye_R_local/norm(v_eye_R_local);
v_eye_L_local = mean(v_eye_local(~mask_eye_R,:),1,'omitnan'); v_eye_L_local = v_eye_L_local/norm(v_eye_L_local);

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

% Add local vectors of eye relative to head
H.v_eye_R_head = H.R_head'*H.v_eye_R;
H.v_eye_L_head = H.R_head'*H.v_eye_L;


% Get the eyes in local spherical coordinate relative to the
% eye-intersection -> beak vector
% theta_phi_L = theta_phi(H.v_eye_L_head)
% theta_phi_R = theta_phi(H.v_eye_R_head)

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
mask = find(~isnan(p_beak(:,1)));
colormap(jet)
scatter3(p_beak_local_all(mask,1),p_beak_local_all(mask,2), p_beak_local_all(mask,3),10^2,'.k'); hold on; % Plot the raw data points
% scatter3(p_beak_local_all(mask(1:end-1),1),p_beak_local_all(mask(1:end-1),2), p_beak_local_all(mask(1:end-1),3),30^2,'.'); hold on; % Plot the raw data points
scatter3(p_beak_local(:,1),p_beak_local(:,2), p_beak_local(:,3),40,'^m'); hold on;
plot3(p_head_local(1)+[0 vx_head_local(1)*b], p_head_local(2)+[0 vx_head_local(2)*b], p_head_local(3)+[0 vx_head_local(3)*b], '-m','LineWidth',2)

axis vis3d;     axis equal
xlabel('x');   ylabel('y'); zlabel('z')
set(gca,'Pos',[0 0 1 1])
view([0 0 1])
view([1 0 0])
grid on



%% Save as a template for use with old data:
Ht = table;
vx_template_local = (p_beak_local-p_meaneye_local)/norm(p_beak_local-p_meaneye_local);

% Define a rotation matrix to convert from the local rigid body reference frame to the "head" (beak-eyes) reference frame.
vy_template_local = (p_eye_L_local - p_eye_R_local)/norm(p_eye_L_local-p_eye_R_local);
vy_template_local = vy_template_local - dot(vy_template_local, vx_template_local)*vx_template_local; % Get component of vy that is orthogonal to vx
vy_template_local= vy_template_local/norm(vy_template_local);
vz_template_local = cross(vx_template_local, vy_template_local);
R_template_local = [vx_template_local(:) vy_template_local(:) vz_template_local(:)]; % Defined in rigid body ref frame
% x axis = mean of p_eyes to beak. y axis pointing left, z axis up

v_template_beak = R_template_local'*H.R_head(:,1); % beak vector (from eye-intersect to tip) in this eye-mean ref frame

v_template_L = R_template_local'*H.v_eye_L ;  % left eye vector in this eye-mean reference frame
v_template_R = R_template_local'*H.v_eye_R ;  % right eye vector in this eye-mean reference frame
Ht.p_head =  (R_template_local'*(p_head_local(:)-p_meaneye_local(:)))'; % eye-intersection relative to this eye-mean reference frame

% Set y coordinate to zero because it should be symmetric -- equivalent to
% averaging vectors from both eyes?
Ht.p_head(2)  = 0;

theta_phi = @(v) [atan2d(v(2,:), v(1,:)); acosd(v(3,:)/norm(v))];
theta_phi_L = theta_phi(v_template_L);
theta_phi_R = theta_phi(v_template_R);
theta_phi_beak = theta_phi(v_template_beak);

% Take the average theta and phi of both eyes and beak
theta_eye = (theta_phi_L(1)-theta_phi_R(1))/2;
phi_eye = mean([theta_phi_L(2) theta_phi_R(2)]);

Ht.theta_beak = 0;
Ht.theta_eye = theta_eye;
Ht.phi_beak = theta_phi_beak(2);
Ht.phi_eye = phi_eye;


%% Scatter plot of gaze in polar coords
figure('Units','Centi','Pos',[14   12   16   8]); 
hs  = gobjects(1,2);
dists= [];
nframes= [];
for which_eye  = [0 1]
    if which_eye == 1
        mask = mask_eye_R & ~isnan(v_eye_local(:,1));
        theta_mean = atan2d(H.v_eye_R_head(2), H.v_eye_R_head(1));
        phi_mean = acosd(H.v_eye_R_head(3));
    else
        mask = ~mask_eye_R & ~isnan(v_eye_local(:,1));
        theta_mean = atan2d(H.v_eye_L_head(2), H.v_eye_L_head(1));
        phi_mean = acosd(H.v_eye_L_head(3));
    end
    
    % Get local vectors of eye relative to head 
    v_eye_local_head = H.R_head'*v_eye_local';    
    theta = atan2d(v_eye_local_head(2,mask), v_eye_local_head(1,mask));
    phi = acosd(v_eye_local_head(3,mask));    
    nframes(which_eye+1) = length(theta);
    hs(which_eye+1) = subplot(1,2,which_eye+1);
%     plot(theta-theta_mean, phi-phi_mean,'co','MarkerSize',2,'Color',[0 1 1 1])
    scatter(theta-theta_mean, phi-phi_mean,9, 'c','MarkerEdgeAlpha',.3)
    axis equal
    a = 30; xlim([-a a]); ylim([-a a])
    grid on
    xlabel('Horizontal (°)')
    ylabel('Vertical (°)')
    
    std_thetas(which_eye+1) = nanstd(theta);
    std_phis(which_eye+1) = nanstd(phi);    
    dists = [dists sqrt((theta-theta_mean).^2+(phi-phi_mean).^2)];
end
set(hs,'TickLength',[0 0])
set(hs,'XTick',-30:10:30,'YTick',-30:10:30)
set(hs,'XTickLabelRotationMode','manual')
mean_dist = nanmean(dists);
std_theta = mean(std_thetas);
std_phi = mean(std_phis);
fprintf('Std horiz ang. %.2f deg, Std vert ang. %.2f deg, Mean distance %.2f deg\n',std_theta, std_phi, mean_dist)

stats = table;
stats.std_theta = std_theta;
stats.std_phi = std_phi;
stats.mean_dist = mean_dist;
stats.nframes = sum(nframes);

% export_fig(fullfile(dir_figures,'eye_scatter.png'),'-r300') % Careful not to
% overwrite earlier examples - run posterDefaults first
%%

%% Plot the horizontal angle of the eye in local rigid body reference frame v. anterior position (in rigid body ref frame for now)

figure
h_angle_eye_local = atan2d(v_eye_local(:,2), v_eye_local(:,1)); % in deg
h_angle_eye_local = wrapTo180(90-h_angle_eye_local);
h_angle_eye_local_R = nanmean(h_angle_eye_local(mask_eye_R)); % in deg
h_angle_eye_local_L = nanmean(h_angle_eye_local(~mask_eye_R)); % in deg
ylims1 = [-1 1]*.8;
msize = 5;

hs = subplot(1,2,1);
plot(h_angle_eye_local(~mask_eye_R),p_eye_local(~mask_eye_R,2)-p_eye_L_local(2),'c.','MarkerSize',msize);
xlabel('Horizontal angle (°)')
ylabel('Anterior pos. (mm)')
title('L eye')
ylim(ylims1); xlim([-20 20]+h_angle_eye_local_L)
grid on; axis square

hs(2) = subplot(1,2,2);
plot(h_angle_eye_local(mask_eye_R),p_eye_local(mask_eye_R,2)-p_eye_R_local(2), 'c.','MarkerSize',msize);
xlabel('Horizontal angle (°)')
title('R eye')
ylim(ylims1); xlim([-20 20]+h_angle_eye_local_R)
grid on; axis square

set(hs,'YTick',-.8:.4:.8)
set(hs,'TickLength',[0 0])

% export_fig(fullfile(dir_figures,'eye_shift.pdf'),'-r300') % Careful not to

%%
return

%% Plot local eye position in 2d (top down/zoomed in)
ds = 6; % downsample for plotting
a = 6;
msize = 60;
pad = [-1.5 1.5];

figure;
% Left eye
subplot(1,2,1)
scatter(p_eye_local(1:ds:end,1),p_eye_local(1:ds:end,2),30, 'k.'); hold on;% Plot the raw data points
for ii = 1:ds:length(E.t)
    if ~isnan(p_eye_local(ii,1))
        plot(p_eye_local(ii,1) + [0 v_eye_local(ii,1)],p_eye_local(ii,2)+ [0 v_eye_local(ii,2)], 'k','LineWidth',.1);
    end
end

% Plot means
scatter(p_eye_L_local(1), p_eye_L_local(2), msize, 'oc','Filled')
plot(p_eye_L_local(1)+[0 v_eye_L_local(1)*a], p_eye_L_local(2)+[0 v_eye_L_local(2)*a], '-c','LineWidth',2)
axis equal
xlim(p_eye_L_local(1)+pad);     ylim(p_eye_L_local(2)+pad);
xlabel('x');   ylabel('y');
grid on;  
% Right eye
subplot(1,2,2)
scatter(p_eye_local(1:ds:end,1),p_eye_local(1:ds:end,2),30, 'k.'); hold on;% Plot the raw data points
for ii = 1:ds:length(E.t)
    if ~isnan(p_eye_local(ii,1))
        plot(p_eye_local(ii,1) + [0 v_eye_local(ii,1)],p_eye_local(ii,2)+ [0 v_eye_local(ii,2)], 'k','LineWidth',.1);
    end
end

% Plot means
scatter(p_eye_R_local(1), p_eye_R_local(2), msize, 'oc','Filled')
plot(p_eye_R_local(1)+[0 v_eye_R_local(1)*a], p_eye_R_local(2)+[0 v_eye_R_local(2)*a], '-c','LineWidth',2) % Plot the mean local eye vectors
axis equal
xlim(p_eye_R_local(1)+pad);     ylim(p_eye_R_local(2)+pad);
xlabel('x');   ylabel('y');
grid on;    



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


%% Plot example: head and eye example yaw

% Get the eye position/vector in the global coordinate frame
[yaw_head, pitch_head, roll_head] = getYawPitchRollDeg(R_head);
yaw_eye= atan2d(v_eye_common(:,2), v_eye_common(:,1));
pitch_eye = squeeze(-asind(v_eye_common(:,3)));

% Early example: IND?
xlims2 = [101 103.6] 
yaw_eye_smooth = smooth(interp1(t_eye,inpaint_nans(yaw_eye),  t_head,'linear'),'moving',3);

figure;  
plot(t_head, yaw_head,'k','LineWidth',1.5); ylabel('Yaw (°)'); hold on
plot(t_head, yaw_eye_smooth,'c-');
plot(E.t, yaw_eye,'c.','MarkerSize',8);
plot(max(xlim) + [-1 0], -170+[0 0],'k')
plot(max(xlim) + [0 0], -170+[0 40],'k')
xlim(xlims2)
legend('Head','Eye')


%% Plot example: head pos and angular speed with saccades marked
xlims2 = xlims;
yaw_head_unwrap = unwrap(yaw_head*pi/180)*180/pi;

ang_speed = [0; smooth(abs(diff(yaw_head_unwrap)*Q_raw.fps_head),5,'moving')];

P_saccade.a = 60;       % (deg/s) Min angular head speed at saccade peak and at end
P_saccade.b = 0.5;      % (norm) Max normalized angular speed at start and end of saccade
P_saccade.T = 0.5;      % (s) Exclude saccades within T seconds of beginning/end of file
P_saccade.max_duration = 0.2;       % (s) Max saccade duration, only excludes a few saccades
P_saccade.min_intersaccade = 0.15;   % (s) min peak distance

S = detectHeadSaccadeRotm(R_head, t_head, P_saccade, 0);

figure;  
hs = subplot(2,1,1);
plot(t_head, yaw_head_unwrap,'k','LineWidth',1.5); ylabel('Yaw (°)'); hold on
plot(S.time, yaw_head_unwrap(S.ind), '+r');
xlim(xlims2)

hs(2) = subplot(2,1,2);
plot(t_head, ang_speed,'k','LineWidth',1.5); ylabel('Speed (°/s)'); hold on
xlim(xlims2)
linkaxes(hs,'x')






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



%% Collect results here?
% ROS38: 6.1 (old, combined across sessions)
% CHC37: 8.7
% TRQ180: 6.1
% IND102_220707a:  5.6300
% AMB111: 6.5 + 4.7500 + 
%  deviations_from_mean = [6.1 8.7 6.1 5.3 6.5 ]
%  
%  
%  fprintf('Mean std of fixation angles %.2f+-%.2f deg, n = %i\n', mean(deviations_from_mean), std(deviations_from_mean)/sqrt(numel(deviations_from_mean)-1), numel(deviations_from_mean))
