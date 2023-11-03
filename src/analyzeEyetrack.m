function [H, Ht, stats] = analyzeEyetrack(Eraw, p_pupil0, p_cornea0, p_beak,  Q_raw, A, downsample_eye)

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
    mask_eye_R = p_eye_local(:,1)<divide_RL; 
    mask_eye_L = p_eye_local(:,1)>divide_RL;
else
    mask_eye_R = p_eye_local(:,1)>divide_RL; 
    mask_eye_L = p_eye_local(:,1)<divide_RL;
end

%% Get mask for eye positions that are out of range
max_eye_deviation = 1; % (mm)
getMax = @(x) median(x,'omitnan') + max_eye_deviation;
getMin= @(x) median(x,'omitnan') - max_eye_deviation;

figure
subplot(3,1,1)
histogram(p_eye_local(:,1), [-inf 0:.2:18 inf]); xlabel('p_eye_local (x coord/lateral pos, mm)','interp','none')
hold on; title('Lateral position')
dashedline(divide_RL, ylim,6,'Color','c');
dashedline(getMin(p_eye_local(mask_eye_R,1)), ylim,6,'Color','r');
dashedline(getMax(p_eye_local(mask_eye_R,1)), ylim,6,'Color','r');

dashedline(getMin(p_eye_local(mask_eye_L,1)), ylim,6,'Color','r');
dashedline(getMax(p_eye_local(mask_eye_L,1)), ylim,6,'Color','r');

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
mask_lateral = false(length(p_eye_local),1);
mask_lateral(mask_eye_R) =  p_eye_local(mask_eye_R,1)>getMax(p_eye_local(mask_eye_R,1)) | p_eye_local(mask_eye_R,1)<getMin(p_eye_local(mask_eye_R,1));
mask_lateral(mask_eye_L) =  p_eye_local(mask_eye_L,1)>getMax(p_eye_local(mask_eye_L,1)) | p_eye_local(mask_eye_L,1)<getMin(p_eye_local(mask_eye_L,1));

mask_anterior= p_eye_local(:,2)>getMax(p_eye_local(:,2)) | p_eye_local(:,2)<getMin(p_eye_local(:,2)) ;
mask_dorsal= p_eye_local(:,3)>getMax(p_eye_local(:,3)) | p_eye_local(:,3)<getMin(p_eye_local(:,3)) ;

mask_median_pos = mask_lateral | mask_anterior | mask_dorsal;
p_eye_local(mask_median_pos,:) = NaN;
v_eye_local(mask_median_pos,:) = NaN;

% Update L/R eye masks to exclude NaNs
if flag_reverse
    mask_eye_R = p_eye_local(:,1)<divide_RL; % ***Update this automatically?
    mask_eye_L = p_eye_local(:,1)>divide_RL;
else
    mask_eye_R = p_eye_local(:,1)>divide_RL; % ***Update this automatically?
    mask_eye_L = p_eye_local(:,1)<divide_RL;
end

%% Store the mean vector and origin for each eye, plus the average beak location, in the head frame

% Get means for each eye
p_eye_R_local = mean(p_eye_local(mask_eye_R,:),1,'omitnan');
p_eye_L_local = mean(p_eye_local(mask_eye_L,:),1,'omitnan');
v_eye_R_local = mean(v_eye_local(mask_eye_R,:),1,'omitnan'); v_eye_R_local = v_eye_R_local/norm(v_eye_R_local);
v_eye_L_local = mean(v_eye_local(mask_eye_L,:),1,'omitnan'); v_eye_L_local = v_eye_L_local/norm(v_eye_L_local);

% Get the mean "head position": the mean of the two eyes
p_meaneye_local = mean([p_eye_R_local; p_eye_L_local]);

% for HC17: the intersection point doesn't work well, use mean of two eyes:
p_head_local = p_meaneye_local;

% OR get the intersection point of the two eye vectors:
% A1 = p_eye_R_local;
% A2 = p_eye_R_local+v_eye_R_local;
% B1 = p_eye_L_local;
% B2 = p_eye_L_local+v_eye_L_local;
% nA = dot(cross(B2-B1,A1-B1)',cross(A2-A1,B2-B1)')';
% nB = dot(cross(A2-A1,A1-B1)',cross(A2-A1,B2-B1)')';
% d = dot(cross(A2-A1,B2-B1)',cross(A2-A1,B2-B1)')';
% A0 = A1 + bsxfun(@times, nA./d, A2-A1);
% B0 = B1 + bsxfun(@times, nB./d, B2-B1);
% p_head_local = (A0+B0)/2;


%% Get the mean "head vector": from mean of eyes to beak
vx_head_local = (p_beak_local-p_head_local)/norm(p_beak_local-p_head_local);

% Define a rotation matrix to convert from the local rigid body reference frame to the "head" (beak-eyes) reference frame.
vy_head_local = (p_eye_L_local - p_eye_R_local)/norm(p_eye_L_local-p_eye_R_local);
vy_head_local = vy_head_local - dot(vy_head_local, vx_head_local)*vx_head_local; % Get component of vy that is orthogonal to vx
vy_head_local= vy_head_local/norm(vy_head_local);
vz_head_local = cross(vx_head_local, vy_head_local);
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

%% Plot local eye position in 3d (in rigid body coordinates)
figure;
ds = 6; % downsample for plotting

scatter3(p_eye_local(1:ds:end,1),p_eye_local(1:ds:end,2), p_eye_local(1:ds:end,3),30, 'k.'); hold on;% Plot the raw data points
for ii = 1:ds:length(E.t)
    if ~isnan(p_eye_local(ii,1))
        plot3(p_eye_local(ii,1) + [0 v_eye_local(ii,1)], ...
            p_eye_local(ii,2)+ [0 v_eye_local(ii,2)],...
            p_eye_local(ii,3)+ [0 v_eye_local(ii,3)],'k','LineWidth',.25);
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
scatter3(p_beak_local(:,1),p_beak_local(:,2), p_beak_local(:,3),40,'^m'); hold on;
plot3(p_head_local(1)+[0 vx_head_local(1)*b], p_head_local(2)+[0 vx_head_local(2)*b], p_head_local(3)+[0 vx_head_local(3)*b], '-m','LineWidth',2)

axis vis3d;     axis equal
xlabel('x');   ylabel('y'); zlabel('z')
set(gca,'Pos',[0 0 1 1])
view([0 0 1])
view([1 0 0])
grid on

%% Plot local eye position in 2d (in head coordinates)

% Get local vectors of eye relative to head
v_eye_local_head = (H.R_head'*v_eye_local')';
p_eye_local_head = (H.R_head'*(p_eye_local - p_head_local)')';
p_eye_R_head = H.R_head'*(H.p_eye_R - p_head_local(:));
p_eye_L_head = H.R_head'*(H.p_eye_L - p_head_local(:));
v_beak_head = H.R_head'*vx_head_local'; % should be [1 0 0]
color_pts = [67 127 138]/255;
a = 6; % length mean eyes
c = 6; % length individ eyes
b = norm(p_beak_local-p_head_local); % length beak
msize = 20;
lw = 1;

% Top view
figure;
ds = 6; % downsample for plotting
scatter(p_eye_local_head(1:ds:end,1),p_eye_local_head(1:ds:end,2),20,color_pts,'filled'); hold on;% Plot the raw data points
for ii = 1:ds:length(E.t)
    if ~isnan(p_eye_local_head(ii,1))
        plot(p_eye_local_head(ii,1) + c*[0 v_eye_local_head(ii,1)],...
            p_eye_local_head(ii,2)+ c*[0 v_eye_local_head(ii,2)],'Color',color_pts,'LineWidth',.25);
    end
end

% Plot eye means
scatter(p_eye_R_head(1), p_eye_R_head(2),msize, 'oc','Filled')
scatter(p_eye_L_head(1), p_eye_L_head(2),  msize, 'oc','Filled')
plot(p_eye_R_head(1)+[0 H.v_eye_R_head(1)*a], p_eye_R_head(2)+[0 H.v_eye_R_head(2)*a], '-c','LineWidth',lw) % Plot the mean local eye vectors
plot(p_eye_L_head(1)+[0 H.v_eye_L_head(1)*a], p_eye_L_head(2)+[0 H.v_eye_L_head(2)*a],'-c','LineWidth',lw)

% Plot beak
scatter(0,0, msize, 'ok','Filled')
plot([0 v_beak_head(1)*b], [0 v_beak_head(2)*b], '-k','LineWidth',lw)

xlabel('x position (mm)');   ylabel('y position (mm)');
set(gca,'Pos',[0.1 0.2 .7 .7])
shrink
view(-90, 90) %# Swap the axes
set(gca, 'ydir', 'reverse');
axis equal;
xlim([-2 19]); ylim(11*[-1 1])
set(gca,'XTick',0:5:15,'YTick',-10:5:10)
fixticks

% Side view
figure;
ds = 6; % downsample for plotting
scatter(p_eye_local_head(1:ds:end,1),p_eye_local_head(1:ds:end,3),20,color_pts,'filled'); hold on;% Plot the raw data points
for ii = 1:ds:length(E.t)
    if ~isnan(p_eye_local_head(ii,1))
        plot(p_eye_local_head(ii,1) + c*[0 v_eye_local_head(ii,1)],...
            p_eye_local_head(ii,3)+ c*[0 v_eye_local_head(ii,3)],'Color',color_pts,'LineWidth',.25);
    end
end

% Plot eye means
scatter(p_eye_R_head(1), p_eye_R_head(3),msize, 'oc','Filled')
scatter(p_eye_L_head(1), p_eye_L_head(3),  msize, 'oc','Filled')
plot(p_eye_R_head(1)+[0 H.v_eye_R_head(1)*a], p_eye_R_head(3)+[0 H.v_eye_R_head(3)*a], '-c','LineWidth',lw) % Plot the mean local eye vectors
plot(p_eye_L_head(1)+[0 H.v_eye_L_head(1)*a], p_eye_L_head(3)+[0 H.v_eye_L_head(3)*a],'-c','LineWidth',lw)

% Plot beak
scatter(0,0, msize, 'ok','Filled')
plot([0 v_beak_head(1)*b], [0 v_beak_head(3)*b], '-k','LineWidth',lw)

xlabel('x position (mm)');   ylabel('z position (mm)');
set(gca,'Pos',[0.1 0.2 .7 .7])
shrink
axis equal;
xlim([-2 19]); ylim(11*[-1 1])
set(gca,'XTick',0:5:15,'YTick',-10:5:10)
fixticks

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
dists= {};
nframes= [];
for which_eye  = [0 1]
    if which_eye == 1
        mask = mask_eye_R;
        theta_mean = atan2d(H.v_eye_R_head(2), H.v_eye_R_head(1));
        phi_mean = acosd(H.v_eye_R_head(3));
    else
        mask = mask_eye_L;
        theta_mean = atan2d(H.v_eye_L_head(2), H.v_eye_L_head(1));
        phi_mean = acosd(H.v_eye_L_head(3));
    end
    
    % Get local vectors of eye relative to head
    v_eye_local_head = H.R_head'*v_eye_local';
    theta = atan2d(v_eye_local_head(2,mask), v_eye_local_head(1,mask));
    phi = acosd(v_eye_local_head(3,mask));
    nframes(which_eye+1) = nnz(mask);
    hs(which_eye+1) = subplot(1,2,which_eye+1);
    scatter(theta-theta_mean, phi-phi_mean,9, 'c','MarkerEdgeAlpha',.3)
    axis equal
    a = 30; xlim([-a a]); ylim([-a a])
    grid on
    xlabel('Horizontal (°)')
    ylabel('Vertical (°)')
    
    std_thetas(which_eye+1) = nanstd(theta);
    std_phis(which_eye+1) = nanstd(phi);
    dists{which_eye+1} = sqrt((theta-theta_mean).^2+(phi-phi_mean).^2);
end
set(hs,'TickLength',[0 0])
set(hs,'XTick',-30:10:30,'YTick',-30:10:30)
set(hs,'XTickLabelRotationMode','manual')
mean_dist = cellfun(@mean,dists);
fprintf('Std horiz ang. %.2f %.2f deg, Std vert ang. %.2f %.2f deg (L/R)\n',std_thetas, std_phis)

stats = table;
stats.std_theta = std_thetas;
stats.std_phi = std_phis;
stats.nframes = nframes;
fixticks

%% Plot the horizontal angle of the eye in local rigid body reference frame v. anterior position (in rigid body ref frame for now)

figure
h_angle_eye_local = atan2d(v_eye_local(:,2), v_eye_local(:,1)); % in deg
h_angle_eye_local = wrapTo180(90-h_angle_eye_local);
h_angle_eye_local_R = nanmean(h_angle_eye_local(mask_eye_R)); % in deg
h_angle_eye_local_L = nanmean(h_angle_eye_local(mask_eye_L)); % in deg
ylims1 = [-1 1]*.8;
msize = 5;

hs = subplot(1,2,1);
plot(h_angle_eye_local(mask_eye_L),p_eye_local(mask_eye_L,2)-p_eye_L_local(2),'c.','MarkerSize',msize);
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
fixticks

[rhoL, pvalL] = corrcoef(h_angle_eye_local(mask_eye_L), p_eye_local(mask_eye_L,2));
[rhoR, pvalR] = corrcoef(h_angle_eye_local(mask_eye_R), p_eye_local(mask_eye_R,2));
stats.antpos_hang_r = [rhoL(2) rhoR(2)];
stats.antpos_hang_p = [pvalL(2) pvalR(2)];
