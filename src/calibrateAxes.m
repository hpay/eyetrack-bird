
function  A = calibrateAxes(filepath_axes, p_axes, stereo_params)


%% Get the axes in the head reference frame
temp = dir(fullfile(filepath_axes, 'axes.mat'));
A_head = getfield(load(fullfile(filepath_axes, temp.name)),temp.name(1:end-4));
p0_head  = mean(A_head.Trajectories.Labeled.Data(strcmp(A_head.Trajectories.Labeled.Labels,'p0'),1:3,:),3);
p1_head  = mean(A_head.Trajectories.Labeled.Data(strcmp(A_head.Trajectories.Labeled.Labels,'px'),1:3,:),3);
p2_head  = mean(A_head.Trajectories.Labeled.Data(strcmp(A_head.Trajectories.Labeled.Labels,'py'),1:3,:),3);

p_head = [p0_head; p1_head; p2_head];
% figure; scatter3(p_head(:,1), p_head(:,2), p_head(:,3),50,[1 0 0; 0 1 0; 0 0 1],'filled'); axis vis3d;axis equal
% xlabel('x'); ylabel('y'); zlabel('z');
fprintf('HEAD AXES: x dist: %.2f mm, y dist %.2f mm\n',sqrt(sum((p1_head-p0_head).^2)), sqrt(sum((p2_head-p0_head).^2)))

vx_head = (p1_head-p0_head)/norm(p1_head-p0_head);
vy_head_temp = (p2_head-p0_head)/norm(p2_head-p0_head);
vz_head = cross(vx_head, vy_head_temp);
vz_head = vz_head/norm(vz_head);
vy_head = -cross(vx_head, vz_head);
vy_head = vy_head/norm(vy_head);
R_head = [vx_head(:) vy_head(:) vz_head(:)];

% Rotate 45 deg around x axis
R_45 = [1 0 0; 0 cosd(315) -sind(315); 0 sind(315) cosd(315)];

%% Eye reference frame
temp1 = dir(fullfile(filepath_axes, 'cam1.bmp'));
temp2 = dir(fullfile(filepath_axes, 'cam2.bmp'));

axes_filename_cam1 = fullfile(filepath_axes, temp1.name);
axes_filename_cam2 = fullfile(filepath_axes, temp2.name);

I1 = imread(axes_filename_cam1);
I2 = imread(axes_filename_cam2);

if size(I1,3)==3
   I1 = rgb2gray(I1);
   I2 = rgb2gray(I2);
end

% Enhance contrast
% I1b = (double(I1)-p_axes.thresh2)/(p_axes.thresh1-p_axes.thresh2)*255; I1b(I1b>255) = 255; I1b(I1b<0) = 0;
% I2b = (double(I2)-p_axes.thresh2)/(p_axes.thresh1-p_axes.thresh2)*255; I2b(I2b>255) = 255; I2b(I2b<0) = 0;

I1b = double(I1);
I2b = double(I2);

% Detect circles in the stereocamera axes images
[A1, p_eye1, r_eye1] = CircularHoughGrd(I1b, p_axes.RadiusRange, p_axes.thresh, p_axes.filter, 1);

vals = A1(sub2ind(size(A1),  round(p_eye1(:,2)), round(p_eye1(:,1))));
[~, order] = sort(vals,'descend');
p_eye1 = p_eye1(order,:);
r_eye1 = r_eye1(order,:);

% cla; imagesc(A1)
% hold on
% plot(p_eye1(:,1), p_eye1(:,2),'+r')

%%
[A2, p_eye2, r_eye2] = CircularHoughGrd(I2b, p_axes.RadiusRange, p_axes.thresh, p_axes.filter, 1);

vals = A2(sub2ind(size(A2),  round(p_eye2(:,2)), round(p_eye2(:,1))));
[~, order] = sort(vals,'descend');
p_eye2 = p_eye2(order,:);
r_eye2 = r_eye2(order,:);

figure('Pos',[  36         558        1204         420]);
subplot(1,2,1)
image(I1); colormap gray; axis image; hold on
viscircles(p_eye1, r_eye1,'Color','y');

subplot(1,2,2)
image(I2);axis image; hold on
hp = viscircles(p_eye2, r_eye2,'Color','y');


p_eye1 = p_eye1(1:min(length(r_eye1),3),:);
r_eye1 = r_eye1(1:min(length(r_eye1),3));
p_eye2 = p_eye2(1:min(length(r_eye2),3),:);
r_eye2 = r_eye2(1:min(length(r_eye2),3),:);

% Sort into p0, px, and py
[~, ind_y] = min(p_eye1(:,2));
[~, ind_x] = max(p_eye1(:,1));
ind_0 = find(~ismember(1:3, [ind_y ind_x]));
p_eye1 = p_eye1([ind_0; ind_x; ind_y],:);
[~, ind_y] = min(p_eye2(:,2));
[~, ind_x] = max(p_eye2(:,1));
ind_0 = find(~ismember(1:3, [ind_y ind_x]));
p_eye2 = p_eye2([ind_0; ind_x; ind_y],:);

subplot(1,2,1); title('Camera 1')
scatter(p_eye1(:,1),p_eye1(:,2), 50,[1 0 0; 0 1 0; 0 0 1],'filled')

subplot(1,2,2); title('Camera 2')
scatter(p_eye2(:,1),p_eye2(:,2), 50,[1 0 0; 0 1 0; 0 0 1],'filled')
legend(hp,'All','Good')

% Undistort and triangulate eye points
[p_eye, reprojection_errors] = triangulate(undistortPoints(p_eye1,stereo_params.CameraParameters1),...
    undistortPoints(p_eye2,stereo_params.CameraParameters2),stereo_params);
p_eye = p_eye(:,[1 3 2]); % Swap y and z
p_eye = p_eye*diag([1 1 -1]); % Invert z axis
disp(reprojection_errors)

% Find which point is which (p0, px, py)
% figure; scatter3(p_eye(:,1), p_eye(:,2), p_eye(:,3),30,[1 0 0; 0 1 0; 0 0 1],'filled'); axis vis3d; axis equal
% xlabel('x'); ylabel('y'); zlabel('z');

p0_eye = p_eye(1,:);
p1_eye = p_eye(2,:);
p2_eye = p_eye(3,:);

fprintf('EYE AXES: x dist: %.2f mm, y dist %.2f mm\n',sqrt(sum((p1_eye-p0_eye).^2)), sqrt(sum((p2_eye-p0_eye).^2)))

vx_eye = (p1_eye-p0_eye)/norm(p1_eye-p0_eye);
vy_eye_temp = (p2_eye-p0_eye)/norm(p2_eye-p0_eye);
vz_eye = cross(vx_eye, vy_eye_temp);
vz_eye = vz_eye/norm(vz_eye);
vy_eye = -cross(vx_eye, vz_eye);
vy_eye = vy_eye/norm(vy_eye);
R_eye = [vx_eye(:) vy_eye(:) vz_eye(:)];


A = [];
A.R_eye = R_eye;
A.R_head = R_head;
A.R_45 = R_45;
A.p0_eye = p0_eye;
A.p0_head = p0_head;


%% [x y z] Sanity checks of common axis
p_head_local = (p_head - p0_head)*R_head;
p_head_local2 = p_head_local*R_45; % Rotate around x axis by 45 deg to account to keep vertical truly vertical (axes is titled 45 so both camera and QTM can see it)

figure; subplot(2,2,1)
scatter3(p_head_local(:,1), p_head_local(:,2), p_head_local(:,3),50,[1 0 0; 0 1 0; 0 0 1],'filled'); axis vis3d;axis equal
xlabel('x'); ylabel('y'); zlabel('z');
title('Head')

subplot(2,2,2); title('Head, rotate 45')
scatter3(p_head_local2(:,1), p_head_local2(:,2), p_head_local2(:,3),50,[1 0 0; 0 1 0; 0 0 1],'filled'); axis vis3d;axis equal
xlabel('x'); ylabel('y'); zlabel('z');
title('Head, rotate 45')

p_eye_local = (p_eye - p0_eye)*R_eye;
p_eye_local2 = p_eye_local*R_45; % Rotate around x axis by 45 deg to keep vertical truly vertical (axes is titled 45 so both camera and QTM can see it)

subplot(2,2,3);
scatter3(p_eye_local(:,1), p_eye_local(:,2), p_eye_local(:,3),50,[1 0 0; 0 1 0; 0 0 1],'filled'); axis vis3d;axis equal
xlabel('x'); ylabel('y'); zlabel('z');
title('Eye')

subplot(2,2,4); title('Eye, rotate 45')
scatter3(p_eye_local2(:,1), p_eye_local2(:,2), p_eye_local2(:,3),50,[1 0 0; 0 1 0; 0 0 1],'filled'); axis vis3d;axis equal
xlabel('x'); ylabel('y'); zlabel('z');
title('Eye, rotate 45')