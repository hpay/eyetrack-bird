function C = calibrateLights(fileroot, stereo_params)
% calibrateDualCameras(fileroot)
%
% Obtain internal and external camera parameters and external light params
%
% Take 4-10 images of a calibrated checkerboard pattern (4 mm) while it was moved around
% the cameras at various angles, save in two subfolders cam1 and cam2
% 
% Also take images of each light source in a mirror with a checkerboard on
% it, save as "cam1_light1.bmp" etc. in a subfolder "lights" 
% 
% All real-world units in mm
%
% The common world-centered frame of reference uses a right-handed 
% Cartesian coordinate system with its origin located at the nodal point of Camera 1.
% The x- and y-axis of the tracker’s coordinate system are parallel to the image plane of Camera 1, and the
% positive zaxis pointed away from it (i.e., toward the eye). 
%
% Protocol by Barsingerhorn, Boonstra & Goossens 2018

p_cam1 = [0 0 0];
p_cam2 = -stereo_params.TranslationOfCamera2*stereo_params.RotationOfCamera2';

% Images with the light source visible as a point of light
file_cam1_light1 = fullfile(fileroot,'lights','cam1_light1.bmp');
file_cam2_light1 = fullfile(fileroot, 'lights','cam2_light1.bmp');
file_cam1_light2 = fullfile(fileroot, 'lights','cam1_light2.bmp');
file_cam2_light2 = fullfile(fileroot, 'lights','cam2_light2.bmp');

% Imagesc with same exact position, with exposure turned up for checkerboard 
file_cam1_light1_check = fullfile(fileroot,'lights','cam1_light1_checkerboard.bmp');
file_cam2_light1_check = fullfile(fileroot, 'lights','cam2_light1_checkerboard.bmp');
file_cam1_light2_check = fullfile(fileroot, 'lights','cam1_light2_checkerboard.bmp');
file_cam2_light2_check = fullfile(fileroot, 'lights','cam2_light2_checkerboard.bmp');
% square_size_mirror = 1; % (mm) Doesn't matter! Just needs to be in a plane

%%
[u_mirror_cam1_light1, maskC1L1] = getChecks(file_cam1_light1_check);
[u_mirror_cam2_light1, maskC2L1] = getChecks(file_cam2_light1_check);
[u_mirror_cam1_light2, maskC1L2] = getChecks(file_cam1_light2_check);
[u_mirror_cam2_light2, maskC2L2] = getChecks(file_cam2_light2_check);


%% Detect lights. New method: intensity turned down so a single spot of light is visible, detect the center of it
I_cam1_light1 = imread(file_cam1_light1);
I_cam2_light1 = imread(file_cam2_light1);
I_cam1_light2 = imread(file_cam1_light2);
I_cam2_light2 = imread(file_cam2_light2);

temp = regionprops(I_cam1_light1==255);
[~, idx] = max([temp.Area]);
u_cam1_light1 = temp(idx).Centroid;

temp = regionprops(I_cam2_light1==255);
[~, idx] = max([temp.Area]);
u_cam2_light1 = temp(idx).Centroid;

temp = regionprops(I_cam1_light2==255);
[~, idx] = max([temp.Area]);
u_cam1_light2 = temp(idx).Centroid;

temp = regionprops(I_cam2_light2==255);
[~, idx] = max([temp.Area]);
u_cam2_light2 = temp(idx).Centroid;


% Plot results for each image
figure; %maximize;
subplot(2,4,1);
imshow(I_cam1_light1); hold on
plot(u_cam1_light1(1), u_cam1_light1(2),'+r')
title('light 1 in camera 1')

subplot(2,4,2);
J = insertText(imread(file_cam1_light1_check),u_mirror_cam1_light1(maskC1L1,:),find(maskC1L1));
imshow(J)
hold on; plot(u_mirror_cam1_light1(:,1), u_mirror_cam1_light1(:,2),'go')

subplot(2,4,3); 
imshow(I_cam2_light1); hold on
plot(u_cam2_light1(1), u_cam2_light1(2),'+r')
title('light 1 in camera 2')

subplot(2,4,4);
J = insertText(imread(file_cam2_light1_check),u_mirror_cam2_light1(maskC2L1,:),find(maskC2L1));
imshow(J)
hold on; plot(u_mirror_cam2_light1(:,1), u_mirror_cam2_light1(:,2),'go')

subplot(2,4,5); 
imshow(I_cam1_light2); hold on
plot(u_cam1_light2(1) ,u_cam1_light2(2),'+r')
title('light 2 in camera 1')

subplot(2,4,6)
J = insertText(imread(file_cam1_light2_check),u_mirror_cam1_light2(maskC1L2,:),find(maskC1L2));
imshow(J)
hold on; plot(u_mirror_cam1_light2(:,1), u_mirror_cam1_light2(:,2),'go')

subplot(2,4,7)
imshow(I_cam2_light2); hold on
plot(u_cam2_light2(1) ,u_cam2_light2(2),'+r')
title('light 2 in camera 2')

subplot(2,4,8); 
J = insertText(imread(file_cam2_light2_check),u_mirror_cam2_light2(maskC2L2,:),find(maskC2L2));
imshow(J)
hold on; plot(u_mirror_cam2_light2(:,1), u_mirror_cam2_light2(:,2),'go')

%% Undistort and project checkerboard points to world coordinates and find mirror plane 

u_mirror_cam1_light1_undistort(maskC1L1,:) = undistortPoints(u_mirror_cam1_light1(maskC1L1,:), stereo_params.CameraParameters1);
u_mirror_cam2_light1_undistort(maskC2L1,:) = undistortPoints(u_mirror_cam2_light1(maskC2L1,:), stereo_params.CameraParameters2);
p_mirror_light1 = triangulate(u_mirror_cam1_light1_undistort(maskC1L1&maskC2L1,:), u_mirror_cam2_light1_undistort(maskC1L1&maskC2L1,:), stereo_params); % in mm
[n_mirror_light1,V_mirror_light1,p0_mirror_light1_offset] = affine_fit(p_mirror_light1);

u_mirror_cam1_light2_undistort(maskC1L2,:) = undistortPoints(u_mirror_cam1_light2(maskC1L2,:), stereo_params.CameraParameters1);
u_mirror_cam2_light2_undistort(maskC2L2,:) = undistortPoints(u_mirror_cam2_light2(maskC2L2,:), stereo_params.CameraParameters2);
p_mirror_light2 = triangulate(u_mirror_cam1_light2_undistort(maskC1L2&maskC2L2,:), u_mirror_cam2_light2_undistort(maskC1L2&maskC2L2,:), stereo_params); % in mm
[n_mirror_light2,V_mirror_light2,p0_mirror_light2_offset] = affine_fit(p_mirror_light2);

% Add the thickness of the mirror to the get the actual mirror plane
% ****
mirror_thickness = -0.1; % (mm) Measure thickness of mirror + checkerboard sticker -- NEW thorlabs silver front-coated mirror!
% mirror_thickness = -2.86; % (mm) Measure thickness of mirror + checkerboard sticker - old back-coated acrylic
p0_mirror_light1 = p0_mirror_light1_offset + mirror_thickness*n_mirror_light1';
p0_mirror_light2 = p0_mirror_light2_offset + mirror_thickness*n_mirror_light2';


% Undistort and project light virtual image to world coordinates
u_light1_cam1_undistort = undistortPoints(u_cam1_light1, stereo_params.CameraParameters1);
u_light1_cam2_undistort = undistortPoints(u_cam2_light1, stereo_params.CameraParameters2);
p_virtual_light1 = triangulate(u_light1_cam1_undistort, u_light1_cam2_undistort, stereo_params); % in mm

u_light2_cam1_undistort = undistortPoints(u_cam1_light2, stereo_params.CameraParameters1);
u_light2_cam2_undistort = undistortPoints(u_cam2_light2, stereo_params.CameraParameters2);
p_virtual_light2 = triangulate(u_light2_cam1_undistort, u_light2_cam2_undistort, stereo_params); % in mm


% Calculate position of light in reality given virtual image in mirror

% Find the distance to the plane from the virtual IRLED point
t1 = (p0_mirror_light1 - p_virtual_light1)/n_mirror_light1';
t2 = (p0_mirror_light2 - p_virtual_light2)/n_mirror_light2';
p_light1 = p_virtual_light1 + t1*2*n_mirror_light1(:)';
p_light2 = p_virtual_light2 + t2*2*n_mirror_light2(:)';

% Show 
figure;  
hp = pcshow([p_cam1 ; p_cam2],'c','MarkerSize',1000); hold on; axis vis3d % Plot camera nodal points
hp(2) = pcshow(p_light1,'r','MarkerSize',1000);                            % Plot light image in mirror
hp(3) = pcshow(p_light2,'m','MarkerSize',1000);                            % Plot light image in mirror

pcshow(p_mirror_light1,'r','MarkerSize',100);                       % Plot mirror points
pcshow(p_mirror_light2,'m','MarkerSize',100);                       % Plot mirror points
pcshow(p_virtual_light1,'r','MarkerSize',400)                            % Plot light image in mirror
pcshow(p_virtual_light2,'m','MarkerSize',400)                            % Plot light image in mirror
view([0 -1 0])
xlim(xlim+[-20 20])
zlim(zlim+[-20 20])
xlabel('X')
ylabel('Y')
zlabel('Z')

% campos([-16.2859 -166.1160 -603.8011])
% legend(hp , {'Cameras','light1','light2'})

C = [];
C.stereo_params = stereo_params;
C.p_light1 = p_light1;
C.p_light2 = p_light2;
C.p_cam1 = p_cam1;
C.p_cam2 = p_cam2;


end

function [u, mask_mirror] = getChecks(filepathname)
% Detect checkerboard in image coordinates

% Load and adjust image
I = imread(filepathname);
% I = imlocalbrighten(I);
I = localcontrast(I);

% Detect points
MinCornerMetric = 0.12;% default 0.15, decrease to get more corner detections
[u, board_size] = detectCheckerboardPoints(I, 'PartialDetections',true,'MinCornerMetric',MinCornerMetric);

% Reshape x and y
ux = reshape(u(:,1), board_size-1);
uy = reshape(u(:,2), board_size-1);

% Standardize the orientation
if nanmean(u(1:(board_size(1)-1):end,2)) > nanmean(u(board_size-1:(board_size(1)-1):end,2)) 
    ux = flipud(ux);
    uy = flipud(uy);
end
if nanmean(u(1:(board_size(1)-1),1)) > nanmean(u(end-board_size(1)+2:end,1)) 
    ux = fliplr(ux);
    uy = fliplr(uy);
end

u = [ux(:) uy(:)];

% Output mask for NaNs as well1
mask_mirror = ~isnan(u(:,1));

% 
% %%
% I_cam1_light1 = imread(file_cam1_light1_check);
% I_cam2_light1 = imread(file_cam2_light1_check);
% I_cam1_light2 = imread(file_cam1_light2_check);
% I_cam2_light2 = imread(file_cam2_light2_check);
% 
% I_cam1_light1 = imlocalbrighten(I_cam1_light1);
% I_cam2_light1 = imlocalbrighten(I_cam2_light1);
% I_cam1_light2 = imlocalbrighten(I_cam1_light2);
% I_cam2_light2 = imlocalbrighten(I_cam2_light2);
% 
% I_cam1_light1 = localcontrast(I_cam1_light1);
% I_cam2_light1 = localcontrast(I_cam2_light1);
% I_cam1_light2 = localcontrast(I_cam1_light2);
% I_cam2_light2 = localcontrast(I_cam2_light2);
% 
% u_mirror_cam1_light1 = detectCheckerboardPoints(I_cam1_light1, 'PartialDetections',true);
% u_mirror_cam2_light1 = detectCheckerboardPoints(I_cam2_light1, 'PartialDetections',true);
% u_mirror_cam1_light2 = detectCheckerboardPoints(I_cam1_light2, 'PartialDetections',true);
% u_mirror_cam2_light2 = detectCheckerboardPoints(I_cam2_light2, 'PartialDetections',true);
% 
% 
% 
% maskC1L1 = ~isnan(u_mirror_cam1_light1(:,1));
% maskC2L1 = ~isnan(u_mirror_cam2_light1(:,1));
% maskC1L2 = ~isnan(u_mirror_cam1_light2(:,1));
% maskC2L2 = ~isnan(u_mirror_cam2_light2(:,1));


end