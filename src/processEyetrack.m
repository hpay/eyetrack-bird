function [E, p_pupil, p_cornea, p_beak, p] = processEyetrack(filepath_eye,  camfilename, C,  resume_beak, run_beak, resume_eye)
% Process camera calibration, run eye tracking, and store
% head/eye/beak calibration for use in gaze analysis
%
% Returns the estimated 3d locations of pupil, center of corneal curvature,
% and beak (uncleaned)
%
% [C,A] = calibrateCameras(filepath_camera);
%
% Hannah Payne 2022


%% Get beak
beak = getBeak(filepath_eye, camfilename, resume_beak, run_beak);


%% Track eye
if ~exist(fullfile(filepath_eye, 'eye.mat'),'file')
    %%
    p = [];
    p.radiiPupil = 32;      % (pixels) 20
    p.radiiCR = [2 5];          % (pixels) [4 7]
    p.CRthresh = 12;        % (default 10) Threshold for subtracting background
    p.CRfilter = 6;         % (default 3, minimum 3). Set larger to detect imperfect circles - but sometimes too large throws an error esp with small radiiCR
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
    p.pupil_start = [NaN NaN];
    p(2) = p(1); % Same settings for the second camera
    
    p(1).pupil_intensity = 20; % Adjust based on images if needed % 30 % 100/130
    p(1).iris_intensity = 70; % Adjust based on images if needed %80
    p(2).pupil_intensity = 20; % Adjust based on images if needed % 30 % 90/120
    p(2).iris_intensity = 70;  % Adjust based on images if needed %80
    
    [E,p] = trackEye(filepath_eye, p, camfilename); % TRACK THE PUPIL AND CR
else
    temp = load(fullfile(filepath_eye,'eye.mat'));
    E = temp.E;
    p = temp.p;
    
    % To resume where we left off
    if isfield(temp,'ii') && resume_eye
        [E,p] = trackEye(filepath_eye, p, camfilename); % TRACK THE PUPIL AND CR
    end
end

N_eye = length(E.pupil1(:,1));


%% If beak is empty
if isempty(beak)
    beak.ii = 1; % index into video
    beak.p1 = NaN(N_eye,2);
    beak.p2 = NaN(N_eye,2);
end

%% Convert pupil and corneal reflections from image to camera world coordinates
if resume_eye || ~exist(fullfile(filepath_eye,'eye_triangulate.mat'),'file')
    p_cornea = NaN(N_eye, 3);
    p_pupil = NaN(N_eye, 3);
    nanmask = ~isnan(E.pupil1(:,1)) & ~isnan(E.cr1(:,1,2)) & ~isnan(E.pupil2(:,1)) & ~isnan(E.cr2(:,1,2));
    [p_cornea(nanmask,:), p_pupil(nanmask,:)] = estimatepandc(...
        C.stereo_params, C.p_light1, C.p_light2,...
        E.pupil1(nanmask,1:2),E.pupil2(nanmask,1:2),...
        E.cr1(nanmask,1:2,1),E.cr2(nanmask,1:2,1),...
        E.cr1(nanmask,1:2,2),E.cr2(nanmask,1:2,2));
    
    p_beak = NaN(N_eye, 3);
    if any(~isnan(beak.p1(:,1)))
        [p_beak(~isnan(beak.p1(:,1)),:), reprojectionErrors] = triangulate(...
            undistortPoints(beak.p1(~isnan(beak.p1(:,1)),:),...
            C.stereo_params.CameraParameters1), ...
            undistortPoints(beak.p2(~isnan(beak.p2(:,1)),:), ...
            C.stereo_params.CameraParameters2),C.stereo_params);
    end
    save(fullfile(filepath_eye,'eye_triangulate.mat'),'p_pupil','p_cornea','p_beak')
else
    temp = load(fullfile(filepath_eye,'eye_triangulate.mat'),'p_pupil','p_cornea','p_beak');
    p_pupil = temp.p_pupil;
    p_cornea= temp.p_cornea;
    p_beak = temp.p_beak;
    
end
