function runEyetrackAll(filepath_eye_root, ploton)

% Find all folders with this root

% Loop eye tracking over all of them (just click on beak in the first)--
% assuming nothing changes between files!

%% May need to change these 
camfilename = 'cam%i*.avi'; % Filename template for eye cameras
calib_root = 'Z:\Hannah\eyetrack\calibration'; % Root dir for camera calibrations
downsample_eye = 6; % Set the downsampling rate at which the eye camera data was collected relative to QTM

data_root = fileparts(filepath_eye_root);
a = dir(filepath_eye_root);
folders = {a.name};
Q_all = [];
Q_all.N_head = 0;
Q_all.p_rigidbody  = [];
Q_all.R_rigidbody = [];
E_all = [];
E_all.pupil1 = [];
E_all.cr1 = [];
E_all.pupil2 = [];
E_all.cr2 = [];
E_all.resid1 = [];
E_all.resid2 = [];
E_all.points_fraction1 = [];
E_all.points_fraction2 = [];


p_pupil_all= [];
p_cornea_all = [];
p_beak_all =[];

for ii  = 1:length(folders)
    
    filepath_eye = fullfile(data_root, folders{ii});
    disp(filepath_eye)
    
    %% Get camera calibration -- folder must be named same date as recording, e.g. 220831
    temp = strfind(filepath_eye,'_');
    folder_camera_calib = filepath_eye(temp(end)+1:temp(end)+6);
    filepath_camera = fullfile(calib_root, folder_camera_calib);
    disp(filepath_camera)
    [C,A] = calibrateCameras(filepath_camera);
    
    %% Run eye tracking (will skip if already run)
    resume_beak = 0; % Change to 1 to add more beak points
    run_beak = 1;    
    if ii>1
        run_beak = 0;
    end
    recalculate = 0;
    [E, p_pupil, p_cornea, p_beak] = processEyetrack(filepath_eye, camfilename,  C, resume_beak, run_beak, recalculate);
    N_eye = length(E.resid1);
    
    E_all.pupil1 = [E_all.pupil1; E.pupil1];
    E_all.cr1 = cat(1,E_all.cr1, E.cr1);
    E_all.pupil2 = [E_all.pupil2; E.pupil2];
    E_all.cr2 = cat(1, E_all.cr2, E.cr2);
    E_all.resid1 = [E_all.resid1; E.resid1];
    E_all.resid2 = [E_all.resid2; E.resid2];
    try
    E_all.points_fraction1 = [E_all.points_fraction1; E.points_fraction1];
    E_all.points_fraction2 = [E_all.points_fraction2; E.points_fraction2];
    catch % Early recordings didn't caculate this                
    E_all.points_fraction1 = [E_all.points_fraction1; NaN(N_eye,1)];
    E_all.points_fraction2 = [E_all.points_fraction2; NaN(N_eye,1)];
    end
    p_pupil_all = [p_pupil_all; p_pupil];
    p_cornea_all = [p_cornea_all; p_cornea];
    p_beak_all = [p_beak_all; p_beak];
    
    %% Load Qualysis rigid body
    filename_qtm = 'qtm.mat'; % 6DOF
    Q = getfield(load(fullfile(filepath_eye,filename_qtm)),'qtm');
    N_head = Q.Frames;
    fprintf('Eye frames %i, predicted head frames %i, actual head frames %i\n', N_eye, N_eye*downsample_eye,N_head)

    % TODO!!!! Crop if longer than eye cameras
    if N_head  > N_eye*downsample_eye
        Q.RigidBodies.Positions = Q.RigidBodies.Positions(1,:,1:N_eye*downsample_eye);
        Q.RigidBodies.Rotations = Q.RigidBodies.Rotations(1,:,1:N_eye*downsample_eye);        
        N_head = N_eye*downsample_eye;
    end
    
    Q_raw.fps_head = Q.FrameRate;
    Q_raw.N_head =  N_head;
    Q_raw.p_rigidbody = squeeze(Q.RigidBodies.Positions(1,:,:))';
    Q_raw.R_rigidbody = reshape(Q.RigidBodies.Rotations(1,:,:), [3,3,N_head]); % [X Y Z]
    
    
    Q_all.fps_head = Q.FrameRate;
    Q_all.N_head = Q_all.N_head + Q_raw.N_head;
    Q_all.p_rigidbody = [Q_all.p_rigidbody; Q_raw.p_rigidbody];
    Q_all.R_rigidbody = cat(3, Q_all.R_rigidbody, Q_raw.R_rigidbody); % [X Y Z]
    
    % Results single
%     [H, Ht] = analyzeEyetrack(E, p_pupil, p_cornea, p_beak, Q_raw,  A, ploton);
%     disp(Ht)
end

%% Analyze results
a = strfind(filepath_eye_root,'_');
filepath_eye = [filepath_eye_root(1:a(end)-1) '_all']
[H, Ht, stats] = analyzeEyetrack(E_all, p_pupil_all, p_cornea_all, p_beak_all, Q_all,  A, ploton);
H.folder_eye_calib = folders;
H.folder_camera_calib = '';
Ht.folder = filepath_eye;
disp(Ht)
disp(stats)


%% Save results
mkdir(filepath_eye)
save(fullfile(filepath_eye, 'head_calibration.mat'),'-struct','H');
save(fullfile(filepath_eye, 'head_calibration_template.mat'),'Ht');
save(fullfile(filepath_eye, 'head_calibration_stats.mat'),'stats');

