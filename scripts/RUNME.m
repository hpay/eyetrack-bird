% pptDefaults
data_root = 'Z:\Hannah\eyetrack\behavior';

code_root = fileparts(which(mfilename));
addpath(fullfile(code_root,'src'));

%% Run a single eye tracking file
dir_root = 'HC08_221028b'; %IND102_220707a
ploton = 1;
runEyetrackSingle(fullfile(data_root, dir_root), ploton)

%% Run a set of eye tracking files for the same bird
dir_root = 'HC07_22*'; % Best example HC07?
runEyetrackAll(fullfile(data_root, dir_root), ploton)

%% Run analysis for all birds (to re-run raw eye tracking, delete the results file e.g. eye.mat and eye_trangulate.mat)
dir_roots = {'IND102_22*', 'HC06_22*','HC07_22*','HC08_22*','HC09_22*'} 
% NOTE: 'ROS38_220308*','TRQ180_220310*','CHC37_220310*', only had two head markers, and acrylic barrier - up through 220630, possibly less accurate?
% NOTE: IND102 was measured before and after change on several days
% NOTE: RBY47_221013 is the same bird as HC06_221013 before surgery, could
% use instead if needed

for ii = 1:length(dir_roots)
    runEyetrackAll(fullfile(data_root, dir_roots{ii}), ploton)   
end

%% Plot results & give stats for all birds
plotEyetrack(data_root, dir_roots)

