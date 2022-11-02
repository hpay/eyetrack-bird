% Load close-up video of free moving bird eyes

function trackEyeVideoKinovea(filepath, video_filename, kinovea_filename)
% 
% filepath = 'Z:\Hannah\ephys\HC1_181011b';
% video_filename = 'video.avi';
% kinovea_filename = 'video.txt';
% filename_qtm = 'HC1_181011_0001';
% 
% Saves result in filepath as "eye.mat"

addpath(fullfile(fileparts(mfilename('fullpath')), 'helper'));

close all

% eye tracking parameters
p.radiiPupil  = 20; % (pixels)
p.radiiCR = 4;
p.CRthresh = 10;        % (default 10) Threshold for subtracting background (default 10)
p.CRfilter = 3;         % (default 8, minimum 3). To detect imperfect circles,
                        % radius of the filter needs to be set larger.
p.removeCRwidth = 3;    % (pixels) width to remove around bright spots
p.removeCRthresh = .5;  % (threshold) fraction of bright spots intensity to remove

p.minfeatures = .5;     % (fraction, [0 1]) min features required for starburst
p.smoothSigma = 1;      % (pixels) smooth image
plot_on = 1;
debug_on = 0;
edge_thresh = 40;       % Increase if pupil fit is too small, decrease to run faster

% Load the high res video
vid = VideoReader(fullfile(filepath, video_filename ));

% In each frame, find the pupil center
ii = 0;
pupilStart = [];
plot_handles = [];
plot_handles2 = [];


% Load kinovea tracking result. Download free Kinovea software and roughly
% track pupil.
% ***This depends on the coordinates in kinovea being relative to the top 
% left corner -- drag the first frame there to approximate***
K = readtable(fullfile(filepath, kinovea_filename));
K_x = K{:,2};
K_y = K{:,3};
K_y = vid.Height - K_y; % Flipped for some reason
K_width_height = [400 200]; % (pixels) size of ROI

% Initialize eye results
nt = round(vid.Duration*vid.FrameRate);
E.pupil1 = NaN(nt,5);
E.cr1a = NaN(nt,3);
E.cr1b = NaN(nt,3);
E.sync1 = NaN(nt,1);

% For plotting
a = linspace(0,2*pi,40);

figure;
while hasFrame(vid)
        I = rgb2gray(readFrame(vid));
        ii = ii+1;

    
    % Use kinovea. 
    start_x = max(1,K_x(ii)-K_width_height(1)/2);
    stop_x = min(size(I, 2), K_x(ii)+K_width_height(1)/2);
    start_y = max(1,K_y(ii)-K_width_height(2)/2); % rows = y
    stop_y = min(size(I, 1), K_y(ii) + K_width_height(2)/2);
    I_crop  = I(start_y:stop_y, start_x:stop_x); % cols = x
    
    try
        [pupil, cra, crb, ~, edge_thresh, E.sync1(ii), plot_handles] = ...
            detectPupilCR(I_crop, edge_thresh+4, pupilStart, p, 'PlotOn', plot_on, 'DebugOn',debug_on, 'PlotHandles',plot_handles);
        
        E.pupil1(ii,:) = pupil + [start_x start_y 0 0 0];
        E.cr1a(ii,:) = cra + [start_x start_y 0 0 0];
        E.cr1b(ii,:) = crb + [start_x start_y 0 0 0];
        
        
        figure(2);
        plot_handles2.img = imagesc(I);  colormap(gray);   axis image
        hold on
        
        % Plot ellipse        
        plot_handles2.pupil_line = line(E.pupil1(ii,3)*cos(a)*cos(E.pupil1(ii,5)) - sin(E.pupil1(ii,5))*E.pupil1(ii,4)*sin(a) + E.pupil1(ii,1), ...
            E.pupil1(ii,3)*cos(a)*sin(E.pupil1(ii,5)) + cos(E.pupil1(ii,5))*E.pupil1(ii,4)*sin(a) + E.pupil1(ii,2),...
            'Color','y');
        
        plot_handles2.pupil_center = plot(E.pupil1(ii,1), E.pupil1(ii,2),'+y','LineWidth',2, 'MarkerSize',6);
                
    catch msg
        
    end
    
end

save(fullfile(filepath,'eye.mat'),'E')

