function S = detectHeadSaccadeRotm(R_head, tt, P, plot_on)
% P.a = 60;       % (deg/s) Min angular head speed at saccade peak and at end
% P.b = 0.5;      % (norm) Max normalized angular speed at start and end of saccade
% P.T = 0.5;      % (s) Exclude saccades within T seconds of beginning/end of file
% P.max_duration = 0.2;       % (s) Max saccade duration, only excludes a few saccades
% P.min_intersaccade = 0.15;   % (s) min peak distance
% % % P.tsmooth = 0.01; % (s) moving boxcar smoothing
% %P.Fpass = 30; % (Hz) 25
% %P.FN = 2;

tt = tt(:);
fps = 1/mean(diff(tt));

% Change in head angle (convert change in rotation matrixes to a single rotation
% about an arbitrary axis)
option_smooth = 1;
[ang_speed_smooth, ang_acc] = getAngSpeed(R_head, fps, option_smooth);

% Smooth head position 
% nanmask = ~isnan(head_xpos);
% smooth_xx = NaN(size(head_xpos));
% smooth_yy = NaN(size(head_ypos));
% smooth_xx(nanmask) = filtfilt(b,a,head_xpos(nanmask));
% smooth_yy(nanmask) = filtfilt(b,a,head_ypos(nanmask));

% Head linear speed
% head_linear_speed = [sqrt(diff(smooth_xx).^2+diff(smooth_yy).^2)*Fs; NaN];

% Initial thresholding for saccade detection
S = table;
[peak_ang_speed, raw_ind] = findpeaks(ang_speed_smooth,...
    'MinPeakHeight', P.a,'MinPeakDist',round(P.min_intersaccade*fps));

S.ind = raw_ind;
S.time = tt(S.ind);
S.angular_speed = peak_ang_speed;

% Remove any too close to ends (0.5 s)
mask = S.ind-P.T*fps<1 | S.ind+P.T*fps>length(tt);
S(mask,:) = [];

% Get the start and stop times for each saccade
zcross_plus = (find(diff(sign(ang_acc))>0)+1);
ntemp = length(zcross_plus);
start_ind = NaN(height(S),1);
stop_ind = NaN(height(S),1);
inds_temp = S.ind;
for ii = 1:length(S.time)  
    temp_ind = find(zcross_plus<inds_temp(ii),1,'last');
    if ~isempty(temp_ind) && temp_ind<ntemp    
    start_ind(ii) = zcross_plus(temp_ind)-1;
    stop_ind(ii) = zcross_plus(temp_ind+1);
    end
end
S.start_ind = start_ind;
S.stop_ind = stop_ind;
S(isnan(start_ind)|isnan(stop_ind),:) = [];
S.time_start = tt(S.start_ind);
S.time_stop = tt(S.stop_ind);

% Angular head speed at start and end of saccade
before_ang_speed = ang_speed_smooth(S.start_ind)./S.angular_speed;
after_ang_speed = ang_speed_smooth(S.stop_ind)./S.angular_speed;

mask_keep  = before_ang_speed < P.b & ...
        after_ang_speed < P.b;
             
S(~mask_keep,:) = [];

% Get velocities and distances
for ii = 1:height(S)
    Rd = R_head(:,:,S.stop_ind(ii))'*R_head(:,:,S.start_ind(ii));
    axang = rotm2axang(Rd);
    S.angular_dist(ii) = real(axang(4))*180/pi; % Convert to deg/s
end

% Check total duration
mask_duration = (S.time_stop-S.time_start) > P.max_duration ;%|... ;
%     (saccade.time_stop-saccade.time_start) < P.min_duration;
% fprintf('n = %i excluded by duration, %i excluded by distance\n',nnz(mask_duration), nnz(mask_distance));
fprintf('n = %i/%i excluded by duration\n',nnz(mask_duration),length(mask_duration));
S(mask_duration,:) = [];

% % Only include saccade with nice biphasic acceleration profiles?
% [~, acc_peaks] = findpeaks(head_angular_acc, 'MinPeakHeight', 3000);
% [~, acc_troughs] = findpeaks(-head_angular_acc, 'MinPeakHeight', 3000);
% mask_keep = false(height(saccade),1);
% for ii = 1:length(saccade.time)
%     if nnz(acc_peaks>=saccade.start_ind(ii) & acc_peaks<=saccade.ind(ii))==1 && nnz(acc_troughs>=saccade.ind(ii) & acc_troughs<=saccade.stop_ind(ii))==1
%         mask_keep(ii) = 1;
%     end
% end
% saccade(~mask_keep,:) = [];


%  Plot saccade detection
if exist('plot_on','var') && plot_on
    figure; 

          
    sh = subplot(2,1,1);
    plot(tt, ang_speed_smooth,'k'); hold on;
    plot(S.time, S.angular_speed, '+r','LineWidth',2); hold on
    plot(S.time_start, ang_speed_smooth(S.start_ind),'+g','LineWidth',2);
    plot(S.time_stop, ang_speed_smooth(S.stop_ind),'+b','LineWidth',2);
    xlabel('Time (s)'); ylabel('Angular speed (deg/s)')
    plot(xlim, [1 1]*P.a,'--c')
    plot(xlim, [1 1]*P.a*P.b,'--c')
    ylim([0 prctile(ang_speed_smooth,99.5)])
    grid on
    
    sh(2) = subplot(2,1,2);
    plot(tt, ang_acc,'k'); hold on;
    plot(S.time, ang_acc(S.ind), '+r','LineWidth',2); hold on
    plot(S.time_start, ang_acc(S.start_ind),'+g','LineWidth',2);
    plot(S.time_stop, ang_acc(S.stop_ind),'+b','LineWidth',2);
    vertical_cursors;
    xlabel('Time (s)'); ylabel('Angular accel (deg/s2)')
        grid on

    
    linkaxes(sh,'x')
%     xlim([0 100]);
    
%     plotSaccadeTest(saccade)

end
