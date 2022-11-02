function S = detectHeadSaccade(v_head, tt, P, plot_on)
% P.a = 60;       % (deg/s) Min angular head speed at saccade peak and at end
% P.b = 0.5;      % (norm) Max normalized angular speed at start and end of saccade
% P.T = 0.5;      % (s) Exclude saccades within T seconds of beginning/end of file
% P.max_duration = 0.2;       % (s) Max saccade duration, only excludes a few saccades
% P.min_intersaccade = 0.15;   % (s) min peak distance
% P.Fpass = 20; % (Hz) 25
% P.FN = 2;

% Head angular position
head_hang = atan2d(v_head(:,2), v_head(:,1));  % (DEG) head horizontal angular position (azimuth)
head_hang = unwrap(head_hang);
head_vang = 90-acosd(v_head(:,3));             % (DEG) head vertical angular position (elevation - distance from horizontal)
        
tt = tt(:);

% Low-pass filtering of head angular/linear position
Fs = 1/mean(diff(tt));
[b,a] = butter(P.FN, P.Fpass/(Fs/2),'low');

% Smooth head angle vector
for ii = 1:3    
    nanmask = ~isnan(v_head(:,ii));
    v_head(nanmask,ii) = filtfilt(b,a,v_head(nanmask,ii));
end

% Head angular speed
head_angular_speed = real([acosd( dot(v_head(1:end-1,:), v_head(2:end,:), 2) ./...
    sqrt(sum(v_head(1:end-1,:).^2,2).*sum(v_head(2:end,:).^2,2)))*Fs; NaN]); % (DEG/s) (Normalization avoids imaginary numbers due to floating point error)

% Head angular acceleration
head_angular_acc = [NaN; diff(head_angular_speed)*Fs]; % (DEG/s^2)

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
[peak_ang_speed, raw_ind] = findpeaks(head_angular_speed,...
    'MinPeakHeight', P.a,'MinPeakDist',round(P.min_intersaccade*Fs));

S.ind = raw_ind;
S.time = tt(S.ind);
S.angular_speed = peak_ang_speed;

% Remove any too close to ends (0.5 s)
mask = S.ind-P.T*Fs<1 | S.ind+P.T*Fs>length(tt);
S(mask,:) = [];

% Get the start and stop times for each saccade
zcross_plus = (find(diff(sign(head_angular_acc))>0)+1);
ntemp = length(zcross_plus);
ind_start = NaN(height(S),1);
ind_stop = NaN(height(S),1);
inds_temp = S.ind;
for ii = 1:length(S.time)  
    temp_ind = find(zcross_plus<inds_temp(ii),1,'last');
    if ~isempty(temp_ind) && temp_ind<ntemp    
    ind_start(ii) = zcross_plus(temp_ind)-1;
    ind_stop(ii) = zcross_plus(temp_ind+1);
    end
end
S.ind_start = ind_start;
S.ind_stop = ind_stop;
S(isnan(ind_start)|isnan(ind_stop),:) = [];
S.time_start = tt(S.ind_start);
S.time_stop = tt(S.ind_stop);

% Angular head speed at start and end of saccade
before_ang_speed = head_angular_speed(S.ind_start)./S.angular_speed;
after_ang_speed = head_angular_speed(S.ind_stop)./S.angular_speed;

mask_keep  = before_ang_speed < P.b & ...
        after_ang_speed < P.b;
             
S(~mask_keep,:) = [];

% Get velocities and distances
theta = head_hang;
phi = -(head_vang-90); % Convert from elevation back to standard definition of phi
a = [sind(phi).*cosd(theta), sind(phi).*sind(theta), cosd(phi)];
S.angular_dist = real(acosd(sum(a(S.ind_start,:) .* a(S.ind_stop,:), 2)));

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
%     if nnz(acc_peaks>=saccade.ind_start(ii) & acc_peaks<=saccade.ind(ii))==1 && nnz(acc_troughs>=saccade.ind(ii) & acc_troughs<=saccade.ind_stop(ii))==1
%         mask_keep(ii) = 1;
%     end
% end
% saccade(~mask_keep,:) = [];


%  Plot saccade detection
if exist('plot_on','var') && plot_on
    figure; 
       sh = subplot(4,1,1);
    plot(tt, head_hang,'k'); hold on;
    plot(S.time, head_hang(S.ind), '+r','LineWidth',2); hold on
    plot(S.time_start, head_hang(S.ind_start),'+g','LineWidth',2);
    plot(S.time_stop, head_hang(S.ind_stop),'+b','LineWidth',2);
    xlabel('Time (s)'); ylabel('Horiz. head ang.')
          grid on
          
        sh(2) = subplot(4,1,2);
    plot(tt, head_vang,'k'); hold on;
    plot(S.time, head_vang(S.ind), '+r','LineWidth',2); hold on
    plot(S.time_start, head_vang(S.ind_start),'+g','LineWidth',2);
    plot(S.time_stop, head_vang(S.ind_stop),'+b','LineWidth',2);
    xlabel('Time (s)'); ylabel('Vert. head ang.')
          grid on
          
    sh(3) = subplot(4,1,3);
    plot(tt, head_angular_speed,'k'); hold on;
    plot(S.time, S.angular_speed, '+r','LineWidth',2); hold on
    plot(S.time_start, head_angular_speed(S.ind_start),'+g','LineWidth',2);
    plot(S.time_stop, head_angular_speed(S.ind_stop),'+b','LineWidth',2);
    xlabel('Time (s)'); ylabel('Angular speed (deg/s)')
    plot(xlim, [1 1]*P.a,'--c')
    plot(xlim, [1 1]*P.a*P.b,'--c')
    ylim([0 prctile(head_angular_speed,99.5)])
    grid on
    
    sh(4) = subplot(4,1,4);
    plot(tt, head_angular_acc,'k'); hold on;
    plot(S.time, head_angular_acc(S.ind), '+r','LineWidth',2); hold on
    plot(S.time_start, head_angular_acc(S.ind_start),'+g','LineWidth',2);
    plot(S.time_stop, head_angular_acc(S.ind_stop),'+b','LineWidth',2);
%     vertical_cursors;
    xlabel('Time (s)'); ylabel('Angular accel (deg/s2)')
        grid on

        
        
%     sh(3) = subplot(4,1,3);
%     plot(tt, head_linear_speed,'k'); hold on;
%     plot(saccade.time, head_linear_speed(saccade.ind), '+r','LineWidth',2); hold on
%     plot(saccade.time_start, head_linear_speed(saccade.ind_start),'+g','LineWidth',2);
%     plot(saccade.time_stop, head_linear_speed(saccade.ind_stop),'+b','LineWidth',2);
% %     vertical_cursors;
%     xlabel('Time (s)'); ylabel('Linear speed (mm/s)')
%           grid on
%  
%     sh(4) = subplot(4,1,4);
%     plot(tt, head_xpos,'k'); hold on;
%     plot(tt, head_ypos,'k'); hold on;
%     plot(saccade.time, head_xpos(saccade.ind), '+r','LineWidth',2); hold on
%     plot(saccade.time_start, head_xpos(saccade.ind_start),'+g','LineWidth',2);
%     plot(saccade.time_stop, head_xpos(saccade.ind_stop),'+b','LineWidth',2);
%     vertical_cursors;
    xlabel('Time (s)'); ylabel('Linear speed (mm/s)')
        grid on

    
    linkaxes(sh,'x')
%     xlim([0 100]);
    
%     plotSaccadeTest(saccade)

end

