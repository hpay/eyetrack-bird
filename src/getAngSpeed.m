function [ang_speed, ang_acc] = getAngSpeed(R, fps, option_smooth_vel, option_med_filt)
Fpass = 25;                % (Hz)    smoothing filter lowpass cutoff
FN = 2;                    % Order of the smoothing filter, if used


% if exist('option_med_filt','var') && option_med_filt
%     for ii = 1:3
%         for jj = 1:3
%             
%             R(ii,jj,:)  = medfilt1(R(ii,jj,:), 3);
%         end
%     end    
% end


ang_speed = NaN(size(R,3),1);
for ii = 1:size(R,3)-1
    Rd = R(:,:,ii)'*R(:,:,ii+1);
    axang = rotm2axang(Rd);
    ang_speed(ii) = fps*real(axang(4))*180/pi; % Convert to deg/s
end

if exist('option_med_filt','var') && option_med_filt
   
    ang_speed = medfilt1(ang_speed,3);
end


if exist('option_smooth_vel','var') && option_smooth_vel
    
    
    % Low-pass filtering of head angular/linear position
    [b,a] = butter(FN, Fpass/(fps/2),'low');
    nanmask = ~isnan(ang_speed);
    ang_speed_smooth = NaN(size(ang_speed));
    ang_speed_smooth(nanmask) = filtfilt(b,a,ang_speed(nanmask));
    
    ang_speed = ang_speed_smooth;
end

% Head angular acceleration
ang_acc = [NaN; diff(ang_speed)*fps]; % (DEG/s^2)
