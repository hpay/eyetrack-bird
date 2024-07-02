function hs = plotHeadEyeSpeed(t_eye, v_head, v_eye_local, mask_eye)
% EXPERIMENTAL, needs close inspection
% Plot histograms of angular speed comparing jumps in:
% 1. head angle relative to the world
% 2. eye angle relative to the head reference frame (defined by the a line connecting mean of eye position


cc = 'c';

% Mask out the wrong eye/bad data
v_eye_local(~mask_eye,:) = NaN;
v_head(~mask_eye,:) = NaN;

dt = mean(diff(t_eye));

p = regionprops(~mask_eye,'Area','PixelList');

maxnan = 0.4/dt; % 0.1 s max gap to measure over

inds_remove = [];
for ii = 1:length(p)
   if p(ii).Area  < maxnan
    inds = p(ii).PixelList(:,2);
    inds_remove = [inds_remove; inds];
   end
end
v_eye_local_trim = v_eye_local;
v_head_trim = v_head;
v_eye_local_trim(inds_remove,:) = [];
v_head_trim(inds_remove,:) = [];


head_speed = sqrt(sum(diff(v_head_trim,[],1).^2,2))/dt;
eye_speed = sqrt(sum(diff(v_eye_local_trim,[],1).^2, 2))/dt;

%%

figure; plot(head_speed); hold on; plot(eye_speed)

figure;

bin_edges = 0:.5:20;
histogram(head_speed, bin_edges); hold on
histogram(eye_speed, bin_edges)
set(gca,'YScale','log')