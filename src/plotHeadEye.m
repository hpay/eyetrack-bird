function hs = plotHeadEye(t_head, R_head, t_eye, v_eye_world, v_eye_local, mask_eye)

% Plot horizontal:
% 1. head angle relative to the world
% 2. eye angle relative to the world
% 3. eye angle relative to the head reference frame (defined by the a line connecting mean of eye position
% to beak tip, and line connecting two eyes.)


cc = 'c';
markerstyle_p = '.';

% Mask out the wrong eye/bad data
v_eye_world(~mask_eye,:) = NaN;
v_eye_local(~mask_eye,:) = NaN;

% [ang_speed, ang_acc] = getAngSpeed(R_head, fps, 1, 1);
[head_yaw, ~, ~] = getYawPitchRollDeg(R_head);
eye_world_yaw = squeeze(atan2d(v_eye_world(:,2), v_eye_world(:,1)));
eye_local_yaw = squeeze(atan2d(v_eye_local(:,2), v_eye_local(:,1)));


figure;
hs = subplot(3,1,1);
hp = plot(t_head, head_yaw,'.-k'); hold on
ylabel('Head')

hs(2) = subplot(3,1,2);
hp = plot(t_eye, eye_world_yaw,markerstyle_p,'Color',cc); hold on
ylabel('Eye in world')

hs(3) = subplot(3,1,3);
hp = plot(t_eye, eye_local_yaw,markerstyle_p,'Color',cc); hold on
ylabel('Eye in head')
linkaxes(hs,'x');

set(hs(1:2),'XTick',[],'XColor','none')
fixticks

% Fill in NaNs?
% hp = plot(hs(2), t_eye, inpaint_nans(eye_world_yaw),'-','Color','c'); uistack(hp,'bottom')
% hp = plot(hs(3), t_eye, inpaint_nans(eye_local_yaw),'-','Color','c'); uistack(hp,'bottom')

