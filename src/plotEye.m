
function plotEye(E, flag_sutract_mean)

if ~exist('flag_sutract_mean','var')
    flag_sutract_mean = 0;
end
meanx = 0;
meany = 0;
hh = [];
hh(1).Color = 'r';
hh(2).Color = 'g';
hh(3).Color = 'b';


figure
hs = subplot(2,1,1);
if flag_sutract_mean
    meanx = nanmean(E.pupil1(:,1));
    meany = nanmean(E.pupil1(:,2));
end
hp = plot(E.t, E.pupil1(:,1)-meanx,'Color',hh(1).Color); hold on
hp(2) = plot(E.t, E.pupil1(:,2)-meany,'Color',hh(2).Color); hold on
hp(3:4) = plot(E.t, bsxfun(@minus, E.cr1(:,1:2,1), [meanx meany]),'k','LineWidth',.5); hold on
hp(5:6) = plot(E.t, bsxfun(@minus,E.cr1(:,1:2,2), [meanx meany]),'Color',.5*[1 1 1],'LineWidth',.5);
uistack(hp(3:end),'bottom')
legend(hp([1 2 3 5]), {'pupilx','pupily','CRa x and y','CRb x and y'});
title('Eye position, camera 1 (pixels)')
ylabel('pixels')

hs(2) = subplot(2,1,2);
if flag_sutract_mean
    meanx = nanmean(E.pupil2(:,1));
    meany = nanmean(E.pupil2(:,2));
end
plot(E.t,  bsxfun(@minus, E.cr2(:,1:2,1), [meanx meany]),'k','LineWidth',.5); hold on
plot(E.t,  bsxfun(@minus, E.cr2(:,1:2,2), [meanx meany]),'Color',.5*[1 1 1],'LineWidth',.5);
plot(E.t, E.pupil2(:,1) - meanx,'Color',hh(1).Color); hold on
plot(E.t, E.pupil2(:,2) - meany,'Color',hh(2).Color); hold on
title('Eye position, camera 2 (pixels)')
ylabel('pixels')
xlabel('Time (s)')

linkaxes(hs, 'x')
drawnow