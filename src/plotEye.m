function plotEye(E, flag_sutract_mean)
% plotEye(E, flag_sutract_mean)
% Plot detect position in each camera image for pupil, CR, etc.

if ~exist('flag_sutract_mean','var')
    flag_sutract_mean = 0;
end
meanx = 0;
meany = 0;
hh = [];
hh(1).Color = 'r';
hh(2).Color = 'g';
hh(3).Color = 'b';

if 1
markerstyle_p = '.'; 
markerstyle_cra = '+'; 
markerstyle_crb = 'square'; 
else
markerstyle_p = ''; 
markerstyle_cra = ''; 
markerstyle_crb = ''; 
end
    
g = .5*[1 1 1];
figure
hs = subplot(2,1,1);
if flag_sutract_mean
    meanx = nanmean(E.pupil1(:,1));
    meany = nanmean(E.pupil1(:,2));
end
hp = plot(E.t, E.pupil1(:,1)-meanx,'Color',hh(1).Color,'Marker',markerstyle_p); hold on
hp(2) = plot(E.t, E.pupil1(:,2)-meany,'Color',hh(2).Color,'Marker',markerstyle_p); hold on
hp(3) = plot(E.t, E.cr1(:,1,1)-meanx,'k','LineWidth',.5,'Marker',markerstyle_cra); hold on
hp(4) = plot(E.t, E.cr1(:,2,1)-meany,'Color',g,'LineWidth',.5,'Marker',markerstyle_cra); hold on
hp(5) = plot(E.t, E.cr1(:,1,2)-meanx,'Color','k','LineWidth',.5,'Marker',markerstyle_crb);
hp(6) = plot(E.t, E.cr1(:,2,2)-meany,'Color',g,'LineWidth',.5,'Marker',markerstyle_crb);
uistack(hp(3:end),'bottom')
legend(hp([1 2 3 4 5 6]), {'pupilx','pupily','CRa x','CRa y','CRb x','CRb y'});
title('Eye position, camera 1 (pixels)')
ylabel('pixels')

hs(2) = subplot(2,1,2);
if flag_sutract_mean
    meanx = nanmean(E.pupil2(:,1));
    meany = nanmean(E.pupil2(:,2));
end
plot(E.t, E.pupil2(:,1) - meanx,'Color',hh(1).Color,'Marker',markerstyle_p); hold on
plot(E.t, E.pupil2(:,2) - meany,'Color',hh(2).Color,'Marker',markerstyle_p); hold on
hp(3) = plot(E.t, E.cr2(:,1,1)-meanx,'k','LineWidth',.5,'Marker',markerstyle_cra); hold on
hp(4) = plot(E.t, E.cr2(:,2,1)-meany,'Color',g,'LineWidth',.5,'Marker',markerstyle_cra); hold on
hp(5) = plot(E.t, E.cr2(:,1,2)-meanx,'Color','k','LineWidth',.5,'Marker',markerstyle_crb);
hp(6) = plot(E.t, E.cr2(:,2,2)-meany,'Color',g,'LineWidth',.5,'Marker',markerstyle_crb);
title('Eye position, camera 2 (pixels)')
ylabel('pixels')
xlabel('Time (s)')

linkaxes(hs, 'x')
drawnow