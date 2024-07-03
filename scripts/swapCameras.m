%% USE CAREFULLY!
% Swap cameras if they were improperly labelled during calibration. cam1
% should be on the left, cam2 on the right
load('eye.mat')
E2 = E;
E2.pupil1 = E.pupil2;
E2.pupil2 = E.pupil1;
E2.cr1 = E.cr2;
E2.cr2 = E.cr1;
E2.resid1 = E.resid2;
E2.resid2 = E.resid1;
E2.points_fraction1 = E.points_fraction2;
E2.points_fraction2 = E.points_fraction1;
E = E2;
save('eye.mat','p','E')


%% beak
load('beak.mat')
p1_actual = p2;
p2 = p1;
p1 = p1_actual;
save('beak.mat','p1','p2','ii')