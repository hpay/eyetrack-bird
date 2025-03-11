function [p_pupil_common, p_cornea_common, p_beak_common, p_rigidbody_common, R_rigidbody_common]...
    = alignCommonReferenceFrame(A, p_pupil, p_cornea, p_beak, p_rigidbody, R_rigidbody)
% Calculate eye features and head features relative to common world
% reference frame defined by the triangular axis with three markers
% (rotated down by 45 so the floor is flat)

% convert from camera coords to common coords
p_pupil_common = (p_pupil - A.p0_eye)*A.R_eye*A.R_45; % A.p0_eye refers to axes origin measured by dual cameras (calibrateAxesBW)
p_cornea_common = (p_cornea - A.p0_eye)*A.R_eye*A.R_45;

if ~isempty(p_beak)
    p_beak_common = (p_beak - A.p0_eye)*A.R_eye*A.R_45;
else
    p_beak_common = [];
end

% convert from QTM coords to common coords
N_head = length(p_rigidbody);
p_rigidbody_common = (p_rigidbody - A.p0_head)*A.R_head*A.R_45; % A.p0_head refers to axes origin measured by QTM (calibrateAxesBW)
R_rigidbody_common = NaN(size(R_rigidbody));
for jj = 1:N_head
    R_rigidbody_common(:,1,jj) = R_rigidbody(:,1,jj)'*A.R_head*A.R_45;
    R_rigidbody_common(:,2,jj) = R_rigidbody(:,2,jj)'*A.R_head*A.R_45;
    R_rigidbody_common(:,3,jj) = R_rigidbody(:,3,jj)'*A.R_head*A.R_45;
end

