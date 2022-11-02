function [c,p] = estimatepandc(params,light1,light2,v1,v2,u1a,u2a,u1b,u2b)
% [snorm,c,p] = estimatepandc(params,light1,light2,v1,v2,u1a,u2a,u1b,u2b)
% Estimate the 3D positions of the center of the virtual pupil and center
% of corneal curvature. See Barsingerhorn, Boonstra & Goossens (2018),
% Behav Res Meth
% Annemiek Barsingerhorn
% 
% Based on Guestrin and Eizenman (2008)
% And on Novel Eye Gaze Tracking Techniques Under Natural Head Movement
% Zhiwei Zhu and Qiang Ji 2005
% Modified by Hannah Payne 2022
%
% params is a stereoParameters object
% l1 = location of light source 1 (relative to nodal point cam1)
% l2 = location of light source 2
% v1 = image pupil center in cam 1 (all points are [x y])
% v2 = image pupil center in cam 2
% u1a= image glint a in cam 1
% u2a= image glint a in cam 2
% u1b= image glint b in cam 1
% u2b= image glint b in cam 2
% c = center of corneal curvature
% p = pupil center
% snorm = unit vector from c to p

% NOTE: 3/9/2022 corrected (?) CameraParameters1 and 2, which were swapped
% in the original version!!!

% pupil world coordinates
p = triangulate(undistortPoints(v1,params.CameraParameters1),...
    undistortPoints(v2,params.CameraParameters2),params);

% glint world coordinates
glint1 = triangulate(undistortPoints(u1a,params.CameraParameters1),...
    undistortPoints(u2a,params.CameraParameters2),params);
glint2 = triangulate(undistortPoints(u1b,params.CameraParameters1),...
    undistortPoints(u2b,params.CameraParameters2),params);

% Find the closest point to the lines defined by the two CRs
A1 = glint1;
A2 = light1;
B1 = glint2;
B2 = light2;
nA = dot(cross(B2-B1,A1-B1)',cross(A2-A1,B2-B1)')';
nB = dot(cross(A2-A1,A1-B1)',cross(A2-A1,B2-B1)')';
d = dot(cross(A2-A1,B2-B1)',cross(A2-A1,B2-B1)')';
A0 = A1 + bsxfun(@times, nA./d, A2-A1);
B0 = B1 + bsxfun(@times, nB./d, B2-B1);
c = (A0+B0)/2; 

