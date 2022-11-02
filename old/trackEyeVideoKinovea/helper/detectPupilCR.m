% DETECTPUPILCR detects pupil and corneal reflection in the eye image
% Starburst Algorithm
%
% This source code is part of the starburst algorithm.
% Starburst algorithm is free; you can redistribute it and/or modify
% it under the terms of the GNU General Public License as published by
% the Free Software Foundation; either version 2 of the License, or
% (at your option) any later version.
%
% Starburst algorithm is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU General Public License for more details.
%
% You should have received a copy of the GNU General Public License
% along with cvEyeTracker; if not, write to the Free Software
% Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
%
% Starburst Algorithm - Version 1.0.0
% Part of the openEyes ToolKit -- http://hcvl.hci.iastate.edu/openEyes
% Release Date:
% Authors : Dongheng Li <donghengli@gmail.com>
%           Derrick Parkhurst <derrick.parkhurst@hcvl.hci.iastate.edu>
% Copyright (c) 2005
% All Rights Reserved.
%
% Modified in 2013 by Hannah Payne <hpayne@stanford.edu>
% Raymond Lab, Stanford University 
%
%
% Input:
% img             = input image
% p.pupilStart    = start point for starburst algorithm for pupil
% p.radiiPupil    = guess of pupil radius
% p.edgeThresh    = threshold for detecting pupil
%
% p.radiiCR       = guess of CR radius
% p.CRthresh      = threshold for CR
% p.CRfilter      = filter size for CR
%
% Output:
% pupil_ellipse = 5-vector of the ellipse parameters of pupil
%   [cx cy  a b theta]
%   cx - the x coordinate of ellipse center
%   cy - the y coordinate of ellipse center
%   a - the ellipse axis of x direction
%   b - the ellipse axis of y direction
%   theta - the orientation of ellipse
% cr1 and cr2 = 3-vector of the circle parameters of the corneal reflection
%   [crx cry crr]
%   crx - the x coordinate of circle center
%   cry - the y coordinate of circle center
%   crr - the radius of circle
% points        = actual points on pupil detected
% edgeThresh    = actual edgeThresh used for pupil


function [pupil, cr1, cr2, points, edgeThresh, sync, plot_handles] = detectPupilCR(img, edgeThresh, pupilStart, p, varargin)


%% Process imputs
P = inputParser;
addParameter(P,'PlotOn',0);
addParameter(P,'DebugOn',0);
addParameter(P,'PlotHandles',[])
parse(P,varargin{:})
plot_on = P.Results.PlotOn;
debug_on = P.Results.DebugOn;
plot_handles = P.Results.PlotHandles;
img = single(img);

if numel( p.radiiCR)==1
     p.radiiCR =  round(p.radiiCR*[.8 1.2]);
end

if ~isfield(p,'smoothSigma')
     p.smoothSigma = 3; % pixels
end
if ~isfield(p,'minfeatures')
     p.minfeatures = .6; % fraction
end
if ~isfield(p,'removeCRthresh')
    p.removeCRthresh = .5;
end

%% Extract the synchronization signal from a flashing LED
sync = mean(mean(img(1:round(size(img,1)/2), round(size(img,2)*3/8):round(size(img,2)*5/8)))); % mask into middle quarter and upper half of image - extra LED signal appears there

%% Find corneal reflections
[A, crxy0, crr0] = CircularHoughGrd(img, p.radiiCR, p.CRthresh, p.CRfilter, 1);

% Check for duplicates
i = 1;
while i<length(crr0)
    distThresh = 5; % (px) minimum separation between CRs    
    duplicates = sqrt(sum((repmat(crxy0(i,:),size(crxy0,1),1) - crxy0).^2,2)) < distThresh; % 30
    duplicates(i) = 0;
    crxy0(duplicates,:) = [];
    crr0(duplicates) = [];
    i = i + 1;
end

% If more than 2 CRs detected, sort by distance from center of image
if size(crxy0,1)>2
    x0 = size(img,2)/2;
    y0 = size(img,1)/2;
    dist = sqrt(sum((bsxfun(@minus, crxy0, [x0 y0])).^2, 2));
    [~, inds] = sort(dist);
    
    crxy = crxy0(inds(1:2),:);
    crr = crr0(inds(1:2),:);
else
    crxy = crxy0;
    crr = crr0;
end
crx = crxy(:,1);
cry = crxy(:,2);

% Fill in 2nd CR with NaNs if not present
if size(crxy,1)<2
    [cry(2), crx(2), crr(2)] = deal(NaN);
else
    % Sort by horizontal position
    [crx, ind] = sort(crx,1,'ascend'); 
    crr = crr(ind);  cry = cry(ind);
end

% DEBUG: Plot CR circles

if size(crxy,2)>2
    pause(.1)
end
if debug_on
    figure; imagesc(A); axis image; colormap(gray)
    a = 0:.001:2*pi; hold on
    for ii = 1:length(crr0)
        plot(crr0(ii).*cos(a) + crxy0(ii,1), crr0(ii).*sin(a)+crxy0(ii,2),'c')
    end
    plot(crx, cry, 'r+')
    keyboard
end
%  END DEBUG: Plot CR circles 

% Format output of CR location
cr1 = [crx(1) cry(1) crr(1)];
cr2 = [crx(2) cry(2) crr(2)];

%% Remove corneal reflection
if ~isempty(crxy0)
    
    % Remove bright spots
    maxvals = img(sub2ind(size(img),round(crxy0(:,2)),round(crxy0(:,1))));
    removeCRthresh = max(maxvals)*p.removeCRthresh; % Set thresh for bright spots
    totalMask = img>removeCRthresh;
    totalMask = imdilate(totalMask,strel('disk', p.removeCRwidth)); % Expand the mask 


    InoCR = img;
    InoCR(totalMask) = NaN;
    
    if p.smoothSigma
    gaussian_smooth_image = @(I, sigma) imfilter(I, fspecial('gaussian', [ceil(2.5*sigma) ceil(2.5*sigma)], sigma), 'symmetric');
    InoCR = gaussian_smooth_image(InoCR, p.smoothSigma);
    end
    
    %% ***   DEBUG: plot image after CR removal   ***
    if debug_on;  figure; colormap(gray);  imagesc(InoCR); 
    keyboard
    end
    %*** ENG DEBUG ***
    
else
    InoCR = img;
end

%% Find guess for pupil center using radial symmetry transform
if isempty(pupilStart) || any(isnan(pupilStart)) || pupilStart(1)==0
    alphaFRST = 1;           % Sensitivity to radial symmetry: was 0.5
    imgRadialPupil = radialSymTransform(img, round(p.radiiPupil*[.8 1.2]), alphaFRST);
    imgRadialPupil = removeBorder(imgRadialPupil, .15);
    
    [pupilY, pupilX] = find(min(imgRadialPupil(:))==imgRadialPupil);
    pupilStart = [pupilX pupilY];
end

%% Detect pupil borders using starburst algorithm
[epx, epy, edgeThresh] = starburstPupilContourDetection(InoCR, pupilStart(1),...
    pupilStart(2), edgeThresh, p.radiiPupil, p.minfeatures);
[~, inliers] = fitEllipseRansac(epx(:), epy(:), mean(p.radiiPupil)*[.8 1.2] );%+ [-20 20]

epx2 = epx(inliers);
epy2 = epy(inliers);

points = [];
pupil = NaN(1,5);

if ~isempty(epx2)
    points = [epx2(:), epy2(:)];
    
    
    p.ellipse_residual_thresh = 3; % TODO
    
    % Do better fit of resulting points
    ellipse_result = fitEllipse(epx2,epy2);
    
    if ~isempty(ellipse_result.X0_in)
        
        % Find the ellipse residuals
            a = linspace(0,2*pi,200);

        ellipse_x = ellipse_result.a*cos(a)*cos( -ellipse_result.phi) - sin( -ellipse_result.phi)*ellipse_result.b*sin(a) + ellipse_result.X0_in;
        ellipse_y = ellipse_result.a*cos(a)*sin( -ellipse_result.phi) + cos( -ellipse_result.phi)*ellipse_result.b*sin(a) + ellipse_result.Y0_in;
                
%         figure; plot(ellipse_x, ellipse_y,'.'); hold on; plot(epx2, epy2,'+r')

        % TODO speed up
        residuals = NaN(length(epx2),1);
        for ii = 1:length(epx2)
            dist_sq = ((epx2(ii) - ellipse_x(:)).^2 + (epy2(ii) - ellipse_y(:)).^2);
            residuals(ii) = sqrt(min(dist_sq));
        end
        
        if mean(residuals) < p.ellipse_residual_thresh                        
            pupil(1) = ellipse_result.X0_in;
            pupil(2) = ellipse_result.Y0_in;
            pupil(3) = ellipse_result.a;
            pupil(4) = ellipse_result.b;
            pupil(5) = -ellipse_result.phi;
        else
        end
        
    end    
end

%% Plotting
if plot_on
    a = linspace(0,2*pi,40);

    if isempty(plot_handles)

        plot_handles.img = imagesc(img);  colormap(gray);   axis image
        hold on
        plot_handles.cr1 = plot(cr1(3).*cos(a) + cr1(1), cr1(3).*sin(a)+cr1(2),'b');
        plot_handles.cr2 = plot(cr2(3).*cos(a) + cr2(1), cr2(3).*sin(a)+cr2(2),'c');
        plot_handles.crs = plot(crx, cry,'+r');
        
        % Plot ellipse
        plot_handles.pupil_line = line(pupil(3)*cos(a)*cos(pupil(5)) - sin(pupil(5))*pupil(4)*sin(a) + pupil(1), ...
            pupil(3)*cos(a)*sin(pupil(5)) + cos(pupil(5))*pupil(4)*sin(a) + pupil(2),...
            'Color','k');
        
        plot_handles.pupil_center = plot(pupil(1), pupil(2),'+y','LineWidth',2, 'MarkerSize',10);
        if exist('epx','var')
            plot_handles.pupil_all = plot(epx, epy,'.c');
            plot_handles.pupil_inliers = plot(epx2, epy2,'.y');
        end
                
    else
        
        % Plot image
        set(plot_handles.img, 'CData',img);
        
        % Plot CRs
        set(plot_handles.cr1,'XData', cr1(3).*cos(a) + cr1(1), 'YData', cr1(3).*sin(a)+cr1(2));
        set(plot_handles.cr2,'XData', cr2(3).*cos(a) + cr2(1),'YData', cr2(3).*sin(a)+cr2(2));
        set(plot_handles.crs,'XData',crx, 'YData',cry);
        
        % Plot ellipse
        set(plot_handles.pupil_line, 'XData', pupil(3)*cos(a)*cos(pupil(5)) - sin(pupil(5))*pupil(4)*sin(a) + pupil(1), ...
            'YData',  pupil(3)*cos(a)*sin(pupil(5)) + cos(pupil(5))*pupil(4)*sin(a) + pupil(2));
        
        % Plot pupil points
        set(plot_handles.pupil_center, 'XData', pupil(1), 'YData', pupil(2));
        set(plot_handles.pupil_all, 'XData', epx, 'YData', epy);
        set(plot_handles.pupil_inliers, 'XData', epx2, 'YData', epy2);
    end    
    drawnow
end


