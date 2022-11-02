function [px1, py1, ellipse_result] = robustEllipse(px0, py0, p, debug_on)
% robustEllipse Grouped outlier removal for robust ellipse fitting
% [px1, py1] = ROBUSTELLIPSE(px, py, p) Returns the subset of points px
% and py that are included in the final ellipse
%
% [px1, py1, ellipse_result] = ROBUSTELLIPSE(px, py, p) Returns ellipse
% parameters [x0, y0, a, b, phi] in ellipse_result
% 
% ... = ROBUSTELLIPSE(px, py, p, debugon) Plots a debugging step in a new
% figure
%
% Based on Shao, Ijiri & Hattori 2015. Grouped Outlier Removal for Robust Ellipse Fitting
% http://www.mva-org.jp/Proceedings/2015USB/papers/05-28.pdf
%
% Implemented by Hannah Payne, 2022

% 1. Segment the points into clusters by proximity
if ~exist('p','var')
    p = [];
    p.min_dist = 2; % distance threshold for segmentation (t)
    p.min_num = 6;  % min number of points per cluster
    p.D = 10;       % expected number of sets
    p.S = 3;        % number of additional sets to keep in each iteration
    p.S_max = 10;   % max number of subsets to keep in each iteration
    p.sigma = 1.05; % error threshold; searching process converges if excluding any subset does not reduce the energy up to certain rate σ
    p.eta = 3;      % default 5. error threshold; in case there are much more outliers than average subset size so that removing any subset does not reduce the energy
    p.inclusion_penalty = 0;    % penalize fits that exclude too many points
    p.size_penalty = 0;         % penalize fits that don't match size_ellipse
    p.size_ellipse = 25;        % Guess to enforce correct pupil size (pixels)
end

p0 = [px0(:) py0(:)];
C = {[]};
N = length(px0);
S = p.S;

% Get distances between each pair of points
d = NaN(N,1);
for ii = 1:N   
    if ii==N; iip1 = 1; else; iip1 = ii+1; end
    d(ii) = sqrt(sum((p0(ii,:) - p0(iip1,:)).^2));    
end

% Segment into groups
kk = 1;
for ii = 1:N
    C{kk,1} = [C{kk};  p0(ii,:)];
    if d(ii) > p.min_dist*median(d)
        kk = kk+1;
        C{kk,1} = [];
    end
end

% Delete any clusters that are too small
C(cellfun(@(x)size(x,1), C)<p.min_num) = [];

% Split clusters that are too big
kk = 1;
while kk<=numel(C)
    C_length = size(C{kk},1);
   if size(C{kk},1)>(2*N/p.D)
       nsets = ceil(C_length/(2*N/p.D));
       nper_set = round(C_length/nsets);
       Cnew = {};
       for jj = 1:nsets           
       Cnew{jj,1} = C{kk}(1+(jj-1)*nper_set:min(jj*nper_set, C_length),:);
       end
       C = [C(1:kk-1); Cnew; C(kk+1:end)];
       kk = kk+nsets-1;
%    elseif kk<length(C) && (C_length + size(C{kk+1},1))<N/p.D
%        C{kk} = [C{kk}; C{kk+1}];
%        C(kk+1) = [];      
   end    
   kk = kk+1;
end



%% Searching for inlier clusters
K = numel(C);

% Fit with all data points
% [u0(1,:), lamda0] = EllipseFitByTaubin(cell2mat(C));

% First round of excluding one cluster at a time
u1 = NaN(K,6); % Ellipse params
lambda1 = NaN(K,1);
Ns= NaN(K,1); % number of points
radii= NaN(K,1); % number of points
for mm = 1:K
    Ctest = cell2mat(C((1:K)~=mm));
    [u1(mm,:), lambda1(mm)] = EllipseFitByTaubin(Ctest);    
    Ns(mm) = size(Ctest, 1);
    radii(mm) = maxRadius(u1(mm,:));
end
lambda1 = lambda1 + p.inclusion_penalty * (1-Ns/N) + p.size_penalty*(radii - p.size_ellipse).^2/p.size_ellipse^2;
    
[~, inds] = sort(lambda1);
Os = inds(1:S); % Set of top S clusters
Es = lambda1(inds(1:S));              % Corresponding energies
Us = u1(inds(1:S),:);                % Corresponding ellipses

% Exclude further clusters
for ll = (K-2):-1:1
    us = NaN(K,size(Os,1),6); % Ellipse params
    lambdas = NaN(K,size(Os,1));
    Ns = NaN(K,size(Os,1));
    radii = NaN(K,size(Os,1));
    [E_old, best_s_old] = min(Es);
    for ss = 1:size(Os,1)
        m_remaining = find(~ismember(1:K, Os(ss,:)));        
        for mm = m_remaining
            m_curr = m_remaining(m_remaining~=mm);
            Ctest = cell2mat(C(m_curr));
            [us(mm,ss,:), lambdas(mm,ss)] = EllipseFitByTaubin(Ctest);
            Ns(mm, ss) = size(Ctest, 1);
            radii(mm,ss) = maxRadius(us(mm,ss,:));
        end
    end
    lambdas = lambdas + p.inclusion_penalty*(1-Ns/N) + p.size_penalty*(radii - p.size_ellipse).^2/p.size_ellipse^2;
    
    [E_new, best_ind_new] = min(lambdas(:));
%     [best_m_new, best_s_new] = ind2sub(size(lambdas), best_ind_new);
    
    
    % If the old one was pretty much the same:
    if (E_old<p.sigma*E_new && E_old<p.eta) || E_old<E_new || ll==1
        
%         Obest = Os(best_s_old,:);
%         max_inliers = cell2mat(C(~ismember(1:K, Obest)));
%         px1 = max_inliers(:,1);
%         py1 = max_inliers(:,2);
%         u_best = Us(best_s_old,:);
%         ellipse_result = convertEllipse(u_best); % maxRadius(u_best)
%         
        break
%         
%         If the latest removal is now below threshold:
%     elseif E_new<p.eta
%         Obest = [Os(best_s_new,:) best_m_new];
%         max_inliers = cell2mat(C(~ismember(1:K, Obest)));
%         px1 = max_inliers(:,1);
%         py1 = max_inliers(:,2);
%         u_best = squeeze(us(best_m_new,best_s_new,:));
%         ellipse_result = convertEllipse(u_best);
%         
%         break
        
        % Keep looping
    else
        if S>ll
            S = ll;
        end
        Es = [];
        Us = [];
        Os_old = Os;
        Os_new = [];
        for ss = 1:size(Os_old,1)
            Os_temp = NaN(S,size(Os_old,2)+1);
            [~, inds] = sort(lambdas(:,ss));
            Es_temp = lambdas(inds(1:S),ss);
            u1s_temp = us(inds(1:S),ss,:);
            for si = 1:S
                Os_temp(si,:) = [Os_old(ss,:) inds(si)];                
            end
            Os_new = [Os_new; Os_temp];
            Es = [Es; Es_temp];
            Us = [Us; squeeze(u1s_temp)];                              
        end
        
        % Remove duplicates
        Os_new = sort(Os_new,2); % Sort each row in order to compare uniqueness next
        [Os_new, mask_unique] = unique(Os_new,'rows') ;
        Es = Es(mask_unique,:);
        Us = Us(mask_unique,:);
        Os = Os_new;
        
        % Subselect best N_max subsets
        if size(Os,1)>p.S_max
            [Es, order] = sort(Es);
            Os = Os(order,:);
            Us = Us(order,:);
            
            Es = Es(1:p.S_max,:);
            Os = Os(1:p.S_max,:);
            Us = Us(1:p.S_max,:);

        end
    end

end

Obest = Os(best_s_old,:);
max_inliers = cell2mat(C(~ismember(1:K, Obest)));
px1 = max_inliers(:,1);
py1 = max_inliers(:,2);
u_best = Us(best_s_old,:);
ellipse_result = convertEllipse(u_best); % maxRadius(u_best)
%

%% Debugging, plot clusters
if exist('debug_on','var') && debug_on
    figure;  colors = jet(length(C));
    for kk = 1:length(C)
        line(C{kk}(:,1), C{kk}(:,2),'LineStyle','none','Marker', 'o','MarkerEdgeColor',colors(kk,:)); hold on;
    end
    
    scatter(px1, py1,16,'k+');
    a = linspace(0,2*pi,200);
    line(ellipse_result.a*cos(a)*cos(ellipse_result.phi) - sin(ellipse_result.phi)*ellipse_result.b*sin(a) + ellipse_result.x0, ...
        ellipse_result.a*cos(a)*sin(ellipse_result.phi) + cos(ellipse_result.phi)*ellipse_result.b*sin(a) +ellipse_result.y0,...
        'Color','k');
    axis equal; set(gca,'YDir','reverse');    
end

end


function rad = maxRadius(A)

a = A(1); b = A(2)/2; c = A(3); d = A(4)/2; f = A(5)/2; g = A(6);
ra = sqrt( 2*(a*f^2+c*d^2+g*b^2-2*b*d*f-a*c*g) / ((b^2-a*c)*(sqrt((a-c)^2+4*b^2)-(a+c))));
rb = sqrt( 2*(a*f^2+c*d^2+g*b^2-2*b*d*f-a*c*g) / ((b^2-a*c)*(-sqrt((a-c)^2+4*b^2)-(a+c))));
rad = abs(max(ra,rb));
end
    

function ellipse_result = convertEllipse(A)
% ellipse_result = [x0 y0 ra rb phi];

a = A(1); b = A(2)/2; c = A(3); d = A(4)/2; f = A(5)/2; g = A(6);

x0 = (c*d - b*f)/(b^2-a*c);
y0 = (a*f - b*d)/(b^2-a*c);

ra = sqrt( 2*(a*f^2+c*d^2+g*b^2-2*b*d*f-a*c*g) / ((b^2-a*c)*(sqrt((a-c)^2+4*b^2)-(a+c))));
rb = sqrt( 2*(a*f^2+c*d^2+g*b^2-2*b*d*f-a*c*g) / ((b^2-a*c)*(-sqrt((a-c)^2+4*b^2)-(a+c))));

if b == 0 && a < c
phi = 0;
elseif b == 0 && a > c
phi = 0.5*pi;
elseif b ~= 0 && a < c
phi = 0.5* acot((a-c)/(2*b));
else
phi = 0.5*pi + 0.5* acot((a-c)/(2*b));
end
% ellipse_result = [x0 y0 ra rb phi];


ellipse_result=[];
ellipse_result.x0 = x0;
ellipse_result.y0 = y0;
ellipse_result.a = ra;
ellipse_result.b = rb;
ellipse_result.phi = phi;

end