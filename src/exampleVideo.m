function exampleVideo(filepathname, fps, Hback, Hfront, Eback, Efront, write_on)
% Make an example video showing head and eye vector
%
% exampleVideo(filepathname, dt, Hback, Hfront, Eback, Efront)

if write_on
v = VideoWriter(filepathname);
v.FrameRate = fps;
v.Quality = 95;
open(v)
end
fh = figure; shrink(2)
ax = gca;
ii = 1;
w = 2;

% Plot the head vector with an arrow at the beak marker
h_mean = nanmean([Hback; Hfront]);
h1 = Hback(ii,:);
h2 = Hfront(ii,:);
hp = arrow3(h1,h2,'k2', w, w); hold on; % s,w,h,ip,alpha,beta
set(hp,'Clipping','off')

% Plot the eye vector with an origin at the center or corneal curvature
% and an arrow in the direction of the pupil
e1 =  Eback(ii,:);
e2 =  Efront(ii,:);
hp = arrow3(e1,e2,'r2', w, w); hold on; % s,w,h,ip,alpha,beta
set(hp,'Clipping','off')

axis equal
a = 10;
xlim(a*[-1 1]+h_mean(1))
ylim(a*[-1 1]+h_mean(2))
zlim(a*[-1 1]+h_mean(3))
axis vis3d; 

grid on
xlabel('x')
ylabel('y')
zlabel('z')
% view(-23.2474, 64.4333)
view([0 0 -1])
for ii =1:size(Hback,1)
    
    cla(ax)
    %     axes(ax)
    
    % Plot the head vector with an arrow 
    h1 = Hback(ii,:) + (Hfront(ii,:)-Hback(ii,:))*.4;
    h2 = Hfront(ii,:) - (Hfront(ii,:)-Hback(ii,:))*.4;
    
%     h1 = Hback(ii,:);
%     h2 = Hfront(ii,:);
    hp = arrow3(h1,h2,'k2', w, w); hold on; % s,w,h,ip,alpha,beta
    set(hp,'Clipping','off')
    
    % Plot the eye vector with an origin at the center or corneal curvature
    % and an arrow in the direction of the pupil
    % e1 =  Eback(ii,:) - (Efront(ii,:)-Eback(ii,:))*1;
    % e2 =  Efront(ii,:) + (Efront(ii,:)-Eback(ii,:))*1;
    
    e1 =  Eback(ii,:);
    e2 =  Efront(ii,:);
    hp = arrow3(e1,e2,'r2', w, w); hold on; % s,w,h,ip,alpha,beta
    set(hp,'Clipping','off')
    drawnow
    
    if write_on
    I = getframe(fh);
    writeVideo(v,I)
    end
    pause(0.02)
end

if write_on
close(v)
end
