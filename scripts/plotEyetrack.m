function plotEyetrack(data_root, dir_roots)

stats = table;
filepath_eyes = [];
for ii = 1:length(dir_roots)
    a = strfind(dir_roots{ii},'_');
    filepath_eye = [dir_roots{ii}(1:a(end)-1) '_all'];
    filepath_eyes{ii} = filepath_eye;
    stats_curr = getfield(load(fullfile(data_root, filepath_eye,'head_calibration_stats.mat')),'stats');
    stats = [stats; stats_curr];
end
stats.filepath = filepath_eyes(:);
disp(stats)

fprintf(['Stdev. horizontal eye angle %.1f +- %.1f° left eye, %.1f +- %.1f° right eye \n'...
    'vertical eye angle %.1f +- %.1f left eye,  %.1f +- %.1f right eye (mean+-SEM)\n'],...
mean(stats.std_theta(:,1)), sem(stats.std_theta(:,1)),...
    mean(stats.std_theta(:,2)), sem(stats.std_theta(:,2)),...
    mean(stats.std_phi(:,1)), sem(stats.std_phi(:,1)), ...
    mean(stats.std_phi(:,2)), sem(stats.std_phi(:,2)))

 fprintf(['Corrcoef anterior position v. horiz. angle '...
     '%.2f +- %.2f° left eye, %.2f +- %.2f° right eye (mean+-SEM)\n'],...
    mean(stats.antpos_hang_r(:,1)), sem(stats.antpos_hang_r(:,1)),...
    mean(stats.antpos_hang_r(:,2)), sem(stats.antpos_hang_r(:,2)))
 
 
  fprintf(['mean %.0f frames per bird, range %i - %i left eye \n'...
    'mean %.0f frames per bird, range %i - %i right eye \nn = %i birds\n'],...
    mean(stats.nframes(:,1)), min(stats.nframes(:,1)), max(stats.nframes(:,1)),...
    mean(stats.nframes(:,2)), min(stats.nframes(:,2)), max(stats.nframes(:,2)),...
     height(stats))
 
 
%%
bin_edges = 0:.5:8;
figure('Units','centi','Pos',[  17.3038   12.8058   14.8167    5.8208]); 
subplot(1,2,1)
histogram(stats.std_theta, bin_edges,'FaceColor','c')
xlabel('Horizontal stdev. (°)')
ylabel('# bird-eyes')
axis square

subplot(1,2,2)
histogram(stats.std_phi,bin_edges,'FaceColor','c')
xlabel('Vertical stdev. (°)')
axis square
drawnow
fixticks
