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

fprintf('Stdev. horizontal eye angle %.1f +- %.1f°, vertical eye angle %.1f +- %.1f,\n n = %i bird, mean %.0f frames per bird\n',...
    mean(stats.std_theta), std(stats.std_theta), mean(stats.std_phi), std(stats.std_phi), height(stats), mean(stats.nframes))

%%
bin_edges = 0:.2:8;
figure('Units','centi','Pos',[  17.3038   12.8058   14.8167    5.8208]); 
subplot(1,2,1)
histogram(stats.std_theta, bin_edges,'FaceColor','c')
xlabel('Horizontal stdev. (°)')
ylabel('# birds')
axis square

subplot(1,2,2)
histogram(stats.std_phi,bin_edges,'FaceColor','c')
xlabel('Vertical stdev. (°)')
axis square
fixticks
