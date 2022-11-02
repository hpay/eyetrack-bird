function gazeTemplateCombine
data_root = 'Z:\Hannah\dualcamera';

clear Ht_all
folder_eye_calib = {'AMB111_220717a','IND102_220707a'};

for ii = 1:length(folder_eye_calib)
        
    Ht_all(ii,:)  = getfield(load(fullfile(data_root,folder_eye_calib{ii},'head_calibration_template.mat')),'Ht');
    
end

Ht = varfun(@mean, Ht_all, 'InputVariables', @isnumeric);
Ht.Properties.VariableNames = Ht_all.Properties.VariableNames(2:end);
save('gazeTemplate.mat','Ht')
