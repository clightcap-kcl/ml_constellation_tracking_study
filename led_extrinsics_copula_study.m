
% script to perform LED extrinsics copula study
clear variables
mmTom = 0.001;
kuka_static_poses_filename = 'kuka_static_poses.csv';
optotrak_static_poses_filename = 'opto_static_poses.csv';
results_directory = fullfile('constellation_data',...
    'GC334020004A-2020-02-04-21-05-45');
%     'GC334020004A-2020-01-16-19-47-22');

% load LED extrinsics results from file
% led_extrinsics = xlsread('Generated_LED_Copula.xlsx');
led_extrinsics = xlsread('Original_LED_Extrinsics.xlsx','Cal Data');

% load prototype/reference calibration file
calib_data = yaml.ReadYaml('calib.yaml');

% interate through copula results
number_of_samples = size(led_extrinsics,1);
copula_rig_position_errors_mm = [];
copula_rig_position_rms_error_mm = nan(number_of_samples,1);
copula_rig_position_q95_error_mm = nan(number_of_samples,1);
copula_rig_rotation_errors_deg = [];
copula_rig_rotation_rms_error_deg = nan(number_of_samples,1);
copula_rig_rotation_q95_error_deg = nan(number_of_samples,1);

for copula_i=1:10%number_of_samples
    fprintf('Copula No. %d\n',copula_i);
    
    % write LED extrinsics calibration to calib.yaml file for solver
    write_led_extrinsics_calib(fullfile(results_directory,'calib.yaml'),...
        calib_data,...
        led_extrinsics(copula_i,:));
    
    % evaluate constellation tracking errors
    [rig_p_rig_norm_error_mm,rig_r_rig_norm_error_deg] = ...
        evaluate_constellation_tracking(kuka_static_poses_filename,...
        optotrak_static_poses_filename,...
        results_directory);

    % store copula results for each sample
    copula_rig_position_errors_mm = [copula_rig_position_errors_mm; ...
        rig_p_rig_norm_error_mm]; %#ok<AGROW>
    copula_rig_position_rms_error_mm(copula_i) = rms(rig_p_rig_norm_error_mm);
    copula_rig_position_q95_error_mm(copula_i) = quantile(rig_p_rig_norm_error_mm,0.95);
    
    copula_rig_rotation_errors_deg = [copula_rig_rotation_errors_deg; ...
        rig_r_rig_norm_error_deg]; %#ok<AGROW>
    copula_rig_rotation_rms_error_deg(copula_i) = rms(rig_r_rig_norm_error_deg);
    copula_rig_rotation_q95_error_deg(copula_i) = quantile(rig_r_rig_norm_error_deg,0.95);
end

% str = datestr(now,'yymmdd_HHMMSS');
% save(['copula_' str '.mat']);

%% plot results
font_size = 15;
nbins = 20;
figure, hist(copula_rig_position_errors_mm,nbins);
xlabel('Rig Position Error [mm]');
set(gca,'FontSize',font_size);

figure, hist(copula_rig_position_rms_error_mm,nbins);
xlabel('RMS Rig Position Error [mm]');
set(gca,'FontSize',font_size);

figure, hist(copula_rig_position_q95_error_mm,nbins);
xlabel('Q95 Rig Position Error [mm]');
set(gca,'FontSize',font_size);

figure, hist(copula_rig_rotation_errors_deg,nbins);
xlabel('Rig Rotation Error [deg]');
set(gca,'FontSize',font_size);

figure, hist(copula_rig_rotation_rms_error_deg,nbins);
xlabel('RMS Rig Rotation Error [deg]');
set(gca,'FontSize',font_size);

figure, hist(copula_rig_rotation_q95_error_deg,nbins);
xlabel('Q95 Rig Rotation Error [deg]');
set(gca,'FontSize',font_size);

