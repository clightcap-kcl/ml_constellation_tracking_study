
% evaluate constellation tracking errors
function [rig_p_rig_norm_error_mm,rig_r_rig_norm_error_deg,pose_set,... MUST COMMENT BEFORE COMPILING!!!!
    array_T_rig,optotrak_T_wearable] = ...
    evaluate_constellation_tracking(kuka_static_poses_filename,...
    optotrak_static_poses_filename,...
    results_directory)

% summary of function inputs
% kuka_static_poses_filename: path specifying csv file with KUKA poses
% collected during hand-eye calibration (order should be based on pose
% numbers in KRL code and not following chronological pose order)
%
% optotrak_static_poses_filename: path specifying csv file with Optotrak
% poses collected during hand-eye calibration (order should be based on
% chronological pose order and not KRL pose numbers)
%
% results_directory: directory containing constellation algo results file
% constellation_pose_results.yaml; also target directory for output results
%
% write_images: boolean specifying whether to write output images to png
% files (includes rotation error and algo error plots)
%
% summary of function outputs
% rms_rig_error_mm: RMS residual position error between constellation algo
% and optotrak reference measurement after hand-eye calibration [mm]
%
% rig_r_rig_error_deg: RMS residual rotation error between constellation algo
% and optotrak reference measurement after hand-eye calibration [deg]
%
% array_T_rig: pose of rig frame, measured by constellation algo, with
% respect to optotrak array frame
%
% optotrak_T_wearable: pose of wearable (reference camera) with respect to
% optotrak base frame

%% constants used in program
mTomm = 1000;
DegToRad = pi / 180;
cell_length_mm = 2.7e3;
debug_on = false;

% Jan 14, 2020
pose_order = [1, 45, 16, 10, 34, ...
    25, 4, 42, 19, 15, ...
    33, 30, 2, 41, 17, ...
    11, 36, 27, 3, 40, ...
    18, 12, 32, 26, 6];

initial_pose_set = 1:length(pose_order);
initial_pose_set([7,9]) = []; % remove outliers

% load KUKA static data
kuka_T_ee_raw = load_kuka_static_data(kuka_static_poses_filename,pose_order);
kuka_T_ee_input = kuka_T_ee_raw;

% set default path
optotrak_static_data_raw = csvread(optotrak_static_poses_filename,1);
optotrak_static_data = optotrak_static_data_raw;

%% run constellation tracking algorithm
constellation_app = '/mnt/c/Users/clightcap/Documents/Code/constellation_tracker/constellation_tracker/build/bin/main_constellation_tracker_module';
linux_results_dir = ['/mnt/c/Users/clightcap/Documents/Code/led_extrinsics_copula_study/',...
    replace(results_directory,'\','/')];
command = ['wsl ' ...
    constellation_app ...
    ' --dataset_path ' ...
    linux_results_dir ...
    ' --output_path ' ...
    linux_results_dir ...
    ' --noviz'];

% fprintf('Executing command: %s\n',command);
[status,cmdout] = system(command) %#ok<NOPTS>

%% load yaml file for current robot poses
% addpath(genpath('C:\Users\clightcap\Documents\Code\yamlmatlab'));
constellation_data_filename = fullfile(results_directory, 'constellation_pose_results.yaml');
constellation_data_raw = yaml.ReadYaml(constellation_data_filename);
constellation_data = constellation_data_raw;

%% compute Ai and Bi matrices for hand-eye calibration
c = 0;
optotrak_T_array_raw = cell(1,length(optotrak_static_data));
for pose_i=1:length(optotrak_static_data)
    % compute optotrak reference pose for all poses
    optotrak_p_array_mm = optotrak_static_data(pose_i,1:3)';
    optotrak_r_array_deg = optotrak_static_data(pose_i,4:end)';
    
    optotrak_R_array = angvec2r(DegToRad * norm(optotrak_r_array_deg), ...
        unit(optotrak_r_array_deg)); % only accepts radians
    optotrak_T_array_raw{pose_i} = [optotrak_R_array, ...
        optotrak_p_array_mm; 0 0 0 1];
    
    % check whether pose is member of initial set and constellation found
    if (ismember(pose_i,initial_pose_set) && ...
            constellation_data{pose_i}.pose_found)
        % increment counter
        c = c + 1;
        pose_set(c) = pose_i; %#ok<SAGROW>
        
        % compute constellation pose
        wearable_q_rig = UnitQuaternion(struct2array(constellation_data{pose_i}.pose.rotation));
        wearable_R_rig = wearable_q_rig.R;
        wearable_p_rig_mm(c,:) = mTomm * struct2array(constellation_data{pose_i}.pose.translation)'; %#ok<SAGROW>
        wearable_T_rig{c} = [wearable_R_rig, wearable_p_rig_mm(c,:)'; 0 0 0 1]; %#ok<SAGROW>
        
        % store optotrak pose
        optotrak_T_array{c} = optotrak_T_array_raw{pose_i}; %#ok<SAGROW>
        
        % compute KUKA pose
        kuka_T_ee{c} = kuka_T_ee_input{pose_i};      %#ok<SAGROW>
    end
end

%% compute hand-eye calibration result and evaluate residual errors
number_of_cal_poses = length(optotrak_T_array);
number_of_cal_motions = number_of_cal_poses - 1;

% pre-allocate arrays
rig_T_rig = cell(1,number_of_cal_motions);
array_T_array = cell(1,number_of_cal_motions);
ee_T_ee = cell(1,number_of_cal_motions);
rig_r_rig_deg = nan(number_of_cal_motions, 3);
array_r_array_deg = nan(number_of_cal_motions, 3);
ee_r_ee_deg = nan(number_of_cal_motions, 3);

% compute rotation differences
for motion_i=1:number_of_cal_motions
    first_pose = motion_i;
    second_pose = motion_i + 1;
    rig_T_rig{motion_i} = wearable_T_rig{first_pose} \ ...
        wearable_T_rig{second_pose};
    [th,r] = tr2angvec(rig_T_rig{motion_i},'deg');
    rig_r_rig_deg(first_pose,:) = th * r;
    
    array_T_array{motion_i} = optotrak_T_array{first_pose} \ ...
        optotrak_T_array{second_pose};
    [th,r] = tr2angvec(array_T_array{motion_i},'deg');
    array_r_array_deg(first_pose,:) = th * r;
    
    ee_T_ee{motion_i} = kuka_T_ee{first_pose} \ kuka_T_ee{second_pose};
    [th,r] = tr2angvec(ee_T_ee{motion_i},'deg');
    ee_r_ee_deg(first_pose,:) = th * r;
end

% solve for hand-eye calibration solution
Ai = cell2mat(array_T_array);
Bi = cell2mat(rig_T_rig);
array_T_rig = liang(Ai,Bi);

Ai = cell2mat(array_T_array);
Bi = cell2mat(ee_T_ee);
array_T_ee = liang(Ai,Bi);

%% compute rotation differences
rig_r_rig_norm_deg = sqrt(sum(rig_r_rig_deg.^2,2));
array_r_array_norm_deg = sqrt(sum(array_r_array_deg.^2,2));
ee_r_ee_norm_deg = sqrt(sum(ee_r_ee_deg.^2,2));
rotation_delta_deg = rig_r_rig_norm_deg - array_r_array_norm_deg;
rms_rig_array_delta_deg = rms(rotation_delta_deg);
rms_ee_array_delta_deg = rms(ee_r_ee_norm_deg - array_r_array_norm_deg);

% compute rotation directions
rotation_directions = ee_r_ee_deg./repmat(ee_r_ee_norm_deg,1,3);

%% compute residual errors
optotrak_T_wearable_i = zeros(4,4,number_of_cal_poses);
optotrak_T_kuka_i = zeros(4,4,number_of_cal_poses);

% calculate average pose and stats
for pose_i=1:number_of_cal_poses
    optotrak_T_wearable_i(:,:,pose_i) = optotrak_T_array{pose_i} * array_T_rig / wearable_T_rig{pose_i};
    optotrak_T_kuka_i(:,:,pose_i) = optotrak_T_array{pose_i} * array_T_ee / kuka_T_ee{pose_i};
end
optotrak_T_wearable = average_hgt(optotrak_T_wearable_i);
optotrak_T_kuka = average_hgt(optotrak_T_kuka_i);

% pre-allocate arrays
wearable_T_rig_est = nan(4,4,number_of_cal_poses);
wearable_p_rig_mm_est = nan(number_of_cal_poses,3);
kuka_T_rig_est = nan(4,4,number_of_cal_poses);
kuka_p_rig_mm_est = nan(number_of_cal_poses,3);
kuka_T_ee_est = nan(4,4,number_of_cal_poses);
rig_p_rig_error_mm = nan(number_of_cal_poses,3);
rig_r_rig_error_deg = nan(number_of_cal_poses,3);
ee_p_ee_error_mm = nan(number_of_cal_poses,3);
ee_r_ee_error_deg = nan(number_of_cal_poses,3);
for pose_i=1:number_of_cal_poses
    % compute wearable hand-eye calibration errors
    wearable_T_rig_est(:,:,pose_i) = optotrak_T_wearable \ ...
        optotrak_T_array{pose_i} * ...
        array_T_rig;
    wearable_p_rig_mm_est(pose_i,:) = wearable_T_rig_est(1:3,4,pose_i);
    rig_T_rig_error = wearable_T_rig_est(:,:,pose_i) \ wearable_T_rig{pose_i};
    rig_p_rig_error_mm(pose_i,:) = rig_T_rig_error(1:3,4);
    [th, r] = tr2angvec(trnorm(rig_T_rig_error),'deg');
    rig_r_rig_error_deg(pose_i,:) = th * r;
    
    % compute KUKA hand-eye calibration errors
    kuka_T_ee_est(:,:,pose_i) = optotrak_T_kuka \ ...
        optotrak_T_array{pose_i} * ...
        array_T_ee;
    ee_T_ee_error = kuka_T_ee_est(:,:,pose_i) \ kuka_T_ee{pose_i};
    ee_p_ee_error_mm(pose_i,:) = ee_T_ee_error(1:3,4);
    [th, r] = tr2angvec(trnorm(ee_T_ee_error),'deg');
    ee_r_ee_error_deg(pose_i,:) = th * r;
    
    % compute rig pose relative to kuka base
    kuka_T_rig_est(:,:,pose_i) = optotrak_T_kuka \ ...
        optotrak_T_array{pose_i} * ...
        array_T_rig;
    kuka_p_rig_mm_est(pose_i,:) = kuka_T_rig_est(1:3,4,pose_i);
end

%% compute estimated rig pose for all images (even those excluded)
% fileID = fopen(fullfile(results_directory,'recording.txt'),'w');
% fprintf(fileID,'Totem Simulation Recording File V1\n');
% fprintf(fileID,'Pose definition: RIG_T_TOTEM as in RIG_p = RIG_T_TOTEM* TOTEM_p\n');
% fprintf(fileID,'frame_name,timestamp[s],pos_x[m],pos_y[m],pos_z[m],ori_w,ori_x,ori_y,ori_z\n');
% for pose_i=1:length(optotrak_T_array_raw)
%     % compute wearable hand-eye calibration errors
%     wearable_T_rig_for_file = optotrak_T_wearable \ ...
%         optotrak_T_array_raw{pose_i} * ...
%         array_T_rig;
%     
%     est_rig_pose_for_file = [(1/mTomm)*wearable_T_rig_for_file(1:3,4)', ...
%         UnitQuaternion(wearable_T_rig_for_file).double];
% 
%     timestamp = -1;
%     fprintf(fileID,'frame_%d,%d,%8.6f,%8.6f,%8.6f,%8.6f,%8.6f,%8.6f,%8.6f\n',...
%         pose_i,timestamp,est_rig_pose_for_file);
% end
% fclose(fileID);
        
%% compute distance from reference control (wearable) to DUT
% wearable_p_rig_norm_mm = sqrt(sum(wearable_p_rig_mm.^2,2));
% wearable_p_rig_norm_mm_est = sqrt(sum(wearable_p_rig_mm_est.^2,2));
% dut_distance_to_wall_mm = cell_length_mm/2 - max(abs(kuka_p_rig_mm_est(:,1:2)),[],2);

% compute rms errors
rig_p_rig_norm_error_mm = sqrt(sum(rig_p_rig_error_mm.^2,2));
rig_r_rig_norm_error_deg = sqrt(sum(rig_r_rig_error_deg.^2,2));

% ee_p_ee_norm_error_mm = sqrt(sum(ee_p_ee_error_mm.^2,2));
% ee_r_ee_norm_error_deg = sqrt(sum(ee_r_ee_error_deg.^2,2));

% rms_rig_error_mm = rms(rig_p_rig_norm_error_mm);
% q95_rig_error_mm = quantile(rig_p_rig_norm_error_mm,0.95);
% 
% rms_rig_error_deg = rms(rig_r_rig_norm_error_deg);
% q95_rig_error_deg = quantile(rig_r_rig_norm_error_deg,0.95);
% 
% rms_ee_error_mm = rms(ee_p_ee_norm_error_mm);
% rms_ee_error_deg = rms(ee_r_ee_norm_error_deg);

%% plot results
% plot algo-optotrak rotation errors
marker_size = 14;
if debug_on
    h_rotation_error = figure();
    
    clf;
    yyaxis left
    plot(pose_set(1:end-1),rig_r_rig_norm_deg,'.-',...
        'MarkerSize',marker_size,...
        'DisplayName','Algo'), hold on
    plot(pose_set(1:end-1),array_r_array_norm_deg,'.-',...
        'MarkerSize',marker_size,...
        'DisplayName','Optotrak')
    plot(pose_set(1:end-1),ee_r_ee_norm_deg,'.-',...
        'MarkerSize',marker_size,...
        'DisplayName','Robot')
    ylabel('Total Rotation Angle [deg]');
    yyaxis right
    plot(pose_set(1:end-1),rotation_delta_deg,'.-',...
        'MarkerSize',marker_size,...
        'DisplayName','Error')
    xlabel('Motion Number');
    ylabel('Rotation Error [deg]');
    legend('show')
    set(gca,'FontSize',14);
    grid on
    grid minor
    
    % plot algo-optotrak hand-eye calibration errors
    h_algo_error = figure();
    clf;
    subplot(2,1,1)
    plot(pose_set,rig_p_rig_error_mm,'.-',...
        'MarkerSize',marker_size);
    ylabel('Algo Error (mm)')
    set(gca,'FontSize',14);
    grid on
    grid minor
    subplot(2,1,2)
    plot(pose_set,rig_r_rig_error_deg,'.-',...
        'MarkerSize',marker_size)
    ylabel('Algo Error (deg)')
    xlabel('Pose number')
    set(gca,'FontSize',14);
    grid on
    grid minor
    
    % plot robot-optotrak hand-eye calibration errors
    figure, subplot(2,1,1)
    plot(pose_set,ee_p_ee_error_mm,'.-',...
        'MarkerSize',marker_size)
    ylabel('KUKA Error (mm)')
    set(gca,'FontSize',14);
    grid on
    grid minor
    subplot(2,1,2)
    plot(pose_set,ee_r_ee_error_deg,'.-',...
        'MarkerSize',marker_size)
    ylabel('KUKA Error (deg)')
    xlabel('Pose number')
    set(gca,'FontSize',14);
    grid on
    grid minor
    
    % plot rotational directions
    plot_directions(rotation_directions);
    
    % plot distance from reference camera
    figure, hold on
    plot(pose_set,wearable_p_rig_norm_mm,'.-',...
        'MarkerSize',marker_size,...
        'DisplayName','Measured')
    plot(pose_set,wearable_p_rig_norm_mm_est,'.-',...
        'MarkerSize',marker_size,...
        'DisplayName','Estimated')
    ylabel('Distance to Camera (mm)')
    xlabel('Pose number')
    set(gca,'FontSize',14);
    grid on
    grid minor
    legend('show');
    
    % plot distance to workcell wall
    figure, hold on
    plot(pose_set,dut_distance_to_wall_mm,'.-',...
        'MarkerSize',marker_size)
    ylabel('Distance to Workcell Edge (mm)')
    xlabel('Pose number')
    set(gca,'FontSize',14);
    grid on
    grid minor
    
    
    saveas(h_rotation_error,fullfile(results_directory,'constellation_rotation_error.png'));
    saveas(h_algo_error,fullfile(results_directory,'constellation_algo_error.png'));
end

% end