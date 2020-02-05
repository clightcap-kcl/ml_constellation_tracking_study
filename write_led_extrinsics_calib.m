function write_led_extrinsics_calib(filename, calib_data, led_extrinsics)
mmTom = 0.001;

% re-write LED extrinsics positions
file_id = fopen(filename,'w');
fprintf(file_id,'led_positions:\n');
for led_i=1:length(calib_data.led_positions)
    fprintf(file_id,'- x: %15.15f\n',mmTom*led_extrinsics(3*led_i-2));
    fprintf(file_id,'  y: %15.15f\n',mmTom*led_extrinsics(3*led_i-1));
    fprintf(file_id,'  z: %15.15f\n',mmTom*led_extrinsics(3*led_i-0));
    fprintf(file_id,'  dir_x: %15.15f\n',calib_data.led_positions{led_i}.dir_x);
    fprintf(file_id,'  dir_y: %15.15f\n',calib_data.led_positions{led_i}.dir_y);
    fprintf(file_id,'  dir_z: %15.15f\n',calib_data.led_positions{led_i}.dir_z);
end

% re-write camera calibration
fprintf(file_id,'camera_left:\n');
fprintf(file_id,'  camera_type: %s\n',calib_data.camera_left.camera_type);
fprintf(file_id,'  image_width: %d\n',calib_data.camera_left.image_width);
fprintf(file_id,'  image_height: %d\n',calib_data.camera_left.image_height);
fprintf(file_id,'  projection_parameters:\n');

fprintf(file_id,'    k1: %15.15f\n',calib_data.camera_left.projection_parameters.k1);
fprintf(file_id,'    k2: %15.15f\n',calib_data.camera_left.projection_parameters.k2);
fprintf(file_id,'    k3: %15.15f\n',calib_data.camera_left.projection_parameters.k3);
fprintf(file_id,'    k4: %15.15f\n',calib_data.camera_left.projection_parameters.k4);
fprintf(file_id,'    k5: %15.15f\n',calib_data.camera_left.projection_parameters.k5);

fprintf(file_id,'    mu: %15.15f\n',calib_data.camera_left.projection_parameters.mu);
fprintf(file_id,'    mv: %15.15f\n',calib_data.camera_left.projection_parameters.mv);
fprintf(file_id,'    u0: %15.15f\n',calib_data.camera_left.projection_parameters.u0);
fprintf(file_id,'    v0: %15.15f\n',calib_data.camera_left.projection_parameters.v0);

fprintf(file_id,'camera_right:\n');
fprintf(file_id,'  camera_type: %s\n',calib_data.camera_left.camera_type);
fprintf(file_id,'  image_width: %d\n',calib_data.camera_left.image_width);
fprintf(file_id,'  image_height: %d\n',calib_data.camera_left.image_height);
fprintf(file_id,'  projection_parameters:\n');

fprintf(file_id,'    k1: %15.15f\n',calib_data.camera_left.projection_parameters.k1);
fprintf(file_id,'    k2: %15.15f\n',calib_data.camera_left.projection_parameters.k2);
fprintf(file_id,'    k3: %15.15f\n',calib_data.camera_left.projection_parameters.k3);
fprintf(file_id,'    k4: %15.15f\n',calib_data.camera_left.projection_parameters.k4);
fprintf(file_id,'    k5: %15.15f\n',calib_data.camera_left.projection_parameters.k5);

fprintf(file_id,'    mu: %15.15f\n',calib_data.camera_left.projection_parameters.mu);
fprintf(file_id,'    mv: %15.15f\n',calib_data.camera_left.projection_parameters.mv);
fprintf(file_id,'    u0: %15.15f\n',calib_data.camera_left.projection_parameters.u0);
fprintf(file_id,'    v0: %15.15f\n',calib_data.camera_left.projection_parameters.v0);

fprintf(file_id,'camera_left_extrinsics:\n');
fprintf(file_id,'  translation:\n');
fprintf(file_id,'    x: 0\n');
fprintf(file_id,'    y: 0\n');
fprintf(file_id,'    z: 0\n');
fprintf(file_id,'  rotation:\n');
fprintf(file_id,'    w: 0\n');
fprintf(file_id,'    x: 0\n');
fprintf(file_id,'    y: 0\n');
fprintf(file_id,'    z: 1\n');

fprintf(file_id,'camera_right_extrinsics:\n');
fprintf(file_id,'  translation:\n');
fprintf(file_id,'    x: 0\n');
fprintf(file_id,'    y: 0\n');
fprintf(file_id,'    z: 0\n');
fprintf(file_id,'  rotation:\n');
fprintf(file_id,'    w: 0\n');
fprintf(file_id,'    x: 0\n');
fprintf(file_id,'    y: 0\n');
fprintf(file_id,'    z: 1\n');

% save to yaml file
% yaml.WriteYaml(fullfile(results_directory,'calib.yaml'),calib);
fclose(file_id);