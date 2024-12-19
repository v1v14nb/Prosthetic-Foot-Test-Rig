% GenerateAxialComressiveCode.m
% From Stance phase vs ankle joint angle and vGRF data, creates a joint agnle vs axial compressive load data file and plots it


clear;
close all;

% Load the data from the CSV files
angle_data = readtable('angle_data.csv'); % Load angle data
vGRF_data = readtable('vGRF_data.csv'); % Load vGRF data

% Extract data
angle_StancePhase = angle_data.Stance_Phase;
angles = angle_data.Angle;
vGRF__StancePhase = vGRF_data.Stance_Phase;
vGRFs = vGRF_data.vGRF;

% Convert angles from degrees to radians for the cosine calculation
angles_rad = deg2rad(angles);

% Find the maximum angle and the corresponding stance phase
[max_angle, max_idx] = max(angles);
max_stance_phase = angle_StancePhase(max_idx);

% Filter data to only include entries up to the stance phase at the maximum angle
angle_range = angle_StancePhase <= max_stance_phase;
angle_StancePhase = angle_StancePhase(angle_range);
angles = angles(angle_range);
angles_rad = angles_rad(angle_range);

vGRF_range = vGRF__StancePhase <= max_stance_phase;
vGRF__StancePhase = vGRF__StancePhase(vGRF_range);
vGRFs = vGRFs(vGRF_range);

% Initialize axial compression array
axial_compression = zeros(size(angles));

% For each angle, find the corresponding vGRF and compute axial compression
for i = 1:length(angles)
    % Find matching vGRF for the current stance phase
    idx = find(vGRF__StancePhase == angle_StancePhase(i));
    if isempty(idx)
        axial_compression(i) = NaN; % Handle case where no match is found
    else
        % Compute axial compression using vGRF and cosine of the angle
        axial_compression(i) = vGRFs(idx) / cos(angles_rad(i));
    end
end

% Plot axial compression vs. angle
figure;
plot(angles, axial_compression, '-o', 'LineWidth', 1.5, 'MarkerSize', 4);
grid on;
xlabel('Joint Angles (degrees)');
ylabel('Axial Compression (BW)');
title('Axial Compression vs. Angle (Bounded by Max Angle)');
legend('Axial Compression vs. Angle');

% Save angle vs. axial compression data to a CSV file
output_table = table(angles, axial_compression, 'VariableNames', {'Angle', 'AxialCompression'});
writetable(output_table, 'angle_vs_axial_compression.csv'); % Save as CSV
disp('Angle vs. Axial Compression data saved as "angle_vs_axial_compression.csv"');
