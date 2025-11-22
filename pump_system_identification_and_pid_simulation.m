% DMT Group 16C - Code for System Identification and Simulation Execution 

% Load the frequency gradual change response dataset (gradual_test1.xlsx)
data = readtable('gradual_test1.xlsx', 'ReadVariableNames', false);

% Assign columns to variables
time = data.Var1;
flow_rate = data.Var2;
frequency = data.Var3;

% Load the frequency step change response dataset ('step_test2.xlsx)
step_data = readtable('step_test2.xlsx', 'ReadVariableNames', false);

% Assign columns to variables
time_step = step_data.Var1;
flow_rate_step = step_data.Var2;
frequency_step = step_data.Var3;

% Load the globe valve closing dataset (disturbance_signal_new.xlsx)
clogging_data = readtable('disturbance_signal_new.xlsx', 'ReadVariableNames', false);

% Assign columns to variables
time_clogging = clogging_data.Var1;
flow_rate_clogging = clogging_data.Var2;

% Fit a higher-order polynomial to the clogging data
degree = 4; % Adjust degree as needed
p = polyfit(time_clogging, flow_rate_clogging, degree);

% Define the clogging effect as a function of time
clogging_effect = polyval(p, time_clogging);

% Normalize the clogging effect based on the target flow rate
target_flow_rate = 20;
clogging_effect_normalized = clogging_effect / max(clogging_effect) * target_flow_rate;

% Create an iddata object for the first dataset with frequency as input
Ts1 = mean(diff(time)); % Sampling period for the original data
data_id1 = iddata(flow_rate, frequency, Ts1);

% Create an iddata object for the second dataset with frequency as input
Ts2 = mean(diff(time_step)); % Sampling period for the step response data
data_id2 = iddata(flow_rate_step, frequency_step, Ts2);

% Combine the datasets
combined_data = merge(data_id1, data_id2);

% Estimate a SISO transfer function model using the combined dataset
np = 2; % number of poles (adjust as needed)
nz = 0; % number of zeros (adjust as needed)
sys_combined = tfest(combined_data, np, nz); % Specify np for each input

% Display the estimated transfer function
disp('Estimated Transfer Function from Combined Data:');
sys_combined

% Compare the combined model response with the actual data
figure;
compare(combined_data, sys_combined);
title('Combined Model Response vs. Actual Data');

% Convert the identified transfer function to a state-space model
sys_primary_ss = ss(sys_combined);

% Create options for the pidtune function focusing on disturbance rejection
% and Phase Margin of at least 60 degrees
options = pidtuneOptions('DesignFocus', 'disturbance-rejection', 'PhaseMargin', 60);

% Tune the PID controller using the created options
C = pidtune(sys_primary_ss, 'PID', options);

% Display the PID parameters
disp('PID Controller:');
C

% Extract PID parameters (Initial Estimation of parameters)
%Kp = C.Kp;
%Ki = C.Ki;
%Kd = C.Kd;

% Finalised Parameters after tuning
Kp = 0.054;
Ki = 0.0000001;
Kd = 1.3;

% Define the time vector for the simulation
total_time = 1380; % Total simulation time in seconds
time_total = (0:1:total_time)';

% Initialize the output vectors
controlled_flow_rate = zeros(size(time_total));
flow_rate_with_clogging = zeros(size(time_total));
vfd_frequency = 30 * ones(size(time_total)); % Initial VFD frequency is 30 Hz

% Simulate the system for the first 240 seconds without control
initial_state = zeros(size(sys_primary_ss.A, 1), 1);
[plant_output, ~] = lsim(sys_primary_ss, vfd_frequency(1:241), time_total(1:241), initial_state);

% Store the initial part of the simulation
controlled_flow_rate(1:241) = plant_output;

% Initialize PID controller variables
integral_error = 0;
previous_error = 0;

% Simulate the system with PID control after 180 seconds
for k = 242:length(time_total)
    % Calculate the error
    error = target_flow_rate - controlled_flow_rate(k-1);
    
    % Update the integral of the error
    integral_error = integral_error + error * (time_total(k) - time_total(k-1));
    
    % Calculate the derivative of the error
    derivative_error = (error - previous_error) / (time_total(k) - time_total(k-1));
    
    % Calculate the control input using the PID formula
    control_input = Kp * error + Ki * integral_error + Kd * derivative_error;
    
    % Adjust the VFD frequency based on control input
    vfd_frequency(k) = vfd_frequency(k-1) + control_input;
    
    % Bound the VFD frequency between 30 Hz and 60 Hz
    if vfd_frequency(k) < 30
        vfd_frequency(k) = 30;
    elseif vfd_frequency(k) > 60
        vfd_frequency(k) = 60;
    end
    
    % Update the previous error
    previous_error = error;
    
    % Apply the disturbance effect after 480 seconds (8 minutes)
    if k > 480
        clogging_effect_sim = clogging_effect_normalized(k - 480);
    else
        clogging_effect_sim = target_flow_rate;
    end
    
    % Simulate the plant response with the accumulated control inputs
    [plant_output, ~] = lsim(sys_primary_ss, vfd_frequency(1:k), time_total(1:k), initial_state);
    
    % Apply the disturbance to the plant output
    controlled_flow_rate(k) = plant_output(end) - (target_flow_rate - clogging_effect_sim);
    flow_rate_with_clogging(k) = clogging_effect_sim;
end

% Plot the response
figure;
plot(time_total, controlled_flow_rate, 'r', 'DisplayName', 'Controlled Flow Rate');
hold on;
plot(time_total(time_total > 480), flow_rate_with_clogging(time_total > 480), 'b', 'DisplayName', 'Flow Rate with Clogging');
xlabel('Time (s)');
ylabel('Flow Rate (m^3/h)');
yline(target_flow_rate * 1.02, '--k', 'DisplayName', '2% Upper Bound');
yline(target_flow_rate * 0.98, '--k', 'DisplayName', '2% Lower Bound');
title('System Response with PID Control and Clogging');
legend;
hold off;


% Plot the VFD frequency evolution
figure;
plot(time_total, vfd_frequency, 'g', 'DisplayName', 'VFD Frequency');
xlabel('Time (s)');
ylabel('Frequency (Hz)');
title('VFD Frequency Evolution');
ylim([25 65]); % Set the y-axis limits from 20 Hz to 70 Hz
legend;
hold off;


% Plot the deviation from the target flow rate
figure;
deviation_from_target = controlled_flow_rate - target_flow_rate;
plot(time_total, deviation_from_target, 'r', 'DisplayName', 'Deviation from Target');
hold on;
yline(target_flow_rate * 0.02, '--k', 'DisplayName', '2% Upper Bound');
yline(-target_flow_rate * 0.02, '--k', 'DisplayName', '2% Lower Bound');
xlabel('Time (s)');
ylabel('Deviation from Target Flow Rate (m^3/h)');
title('Deviation from Target Flow Rate');
legend;
hold off;
