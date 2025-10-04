clear; clc; close all;
% ----------------------- parameters -----------------------
alpha = 1.5;      % [1/s^2]
tau   = 3;        % [s] % headway from your controller
dt    = 0.1;      % [s]
T     = 60;       % [s] duration
t     = 0:dt:T;
num_steps = numel(t);

% ----------------------- platoon initialization settings -------------------
num_cars = 10;
initial_spacing = 30; % [m]
initial_speed = 15;   % [m/s]

% ----------------------- scenario selection ------------------------
run_steady_state        = true;
run_sudden_stop_recover = true;
run_lead_sudden_slow    = true;

% ----------------------- define scenarios -----------------------
% Each scenario now just builds a speed profile for the lead car
scenarios = {};
if run_steady_state
    scenarios{end+1} = struct( ...
        "name","Steady-State", ...
        "build", @(t) piecewise_accel_profile(t, initial_speed, [0], [0]) ... % No acceleration
    );
end
if run_sudden_stop_recover
    scenarios{end+1} = struct( ...
        "name","Lead: Sudden Stop then Recover", ...
        "build", @(t) piecewise_accel_profile(t, initial_speed, [-4.0, +2.0], [3.0, 6.0]) ...
    );
end
if run_lead_sudden_slow
    scenarios{end+1} = struct( ...
        "name","Lead: Sudden Slowdown", ...
        "build", @(t) piecewise_accel_profile(t, initial_speed, [-2.5], [4.0]) ...
    );
end

% ----------------------- Main Scenario Loop -----------------------
fprintf('Running %d scenarios...\n', numel(scenarios));
for s_idx = 1:numel(scenarios)
    S = scenarios{s_idx};
    fprintf('\n--- Running Scenario: %s ---\n', S.name);

    % --- Reset the Platoon for the New Scenario ---
    cars(num_cars) = struct('pos', 0, 'vel', 0);
    for i = 1:num_cars
        cars(i).pos = - (i - 1) * initial_spacing;
        cars(i).vel = initial_speed;
    end

    % --- Store History for this Scenario ---
    pos_history = zeros(num_steps, num_cars);
    vel_history = zeros(num_steps, num_cars);
    pos_history(1, :) = [cars.pos];
    vel_history(1, :) = [cars.vel];

    % --- Get the Lead Car Behavior for this Scenario ---
    lead_car_speed_profile = S.build(t);

    % --- Multi-Car Simulation Loop for this Scenario ---
    for k = 2:num_steps
        cars(1).vel = lead_car_speed_profile(k);
        cars(1).pos = cars(1).pos + cars(1).vel * dt;
        
        for i = 2:num_cars
            ego_car = cars(i);
            lead_car = cars(i-1);
            space_gap = lead_car.pos - ego_car.pos;
            
            accel_cmd = wead_controller_accel_policy(ego_car.vel, space_gap, lead_car.vel, alpha, tau);
            
            cars(i).vel = ego_car.vel + accel_cmd * dt;
            cars(i).pos = ego_car.pos + ego_car.vel * dt;
        end
        
        pos_history(k, :) = [cars.pos];
        vel_history(k, :) = [cars.vel];
    end
    disp('Simulation for this scenario complete.');

    % --- Plotting Results for this Scenario ---
    figure('Name', S.name, 'Color', 'w');
    
    subplot(2, 1, 1);
    plot(t, pos_history, 'LineWidth', 1.5);
    title(['Position vs. Time (' S.name ')']);
    xlabel('Time (s)');
    ylabel('Position (m)');
    grid on;
    legend(arrayfun(@(n) sprintf('Car %d', n), 1:num_cars, 'UniformOutput', false), 'Location', 'eastoutside');
    
    subplot(2, 1, 2);
    plot(t, vel_history, 'LineWidth', 1.5);
    title(['Speed vs. Time (' S.name ')']);
    xlabel('Time (s)');
    ylabel('Speed (m/s)');
    grid on;
    legend(arrayfun(@(n) sprintf('Car %d', n), 1:num_cars, 'UniformOutput', false), 'Location', 'eastoutside');

end % End of main scenario loop
disp('All scenarios complete.');

% ----------------------- Helper Functions -----------------------
function a = wead_controller_accel_policy(vn, sn, lead_v, alpha, tau)
    TIME_GAP_ACCEL = 6.0;
    TIME_GAP_BRAKE = 3.0;
    
    if vn < 0.1
        time_gap = inf;
    else
        time_gap = sn / vn;
    end
    
    if time_gap > TIME_GAP_ACCEL
        a = 0.5;
    elseif time_gap > TIME_GAP_BRAKE
        a = 0;
    else
        a = -1.5;
    end
end

function v = piecewise_accel_profile(t, v0, accels, durations)
    dt = t(2) - t(1);
    v  = zeros(size(t));
    v(1) = v0;
    current_t_index = 1;
    for seg = 1:numel(accels)
        num_steps_in_segment = max(1, round(durations(seg) / dt));
        end_index = min(numel(t), current_t_index + num_steps_in_segment);
        a = accels(seg);
        for k = (current_t_index + 1):end_index
            v(k) = v(k-1) + a * dt;
        end
        if end_index < numel(t)
            v(end_index+1:end) = v(end_index);
        end
        current_t_index = end_index;
        if current_t_index >= numel(t), break; end
    end
    if current_t_index < numel(t)
        v(current_t_index:end) = v(current_t_index);
    end
end