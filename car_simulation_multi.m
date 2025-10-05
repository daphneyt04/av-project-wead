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
run_lead_speedup        = true;
run_lead_stop_hold      = true;
run_traffic_wave        = true;
run_cut_in              = true;
run_cut_out             = true;
run_lead_dropout        = true;
run_noise_injection     = true;


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

if run_lead_speedup
    scenarios{end+1} = struct( ...
        "name","Lead: Sudden Speed-Up", ...
        "build", @(t) piecewise_accel_profile(t, initial_speed, [2.0], [3.0]) ...
    );
end

if run_lead_stop_hold
    scenarios{end+1} = struct( ...
        "name","Lead: Stop and Hold", ...
        "build", @(t) piecewise_accel_profile(t, initial_speed, [-3.0, 0.0], [5.0, 10.0]) ...
    );
end

if run_traffic_wave
    scenarios{end+1} = struct( ...
        "name","Lead: Periodic Traffic Wave", ...
        "build", @(t) initial_speed + 3*sin(0.3*t) ...
    );
end

% (7) Cut-in event
if run_cut_in
    scenarios{end+1} = struct( ...
        "name","Cut-In Event", ...
        "build", @(t) initial_speed + 0*t, ...
        "cut_in_event", true ...
    );
end

if run_cut_out
    scenarios{end+1} = struct( ...
        "name","Cut-Out Event", ...
        "build", @(t) initial_speed + 0*t, ...
        "cut_out_event", true ...
    );
end

if run_lead_dropout
    scenarios{end+1} = struct( ...
        "name","Lead Vehicle Dropout (Sensor Failure)", ...
        "build", @(t) ((t>10 & t<15)*NaN) + ((t<=10 | t>=15)*initial_speed) ...
    );
end

if run_noise_injection
    scenarios{end+1} = struct( ...
        "name","Lead: Random Noise", ...
        "build", @(t) initial_speed + 0.5*randn(size(t)) ...
    );
end

% --- Create a single figure window and a tab group BEFORE the loop ---
main_fig = figure('Name', 'Multi-Scenario Simulation Results', 'Color', 'w', 'Position', [100, 100, 1200, 800]);
tab_group = uitabgroup(main_fig);

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
    accel_history = zeros(num_steps, num_cars - 1);
    pos_history(1, :) = [cars.pos];
    vel_history(1, :) = [cars.vel];

    % --- Get the Lead Car Behavior for this Scenario ---
    lead_car_speed_profile = S.build(t);

    % --- Multi-Car Simulation Loop for this Scenario ---
    for k = 2:num_steps
        
        % Handle NaNs (dropout)
        if isnan(lead_car_speed_profile(k))
            cars(1).vel = cars(1).vel; % hold last value
        else
            cars(1).vel = lead_car_speed_profile(k);
        end
        cars(1).pos = cars(1).pos + cars(1).vel * dt;
        
        % Handle cut-in / cut-out events
        if isfield(S, "cut_in_event") && S.cut_in_event && abs(t(k)-10) < dt/2
            cars(5).pos = cars(5).pos - 10; % vehicle cuts in closer
            fprintf('Cut-in event triggered at t=%.1fs\n', t(k));
        end
        if isfield(S, "cut_out_event") && S.cut_out_event && abs(t(k)-20) < dt/2
            cars(5).pos = cars(5).pos + 20; % vehicle cuts out
            fprintf('Cut-out event triggered at t=%.1fs\n', t(k));
        end

        for i = 2:num_cars
            ego_car = cars(i);
            lead_car = cars(i-1);
            space_gap = lead_car.pos - ego_car.pos;
            
            accel_cmd = wead_controller_accel_policy(ego_car.vel, space_gap, lead_car.vel, alpha, tau);
            accel_history(k, i-1) = accel_cmd;

            cars(i).vel = ego_car.vel + accel_cmd * dt;
            cars(i).pos = ego_car.pos + ego_car.vel * dt;

            
        end
        
        pos_history(k, :) = [cars.pos];
        vel_history(k, :) = [cars.vel];
    end
    
    disp('Simulation for this scenario complete.');


    % --- Plotting Results for this Scenario ---
    % First, calculate the space gap history
    space_gap_history = pos_history(:, 1:end-1) - pos_history(:, 2:end);

    % Create a new title string with the figure number
    new_title = sprintf('Figure %d: %s', s_idx, S.name);
    
    % Create the tab using the new title
    scenario_tab = uitab(tab_group, 'Title', new_title);
    
    % Direct the tiled layout to the current tab
    t_layout = tiledlayout(scenario_tab, 2, 2, 'Padding', 'compact', 'TileSpacing', 'compact');
    
    % Plot 1: Position vs. Time
    ax1 = nexttile(t_layout);
    plot(ax1, t, pos_history, 'LineWidth', 1.5);
    title(ax1, 'Position');
    xlabel(ax1, 'Time (s)');
    ylabel(ax1, 'Position (m)');
    grid(ax1, 'on');

    % Plot 2: Speed vs. Time
    ax2 = nexttile(t_layout);
    plot(ax2, t, vel_history, 'LineWidth', 1.5);
    title(ax2, 'Speed');
    xlabel(ax2, 'Time (s)');
    ylabel(ax2, 'Speed (m/s)');
    grid(ax2, 'on');
    legend(ax2, arrayfun(@(n) sprintf('Car %d', n), 1:num_cars, 'UniformOutput', false), 'Location', 'eastoutside');

    % Plot 3: Space Gap vs. Time
    ax3 = nexttile(t_layout);
    plot(ax3, t, space_gap_history, 'LineWidth', 1.5);
    title(ax3, 'Space Gap');
    xlabel(ax3, 'Time (s)');
    ylabel(ax3, 'Space Gap (m)');
    grid(ax3, 'on');

    % Plot 4: Commanded Acceleration vs. Time
    ax4 = nexttile(t_layout);
    plot(ax4, t, accel_history, 'LineWidth', 1.5); % Assumes you stored accel_history
    title(ax4, 'Commanded Acceleration');
    xlabel(ax4, 'Time (s)');
    ylabel(ax4, 'Acceleration (m/s^2)');
    grid(ax4, 'on');
    legend(ax4, arrayfun(@(n) sprintf('Car %d', n), 2:num_cars, 'UniformOutput', false), 'Location', 'eastoutside');
    
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