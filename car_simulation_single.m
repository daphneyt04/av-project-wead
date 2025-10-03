%% car_simulation_suite.m
% Run many car-following test cases from one file.
clear; clc;

% ----------------------- parameters -----------------------
alpha = 1.5;      % [1/s^2]
tau   = 0.5;      % [s] % headway
dt    = 0.1;      % [s]
T     = 30;       % [s] duration
t     = 0:dt:(T - dt);

PLOT_EACH = false;   % set false to skip plotting per-scenario
SAVE_FIGS = false;  % set true to save figures (PNG) in current folder

% ----------------------- scenarios ------------------------
run_steady_state        = true; % steady state where lead and ego are the same
run_sudden_stop_recover = true;  % "lead car comes to sudden stop and then recover"
run_lead_sudden_slow    = true;  % "lead car suddenly slows down" (decel then hold)
run_lead_sudden_speed   = true;  % "lead car suddenly starts to speed up" (accel then hold)
run_sn_opening_gap      = true;  % "sn shows opening gap" (lead > ego at start)
run_sn_closing_gap      = true;  % "sn is closing gap"   (ego > lead at start)
run_sn_too_small        = true;  % "sn is too small"     (gap < desired at equal speeds)
run_sn_too_large        = true;  % "sn is too large"     (gap > desired at equal speeds)
run_vn_too_fast         = true;  % "vn is faster than desired space gap" (v > s/tau)
run_vn_too_slow         = true;  % "vn is slower than desired space gap" (v < s/tau)


% ----------------------- potential scenarios -----------------------
scenarios = {};

% 1) steady state 
if run_steady_state
    scenarios{end+1} = struct( ...
        "name","Steady-State (baseline)", ...
        "build",@(t) deal( ...
            ones(size(t))*11, ...    % lead_speeds = constant
            11, ...                  % initial_ego_speed (v0)
            11*tau ...               % initial_space_gap (desired) (s0)
        ) ...
    );
end

% 2) lead sudden stop, then recover (piecewise accel: -3 for 2s, +1.5 for 4s)
if run_sudden_stop_recover
    scenarios{end+1} = struct( ...
        "name","Lead: sudden stop then recover", ...
        "build",@(t) build_piecewise_case(t, 11, [-3.0, +1.5], [2.0, 4.0], 11*tau) ...
    );
end

% 3) lead suddenly slows down (single decel, then hold)
if run_lead_sudden_slow
    scenarios{end+1} = struct( ...
        "name","Lead: sudden slowdown", ...
        "build",@(t) build_piecewise_case(t, 11, [-2.5], [2.0], 11*tau) ...
    );
end

% 4) lead suddenly starts to speed up (single accel, then hold)
if run_lead_sudden_speed
    scenarios{end+1} = struct( ...
        "name","Lead: sudden speed-up", ...
        "build",@(t) build_piecewise_case(t, 11, [+2.0], [2.0], 11*tau) ...
    );
end

% 5) s_n opening gap (lead > ego initially)
if run_sn_opening_gap
    scenarios{end+1} = struct( ...
        "name","s_n opening (lead faster than ego)", ...
        "build",@(t) deal( ...
            ones(size(t))*12, ...    % lead faster
            10, ...                  % ego slower
            10*tau ...               % start at ego's desired gap
        ) ...
    );
end

% 6) s_n closing gap (ego > lead initially)
if run_sn_closing_gap
    scenarios{end+1} = struct( ...
        "name","s_n closing (ego faster than lead)", ...
        "build",@(t) deal( ...
            ones(size(t))*9, ...     % lead slower
            12, ...                  % ego faster
            12*tau ...               % start at ego's desired gap
        ) ...
    );
end

% 7) s_n too small (equal speeds; gap < desired)
if run_sn_too_small
    scenarios{end+1} = struct( ...
        "name","s_n too small (equal speeds)", ...
        "build",@(t) deal( ...
            ones(size(t))*11, ...    % both at 11 m/s
            11, ...                  % ego same speed
            11*tau - 2.0 ...         % 2 m too close
        ) ...
    );
end

% 8) s_n too large (equal speeds; gap > desired)
if run_sn_too_large
    scenarios{end+1} = struct( ...
        "name","s_n too large (equal speeds)", ...
        "build",@(t) deal( ...
            ones(size(t))*11, ...    % both at 11 m/s
            11, ...                  % ego same speed
            11*tau + 3.0 ...         % 3 m too far
        ) ...
    );
end

% 9) v_n faster than desired space gap (v > s/tau)
if run_vn_too_fast
    scenarios{end+1} = struct( ...
        "name","v_n too fast for current gap (v > s/tau)", ...
        "build",@(t) deal( ...
            ones(size(t))*11, ...    % lead constant
            14, ...                  % ego too fast
            5.0 ...                  % small gap => s/tau = 10, but v0 = 14 > 10
        ) ...
    );
end

% 10) v_n slower than desired space gap (v < s/tau)
if run_vn_too_slow
    scenarios{end+1} = struct( ...
        "name","v_n too slow for current gap (v < s/tau)", ...
        "build",@(t) deal( ...
            ones(size(t))*11, ...    % lead constant
            6, ...                   % ego too slow
            8.0 ...                  % large gap => s/tau = 16, but v0 = 6 < 16
        ) ...
    );
end



% ----------------------- run scenarios -----------------------
fprintf('running %d scenarios (alpha=%.3g, tau=%.3g, dt=%.3g, T=%.1f)\n', ...
    numel(scenarios), alpha, tau, dt, T);

for i = 1:numel(scenarios)
    S = scenarios{i};
    [lead_speeds, v0, s0] = S.build(t);
    [tt, s, v, vl] = simulate_car_following(t, alpha, tau, v0, s0, lead_speeds);

    % metrics & logs
    m = compute_metrics(tt, s, v, vl, alpha, tau);
    print_metrics(S.name, m);

    % safety flags
    if any(~isfinite(v)) || any(~isfinite(s))
        warning('[%s] incorrect values detected.', S.name);
    end
    if min(s) < 0
        warning('[%s] crash detected (space gap < 0).', S.name);
    end

    % Plot per-scenario
    if PLOT_EACH
        fig = figure('Name', S.name, 'Color', 'w');
        tiledlayout(3,1);

        nexttile; plot(tt, v, tt, vl, '--', 'LineWidth', 1.2);
        ylabel('Speed (m/s)'); legend('Ego','Lead'); grid on; title(S.name);

        nexttile; plot(tt, s, 'LineWidth', 1.2);
        ylabel('Space gap (m)'); grid on;

        a_cmd = accel_trace(v, s, alpha, tau);
        nexttile; plot(tt, a_cmd, 'LineWidth', 1.2);
        ylabel('Cmd accel (m/s^2)'); xlabel('Time (s)'); grid on;

        if SAVE_FIGS
            fn = sprintf('fig_%02d_%s.png', i, sanitize_filename(S.name));
            exportgraphics(fig, fn, 'Resolution', 150);
        end
    end
end

disp('All scenarios complete.');

% helper functions!

% builds lead speed vector by integrating accelerations
function [lead_speeds, v0, s0] = build_piecewise_case(t, Vn, accels, durations, s0)

    lead_speeds = piecewise_accel_profile(t, Vn, accels, durations);
    v0 = Vn; % change to change start ego speed
end % returns s0

function name = sanitize_filename(s) % become safe file titles
    name = regexprep(s, '[^\w\-]+', '_');
end

function print_metrics(name, m)
    fprintf('\n== %s ==\n', name);
    fprintf(' final: v=%.3g m/s, s=%.3g m\n', m.v_end, m.s_end);
    fprintf(' cmd accel: mean=%.3g, min=%.3g, max=%.3g m/s^2\n', m.a_mean, m.a_min, m.a_max);
    fprintf(' %%time s < s_min(2m): %.2f%%\n', 100*m.pct_gap_viol);
end

function m = compute_metrics(t, s, v, vl, alpha, tau)
% Basic metrics for quick comparisons across scenarios.
    e = s - v.*tau;                    % time-gap error
    a = accel_trace(v, s, alpha, tau); % commanded accel

    m.v_end = v(end);
    m.s_end = s(end);
    m.e_rms = sqrt(mean(e.^2));
    m.e_max = max(abs(e));

    m.a_mean = mean(a);
    m.a_min  = min(a);
    m.a_max  = max(a);

    s_min = 2;                         % example safety floor [m]
    m.pct_gap_viol = mean(s < s_min);
end

function a = accel_trace(v_ego, s, alpha, tau)
% Commanded acceleration over time from the policy.
    a = alpha .* (s - v_ego .* tau);
end

function a = accel_policy(vn, sn, rel_v, alpha, tau)
%ACCEL_POLICY Commanded acceleration based on ego speed, gap, and relative speed.
%
% Inputs:
%   vn    = ego (follower) speed [m/s]
%   sn    = space gap to leader [m]
%   rel_v = relative speed (v_lead - v_ego) [m/s]
%   alpha = controller gain [1/s^2]
%   tau   = desired time gap [s]
%
% Output:
%   a     = commanded acceleration [m/s^2]

    % Standard constant time-gap policy (doesn't yet use rel_v explicitly)
    a = alpha * (sn - vn * tau);

    % 🔧 Note: rel_v is here so you can easily extend the law later
    % e.g., a = alpha*(sn - vn*tau) + beta*rel_v
end


function v = piecewise_accel_profile(t, v0, accels, durations)
% Lead speed from piecewise-constant accelerations.
    dt = t(2) - t(1);
    v  = zeros(size(t));
    v(1) = v0;

    idx = 1;
    for seg = 1:numel(accels)
        steps = max(1, round(durations(seg) / dt));
        i2    = min(numel(t), idx + steps - 1);
        a     = accels(seg);

        for k = idx+1:i2
            v(k) = v(k-1) + a * (t(k) - t(k-1));
        end

        if i2 < numel(t)
            v(i2+1:end) = v(i2);   % hold until next segment
        end

        idx = i2 + 1;
        if idx > numel(t), break; end
    end

    if idx <= numel(t)
        v(idx:end) = v(idx-1);     % hold after last segment
    end
end

function [time_steps, space_gaps, ego_speeds, lead_speeds] = simulate_car_following( ...
    time_steps, alpha, tau, initial_ego_speed, initial_space_gap, lead_speeds)
% Euler integration of the car-following model.
    dt = time_steps(2) - time_steps(1);
    N  = numel(time_steps);

    space_gaps = zeros(1, N);
    ego_speeds = zeros(1, N);

    space_gaps(1) = initial_space_gap;
    ego_speeds(1) = initial_ego_speed;

    for k = 2:N
        s_prev = space_gaps(k-1);
        v_prev = ego_speeds(k-1);
        v_lead = lead_speeds(k-1);

        rel_speed = v_lead - v_prev;                 % ds/dt
        acc       = accel_policy(v_prev, s_prev, rel_speed, alpha, tau);

        ego_speeds(k) = v_prev + dt * acc;
        space_gaps(k) = s_prev + dt * rel_speed;
    end
end
