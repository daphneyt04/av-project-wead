clear; clc;

% ----------------------- parameters -----------------------
alpha = 1.5;      % [1/s^2]
tau   = 0.5;      % [s] % headway
dt    = 0.1;      % [s]
T     = 30;       % [s] duration
t     = 0:dt:(T - dt);

PLOT_EACH = true;   % set false to skip plotting per-scenario
SAVE_FIGS = false;  % set true to save figures (PNG) in current folder

% ----------------------- scenarios ------------------------
run_lead_slow

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
