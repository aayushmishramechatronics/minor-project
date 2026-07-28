%% simulate_pid.m
%
% Phase 2 — PID Baseline Controller — Validation Script
% =========================================================
% PURPOSE:
%   Runs the PID controller from pid_closed_loop.m against three
%   standard test cases and checks the results against the project's
%   official target specification:
%
%       Overshoot        <= 15%
%       Settling time    <=  2 s
%       Steady-state err <=  1 deg
%       Disturbance test :  0.01 N*m step torque
%
% REQUIRED FILES IN THE SAME FOLDER:
%   parameters.m         (Phase 1)
%   dynamics.m            (Phase 1)
%   pid_gains.m           (Phase 2)
%   pid_closed_loop.m     (Phase 2)
%   step_metrics.m        (Phase 2)
%
% HOW TO RUN:
%   >> simulate_pid
%
% WHAT THIS SCRIPT DOES:
%   TEST 1 — Step response (roll axis only, isolated).
%            A clean single-axis test with no coupling active.
%            This is the baseline PID performance number.
%
%   TEST 2 — Disturbance rejection (roll axis, at hover).
%            The drone holds level, then a constant torque
%            disturbance is injected. Measures how far it deviates
%            and how long it takes to recover.
%
%   TEST 3 — Coupled-axes test (roll + pitch commanded together,
%            with a nonzero initial yaw rate).
%            This activates the gyroscopic coupling terms and shows
%            how PID performance degrades compared to the clean
%            single-axis case in Test 1 — the core motivation for
%            moving to Sliding Mode Control in Phase 3.
% =========================================================

clear; clc; close all;

% ─────────────────────────────────────────────────────────────────────────
%  STEP 1: Load parameters and gains
% ─────────────────────────────────────────────────────────────────────────
parameters;
pid_gains;

% ODE solver options — same tight tolerances as Phase 1.
opts = odeset('RelTol', 1e-8, 'AbsTol', 1e-10);

% Official target specification (from the project objectives).
TARGET_OVERSHOOT_PCT = 15;
TARGET_SETTLING_S    = 2.0;
TARGET_SS_ERROR_DEG  = 1.0;


% =========================================================================
%  TEST 1 — STEP RESPONSE  (roll axis, isolated, no coupling active)
% =========================================================================
fprintf('\nRunning Test 1 (step response, roll axis)...\n');

ref1  = [deg2rad(10); 0; 0];    % command a 10-degree roll step, hold pitch/yaw at 0

dist1.active  = false;          % no disturbance in this test
dist1.t_start = 0;
dist1.torque  = [0; 0; 0];

x0_1     = zeros(9, 1);         % start at rest, level, zero integral states
t_span1  = [0, 3];

odefun1  = @(t, x) pid_closed_loop(t, x, ref1, dist1, p, g);
[t1, x1] = ode45(odefun1, t_span1, x0_1, opts);

phi1     = x1(:, 1);
phi1_deg = rad2deg(phi1);

% Recompute the commanded torque time-history for plotting
% (not directly output by ode45, so reconstructed from the state history).
e_phi1        = ref1(1) - x1(:, 1);
tau_phi1_raw  = g.roll.Kp * e_phi1 + g.roll.Ki * x1(:, 7) - g.roll.Kd * x1(:, 2);
tau_phi1      = max(-g.roll.tau_max, min(g.roll.tau_max, tau_phi1_raw));

metrics1 = step_metrics(t1, phi1, ref1(1), 2);   % 2% settling band
ss_error1_deg = abs(rad2deg(ref1(1) - phi1(end)));

fprintf('  Done. %d time steps computed.\n', length(t1));


% =========================================================================
%  TEST 2 — DISTURBANCE REJECTION  (roll axis, hovering level)
% =========================================================================
fprintf('Running Test 2 (disturbance rejection, roll axis)...\n');

ref2 = [0; 0; 0];                % hold level — no commanded angle

dist2.active  = true;
dist2.t_start = 2.0;             % disturbance begins at t = 2 s
dist2.torque  = [0.01; 0; 0];    % 0.01 N*m step torque on roll axis

x0_2     = zeros(9, 1);
t_span2  = [0, 5];

odefun2  = @(t, x) pid_closed_loop(t, x, ref2, dist2, p, g);
[t2, x2] = ode45(odefun2, t_span2, x0_2, opts);

phi2     = x2(:, 1);
phi2_deg = rad2deg(phi2);

% ── Compute max deviation and recovery time (custom metric — not a
%    standard step response, since the "target" is 0 both before and
%    after the disturbance) ─────────────────────────────────────────────
recovery_threshold = deg2rad(1);     % "recovered" = within 1 degree of level

after_mask = t2 >= dist2.t_start;
t_after    = t2(after_mask);
phi_after  = phi2(after_mask);

[max_dev_rad, idx_peak] = max(abs(phi_after));
max_dev_deg = rad2deg(max_dev_rad);

recovery_time_s = NaN;
for k = idx_peak:length(phi_after)
    if all(abs(phi_after(k:end)) < recovery_threshold)
        recovery_time_s = t_after(k) - dist2.t_start;
        break;
    end
end

fprintf('  Done. %d time steps computed.\n', length(t2));


% =========================================================================
%  TEST 3 — COUPLED AXES  (roll + pitch commanded together,
%           with a nonzero initial yaw rate to excite gyroscopic coupling)
% =========================================================================
fprintf('Running Test 3 (coupled axes, roll + pitch + initial yaw rate)...\n');

ref3 = [deg2rad(15); deg2rad(15); 0];   % command both roll and pitch to 15 deg

dist3.active  = false;
dist3.t_start = 0;
dist3.torque  = [0; 0; 0];

x0_3     = zeros(9, 1);
x0_3(6)  = 0.5;                  % initial yaw rate = 0.5 rad/s (~28.6 deg/s)
                                  % This is what activates the coupling terms
                                  % in the roll and pitch equations.
t_span3  = [0, 3];

odefun3  = @(t, x) pid_closed_loop(t, x, ref3, dist3, p, g);
[t3, x3] = ode45(odefun3, t_span3, x0_3, opts);

phi3       = x3(:, 1);   phi3_deg   = rad2deg(phi3);
theta3     = x3(:, 3);   theta3_deg = rad2deg(theta3);
psi3       = x3(:, 5);   psi3_deg   = rad2deg(psi3);
psi_dot3   = x3(:, 6);

metrics3_roll  = step_metrics(t3, phi3,   ref3(1), 2);
metrics3_pitch = step_metrics(t3, theta3, ref3(2), 2);

fprintf('  Done. %d time steps computed.\n\n', length(t3));


% =========================================================================
%  FIGURE 1 — Test 1: Step Response
% =========================================================================
c_blue  = [0.09 0.37 0.78];
c_black = [0.10 0.10 0.10];
c_red   = [0.82 0.13 0.13];
c_green = [0.10 0.50 0.15];
c_gray  = [0.55 0.55 0.55];

fig1 = figure('Name', 'Phase 2 — Test 1: Step Response', ...
              'NumberTitle', 'off', 'Position', [30 50 1000 420]);

subplot(1, 2, 1);
plot(t1, phi1_deg, 'Color', c_blue, 'LineWidth', 2.5);
hold on;
yline(rad2deg(ref1(1)), '--', 'Color', c_black, 'LineWidth', 1.5);
hold off;
xlabel('Time  [s]');  ylabel('\phi  [deg]');
title('Roll angle — Step response to 10\circ command');
legend('PID response', 'Reference (10\circ)', 'Location', 'southeast');
grid on; box off;

subplot(1, 2, 2);
plot(t1, tau_phi1, 'Color', c_blue, 'LineWidth', 2);
xlabel('Time  [s]');  ylabel('\tau_\phi  [N*m]');
title('Commanded roll torque');
grid on; box off;

sgtitle('Phase 2 — PID Test 1: Step Response', 'FontSize', 13, 'FontWeight', 'bold');


% =========================================================================
%  FIGURE 2 — Test 2: Disturbance Rejection
% =========================================================================
fig2 = figure('Name', 'Phase 2 — Test 2: Disturbance Rejection', ...
              'NumberTitle', 'off', 'Position', [80 100 700 420]);

plot(t2, phi2_deg, 'Color', c_blue, 'LineWidth', 2.5);
hold on;
xline(dist2.t_start, '--', 'Color', c_gray, 'LineWidth', 1.5);
yline(0, ':', 'Color', c_black, 'LineWidth', 1);
hold off;
xlabel('Time  [s]');  ylabel('\phi  [deg]');
title({'Roll angle — 0.01 N*m disturbance torque injected at t = 2s', ...
       sprintf('Max deviation: %.2f deg   |   Recovery time: %s', ...
               max_dev_deg, format_recovery(recovery_time_s))});
legend('PID response', 'Disturbance onset', 'Location', 'southeast');
grid on; box off;


% =========================================================================
%  FIGURE 3 — Test 3: Coupled Axes
% =========================================================================
fig3 = figure('Name', 'Phase 2 — Test 3: Coupled Axes', ...
              'NumberTitle', 'off', 'Position', [130 150 1000 420]);

subplot(1, 2, 1);
plot(t3, phi3_deg,   'Color', c_blue, 'LineWidth', 2.2); hold on;
plot(t3, theta3_deg, 'Color', c_red,  'LineWidth', 2.2);
yline(15, '--', 'Color', c_black, 'LineWidth', 1.2);
hold off;
xlabel('Time  [s]');  ylabel('Angle  [deg]');
title('Roll and pitch tracking 15\circ commands (with initial yaw rate)');
legend('\phi (roll)', '\theta (pitch)', 'Reference (15\circ)', 'Location', 'southeast');
grid on; box off;

subplot(1, 2, 2);
plot(t3, psi3_deg, 'Color', c_green, 'LineWidth', 2.2); hold on;
yyaxis right;
plot(t3, rad2deg(psi_dot3), '--', 'Color', c_gray, 'LineWidth', 1.5);
ylabel('\psi_{dot}  [deg/s]');
yyaxis left;
ylabel('\psi  [deg]');
hold off;
xlabel('Time  [s]');
title('Yaw response  (\tau_\psi commanded = 0 throughout)');
legend('\psi (angle)', '\psi_{dot} (rate)', 'Location', 'northeast');
grid on; box off;

sgtitle('Phase 2 — PID Test 3: Coupled-Axes Demonstration', 'FontSize', 13, 'FontWeight', 'bold');


% =========================================================================
%  CONSOLE VALIDATION SUMMARY
% =========================================================================
fprintf('============================================================\n');
fprintf('  PHASE 2 — PID VALIDATION RESULTS\n');
fprintf('============================================================\n\n');

% ── Test 1 summary and pass/fail ───────────────────────────────────────
fprintf('  TEST 1 — Step Response (isolated roll axis)\n');
fprintf('  ---------------------------------------------\n');
fprintf('    Overshoot          : %.2f %%\n', metrics1.overshoot_pct);
print_settle('    Settling time      : ', metrics1.settling_time);
fprintf('    Steady-state error : %.4f deg\n', ss_error1_deg);
fprintf('\n');

check1a = metrics1.overshoot_pct <= TARGET_OVERSHOOT_PCT;
check1b = ~isnan(metrics1.settling_time) && metrics1.settling_time <= TARGET_SETTLING_S;
check1c = ss_error1_deg <= TARGET_SS_ERROR_DEG;

print_check('    Overshoot <= 15%%        : ', check1a);
print_check('    Settling time <= 2s     : ', check1b);
print_check('    SS error <= 1 deg       : ', check1c);
fprintf('\n');

% ── Test 2 summary ──────────────────────────────────────────────────────
fprintf('  TEST 2 — Disturbance Rejection\n');
fprintf('  ---------------------------------------------\n');
fprintf('    Disturbance torque      : %.4f N*m\n', dist2.torque(1));
fprintf('    Max deviation           : %.2f deg\n', max_dev_deg);
fprintf('    Recovery time           : %s\n', format_recovery(recovery_time_s));
fprintf('    (This number is the PID baseline. SMC in Phase 3\n');
fprintf('     will be compared directly against it.)\n\n');

% ── Test 3 summary and comparison to Test 1 ─────────────────────────────
fprintf('  TEST 3 — Coupled Axes (vs. Test 1 isolated-axis baseline)\n');
fprintf('  -------------------------------------------------------------\n');
fprintf('    %-24s %10s   %10s\n', 'Metric', 'Test 1', 'Test 3 (roll)');
fprintf('    %-24s %10.2f %% %9.2f %%\n', 'Overshoot', ...
        metrics1.overshoot_pct, metrics3_roll.overshoot_pct);
fprintf('    %-24s %9.3f s %9.3f s\n', 'Settling time', ...
        metrics1.settling_time, metrics3_roll.settling_time);
fprintf('\n');
fprintf('    Pitch axis (Test 3)     : overshoot %.2f %%, settling %.3f s\n', ...
        metrics3_pitch.overshoot_pct, metrics3_pitch.settling_time);
fprintf('    Max yaw drift observed  : %.3f deg  (tau_psi commanded = 0)\n', ...
        max(abs(psi3_deg)));
fprintf('\n');

if metrics3_roll.overshoot_pct > metrics1.overshoot_pct + 0.5
    fprintf('    OBSERVATION: Overshoot increased under coupling.\n');
    fprintf('    This is the gyroscopic effect degrading PID performance —\n');
    fprintf('    the motivation for Sliding Mode Control in Phase 3.\n');
else
    fprintf('    OBSERVATION: Coupling effect is small at this yaw rate.\n');
    fprintf('    Try increasing x0_3(6) to see a larger effect.\n');
end

fprintf('\n============================================================\n');
if check1a && check1b && check1c
    fprintf('  OVERALL: Test 1 meets all official target specifications.\n');
    fprintf('  PID baseline is ready. Proceed to Phase 3 (SMC design).\n');
else
    fprintf('  OVERALL: Test 1 does NOT meet all target specifications.\n');
    fprintf('  Revisit wn and zeta in pid_gains.m before proceeding.\n');
end
fprintf('============================================================\n\n');


% =========================================================================
%  LOCAL HELPER FUNCTIONS
%  (MATLAB allows local functions at the end of a script file.)
% =========================================================================
function print_check(label, passed)
    if passed
        fprintf('%sPASS\n', label);
    else
        fprintf('%sFAIL\n', label);
    end
end

function print_settle(label, settle_val)
    if isnan(settle_val)
        fprintf('%sDID NOT SETTLE within simulation window\n', label);
    else
        fprintf('%s%.3f s\n', label, settle_val);
    end
end

function str = format_recovery(recovery_val)
    if isnan(recovery_val)
        str = 'DID NOT RECOVER within simulation window';
    else
        str = sprintf('%.3f s', recovery_val);
    end
end
