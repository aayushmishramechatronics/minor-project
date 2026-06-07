%% simulate_openloop.m
%
% Phase 1 — Open-Loop Model Validation
% =========================================================
% PURPOSE:
%   Verify that dynamics.m is physically correct BEFORE adding
%   any controller. If any check below fails, fix dynamics.m
%   first. Do NOT proceed to Phase 2 with a broken model.
%
% HOW TO RUN:
%   1. Open MATLAB
%   2. Make sure parameters.m and dynamics.m are in the same folder
%      as this file (or on the MATLAB path)
%   3. Run this script: press F5 or type >> simulate_openloop
%
% WHAT THIS SCRIPT DOES:
%   Runs TWO separate test cases, each with a different purpose.
%
%   TEST CASE 1 — Pure roll torque from rest:
%     Applies only roll torque with all other inputs at zero.
%     Starting from a completely stationary drone.
%     Purpose: Verify that the angular acceleration matches the
%     theoretical value tau_phi / Ixx, and that the simulated roll
%     angle follows the kinematic prediction 0.5*(tau/I)*t^2.
%     For a symmetric drone (Ixx = Iyy), yaw and pitch stay at zero
%     in this test — that is CORRECT, not a bug.
%
%   TEST CASE 2 — Roll torque with initial yaw rate:
%     Same roll torque, but the drone starts with a nonzero yaw rate.
%     This activates the pitch coupling term (Izz-Ixx)*phi_dot*psi_dot.
%     Purpose: Demonstrate that the nonlinear coupling is present and
%     working — pitch responds even though tau_theta = 0.
%
% THREE VALIDATION CHECKS:
%   CHECK 1: Angular acceleration accuracy   (Test 1)
%   CHECK 2: Gyroscopic coupling active      (Test 2)
%   CHECK 3: Kinematic prediction match      (Test 1, first second)
% =========================================================

clear; clc; close all;

% ─────────────────────────────────────────────────────────
%  STEP 1: Load parameters
%  This puts the struct 'p' into the workspace.
% ─────────────────────────────────────────────────────────
parameters;

% ─────────────────────────────────────────────────────────
%  STEP 2: Shared simulation settings
% ─────────────────────────────────────────────────────────
t_span = [0, 4];            % simulate for 4 seconds

% ODE solver options.
% RelTol and AbsTol control how accurate each integration step must be.
% The defaults (RelTol=1e-3) are too loose for this system.
% The gyroscopic coupling terms start small and get missed with loose tolerances.
% Always use tight tolerances for nonlinear systems.
opts = odeset('RelTol', 1e-8, 'AbsTol', 1e-10);

% Control torque applied in both test cases [N*m]
%   u(1) = tau_phi   = 0.005 N*m  (5 milli-Newton-metres of roll torque)
%   u(2) = tau_theta = 0           (no pitch torque commanded)
%   u(3) = tau_psi   = 0           (no yaw torque commanded)
u_test = [0.005; 0; 0];

% =========================================================
%  TEST CASE 1: Pure roll torque, drone starts from rest
% =========================================================
fprintf('\nRunning Test Case 1 (pure roll torque from rest)...\n');

x0_test1 = zeros(6, 1);    % all angles = 0, all rates = 0

odefun1 = @(t, x) dynamics(t, x, u_test, p);
[t1, x1] = ode45(odefun1, t_span, x0_test1, opts);

% Extract states from Test 1
phi1       = x1(:, 1);     % roll  angle [rad]
phi_dot1   = x1(:, 2);     % roll  rate  [rad/s]
theta1     = x1(:, 3);     % pitch angle [rad]
theta_dot1 = x1(:, 4);     % pitch rate  [rad/s]
psi1       = x1(:, 5);     % yaw   angle [rad]
psi_dot1   = x1(:, 6);     % yaw   rate  [rad/s]

% Convert to degrees for plotting only
phi1_deg       = rad2deg(phi1);
phi_dot1_deg   = rad2deg(phi_dot1);
theta1_deg     = rad2deg(theta1);
theta_dot1_deg = rad2deg(theta_dot1);
psi1_deg       = rad2deg(psi1);
psi_dot1_deg   = rad2deg(psi_dot1);

fprintf('  Done. %d time steps computed.\n', length(t1));

% =========================================================
%  TEST CASE 2: Roll torque, drone starts with yaw rate
% =========================================================
fprintf('Running Test Case 2 (roll torque + initial yaw rate)...\n');

x0_test2      = zeros(6, 1);
x0_test2(6)   = 0.5;       % initial yaw rate = 0.5 rad/s (~28.6 deg/s)
                            % Physical meaning: the drone is slowly spinning
                            % in yaw when a roll torque is applied.
                            % This activates the pitch coupling term
                            % (Izz - Ixx) * phi_dot * psi_dot.

odefun2 = @(t, x) dynamics(t, x, u_test, p);
[t2, x2] = ode45(odefun2, t_span, x0_test2, opts);

% Extract states from Test 2
phi2       = x2(:, 1);     % roll  angle [rad]
phi_dot2   = x2(:, 2);     % roll  rate  [rad/s]
theta2     = x2(:, 3);     % pitch angle [rad]
theta_dot2 = x2(:, 4);     % pitch rate  [rad/s]
psi2       = x2(:, 5);     % yaw   angle [rad]
psi_dot2   = x2(:, 6);     % yaw   rate  [rad/s]

% Convert to degrees for plotting only
phi2_deg       = rad2deg(phi2);
phi_dot2_deg   = rad2deg(phi_dot2);
theta2_deg     = rad2deg(theta2);
theta_dot2_deg = rad2deg(theta_dot2);
psi2_deg       = rad2deg(psi2);
psi_dot2_deg   = rad2deg(psi_dot2);

fprintf('  Done. %d time steps computed.\n\n', length(t2));

% ─────────────────────────────────────────────────────────
%  ANALYTICAL PREDICTION (for Check 1 and Check 3)
%
%  If there is no coupling, roll angle grows as a pure parabola:
%    phi(t) = 0.5 * (tau_phi / Ixx) * t^2
%  This is the kinematic solution: integrate constant acceleration twice.
%
%  In Test 1 (symmetric drone, pure roll), coupling is zero so the
%  simulated phi should match this prediction almost exactly.
% ─────────────────────────────────────────────────────────
alpha_expected   = u_test(1) / p.Ixx;           % expected angular accel [rad/s^2]
phi_analytical   = 0.5 * alpha_expected * t1.^2;% kinematic prediction    [rad]
phi_analytical_deg = rad2deg(phi_analytical);    % in degrees for plotting

% =========================================================
%  FIGURE 1: Test Case 1 Results (6 panels)
% =========================================================
fig1 = figure('Name', 'Phase 1 — Test 1: Pure Roll from Rest', ...
              'NumberTitle', 'off', ...
              'Position', [30 50 1200 720]);

% Colours
c_blue  = [0.09 0.37 0.78];
c_black = [0.10 0.10 0.10];
c_red   = [0.82 0.13 0.13];
c_green = [0.10 0.50 0.15];
c_gray  = [0.55 0.55 0.55];

% ── Panel 1: Roll angle with analytical overlay ──────────────────────────
subplot(3, 2, 1);
plot(t1, phi1_deg, 'Color', c_blue, 'LineWidth', 2.5);
hold on;
plot(t1, phi_analytical_deg, '--', 'Color', c_black, 'LineWidth', 1.8);
hold off;
xlabel('Time  [s]',      'FontSize', 11);
ylabel('\phi  [deg]',    'FontSize', 11);
title('Roll angle  \phi', 'FontSize', 12);
legend('ode45 simulation', '0.5 \cdot (\tau/I_{xx}) \cdot t^2', ...
       'Location', 'northwest', 'FontSize', 9);
grid on; box off;

% ── Panel 2: Roll rate ───────────────────────────────────────────────────
subplot(3, 2, 2);
plot(t1, phi_dot1_deg, 'Color', c_blue, 'LineWidth', 2.5);
xlabel('Time  [s]',           'FontSize', 11);
ylabel('\phi_{dot}  [deg/s]', 'FontSize', 11);
title('Roll rate  \phi_{dot} — grows linearly (constant \alpha)', 'FontSize', 12);
grid on; box off;
% Overlay theoretical rate: phi_dot = alpha * t
hold on;
plot(t1, rad2deg(alpha_expected * t1), '--', 'Color', c_black, 'LineWidth', 1.8);
hold off;
legend('Simulated', '\alpha \cdot t  (expected)', ...
       'Location', 'northwest', 'FontSize', 9);

% ── Panel 3: Pitch angle (should be zero for symmetric drone) ────────────
subplot(3, 2, 3);
plot(t1, theta1_deg, 'Color', c_red, 'LineWidth', 2.5);
xlabel('Time  [s]',     'FontSize', 11);
ylabel('\theta  [deg]', 'FontSize', 11);
title({'\theta — Pitch angle  (\tau_\theta = 0)', ...
       'Zero for symmetric drone with pure roll. This is CORRECT.'}, ...
      'FontSize', 11);
grid on; box off;
ylim([-0.001, 0.001]);   % zoom in to show it is truly zero

% ── Panel 4: Yaw angle (should be zero for symmetric drone) ──────────────
subplot(3, 2, 4);
plot(t1, psi1_deg, 'Color', c_green, 'LineWidth', 2.5);
xlabel('Time  [s]',   'FontSize', 11);
ylabel('\psi  [deg]', 'FontSize', 11);
title({'\psi — Yaw angle  (\tau_\psi = 0)', ...
       'Zero because I_{xx} = I_{yy} (symmetric frame). This is CORRECT.'}, ...
      'FontSize', 11);
grid on; box off;
ylim([-0.001, 0.001]);   % zoom in to show it is truly zero

% ── Panel 5: Error between simulated and analytical roll angle ────────────
subplot(3, 2, 5);
error_rad = abs(phi1 - phi_analytical);
error_deg = rad2deg(error_rad);
plot(t1, error_deg, 'Color', c_gray, 'LineWidth', 2);
xlabel('Time  [s]',                          'FontSize', 11);
ylabel('|\phi_{sim} - \phi_{analytical}|  [deg]', 'FontSize', 11);
title('Simulation error vs analytical prediction', 'FontSize', 12);
grid on; box off;

% ── Panel 6: Phase portrait — roll angle vs roll rate ────────────────────
subplot(3, 2, 6);
plot(phi1_deg, phi_dot1_deg, 'Color', c_blue, 'LineWidth', 2.5);
hold on;
% Mark start and end
plot(phi1_deg(1),   phi_dot1_deg(1),   'o', 'Color', c_green, ...
     'MarkerSize', 10, 'MarkerFaceColor', c_green);
plot(phi1_deg(end), phi_dot1_deg(end), 's', 'Color', c_red, ...
     'MarkerSize', 10, 'MarkerFaceColor', c_red);
hold off;
xlabel('\phi  [deg]',         'FontSize', 11);
ylabel('\phi_{dot}  [deg/s]', 'FontSize', 11);
title('Phase portrait: \phi vs \phi_{dot}', 'FontSize', 12);
legend('Trajectory', 'Start (t=0)', 'End (t=4s)', ...
       'Location', 'northwest', 'FontSize', 9);
grid on; box off;

sgtitle('Phase 1 — Test 1: Pure Roll Torque from Rest (Open-Loop, No Controller)', ...
        'FontSize', 13, 'FontWeight', 'bold');

% =========================================================
%  FIGURE 2: Test Case 2 Results — Coupling Demonstration (4 panels)
% =========================================================
fig2 = figure('Name', 'Phase 1 — Test 2: Coupling Demonstration', ...
              'NumberTitle', 'off', ...
              'Position', [80 80 1100 620]);

% ── Panel 1: Roll angle (Test 2) ─────────────────────────────────────────
subplot(2, 2, 1);
plot(t2, phi2_deg, 'Color', c_blue, 'LineWidth', 2.5);
hold on;
plot(t1, phi1_deg, '--', 'Color', c_gray, 'LineWidth', 1.5);
hold off;
xlabel('Time  [s]',      'FontSize', 11);
ylabel('\phi  [deg]',    'FontSize', 11);
title('Roll angle \phi', 'FontSize', 12);
legend('Test 2 (with initial \psi_{dot})', ...
       'Test 1 (pure roll, for reference)', ...
       'Location', 'northwest', 'FontSize', 9);
grid on; box off;

% ── Panel 2: Roll rate (Test 2) ──────────────────────────────────────────
subplot(2, 2, 2);
plot(t2, phi_dot2_deg, 'Color', c_blue, 'LineWidth', 2.5);
hold on;
plot(t1, phi_dot1_deg, '--', 'Color', c_gray, 'LineWidth', 1.5);
hold off;
xlabel('Time  [s]',           'FontSize', 11);
ylabel('\phi_{dot}  [deg/s]', 'FontSize', 11);
title('Roll rate \phi_{dot}', 'FontSize', 12);
legend('Test 2', 'Test 1 (reference)', 'Location', 'northwest', 'FontSize', 9);
grid on; box off;

% ── Panel 3: Pitch angle (Test 2) — THIS IS THE COUPLING ─────────────────
subplot(2, 2, 3);
plot(t2, theta2_deg, 'Color', c_red, 'LineWidth', 2.5);
xlabel('Time  [s]',     'FontSize', 11);
ylabel('\theta  [deg]', 'FontSize', 11);
title({'\theta — Pitch coupling  (I_{zz}-I_{xx}) \cdot \phi_{dot} \cdot \psi_{dot} / I_{yy}', ...
       'Pitch responds even though \tau_\theta = 0 — this is the NONLINEARITY'}, ...
      'FontSize', 10);
grid on; box off;
% Annotate max pitch value
[max_theta, idx_max] = max(abs(theta2_deg));
text(t2(idx_max), theta2_deg(idx_max), ...
     sprintf('  %.2f deg', theta2_deg(idx_max)), ...
     'Color', c_red, 'FontSize', 10);

% ── Panel 4: Pitch rate (Test 2) ─────────────────────────────────────────
subplot(2, 2, 4);
plot(t2, theta_dot2_deg, 'Color', c_red, 'LineWidth', 2.5);
xlabel('Time  [s]',               'FontSize', 11);
ylabel('\theta_{dot}  [deg/s]',   'FontSize', 11);
title('Pitch rate \theta_{dot}  — driven entirely by coupling', 'FontSize', 12);
grid on; box off;

sgtitle({'Phase 1 — Test 2: Gyroscopic Coupling Demonstration', ...
         'Initial yaw rate = 0.5 rad/s  |  Roll torque = 0.005 N*m  |  \tau_\theta = 0'}, ...
        'FontSize', 12, 'FontWeight', 'bold');

% =========================================================
%  VALIDATION CHECKS — Three checks, all must pass.
% =========================================================
fprintf('============================================================\n');
fprintf('  PHASE 1 — MODEL VALIDATION RESULTS\n');
fprintf('============================================================\n\n');

% ─────────────────────────────────────────────────────────
%  CHECK 1: Angular acceleration accuracy
%  Method: The average angular acceleration is (final rate - initial rate)
%  divided by total time. For constant applied torque with no coupling
%  (Test 1), this must match tau_phi / Ixx to within 1%.
% ─────────────────────────────────────────────────────────
alpha_simulated = (x1(end, 2) - x1(1, 2)) / (t1(end) - t1(1));
error_pct       = 100 * abs(alpha_simulated - alpha_expected) / alpha_expected;
check1          = error_pct < 1.0;

fprintf('  CHECK 1 — Angular acceleration accuracy (Test 1)\n');
fprintf('  -------------------------------------------------\n');
fprintf('    Applied roll torque    : %.4f N*m\n', u_test(1));
fprintf('    Expected alpha (tau/I) : %.4f rad/s^2\n', alpha_expected);
fprintf('    Simulated alpha (avg)  : %.4f rad/s^2\n', alpha_simulated);
fprintf('    Percentage error       : %.4f%%\n', error_pct);
if check1
    fprintf('    Result                 : PASS  (error < 1%%)\n\n');
else
    fprintf('    Result                 : FAIL  (error >= 1%%)\n');
    fprintf('    Fix: Check that dynamics.m divides by Ixx correctly.\n');
    fprintf('         Also verify ODE tolerances are set to RelTol=1e-8.\n\n');
end

% ─────────────────────────────────────────────────────────
%  CHECK 2: Gyroscopic coupling is active
%  Method: In Test 2, pitch (theta) must respond even though
%  tau_theta = 0, because of the coupling term (Izz-Ixx)*phi_dot*psi_dot.
%  We expect a significant pitch response (> 1 degree after 4 seconds).
% ─────────────────────────────────────────────────────────
max_pitch_coupling_deg = max(abs(theta2_deg));
check2 = max_pitch_coupling_deg > 1.0;  % expect > 1 degree of pitch

fprintf('  CHECK 2 — Gyroscopic coupling active (Test 2)\n');
fprintf('  -----------------------------------------------\n');
fprintf('    Initial yaw rate       : %.2f rad/s  (%.1f deg/s)\n', ...
        x0_test2(6), rad2deg(x0_test2(6)));
fprintf('    Applied roll torque    : %.4f N*m\n', u_test(1));
fprintf('    tau_theta commanded    : 0 N*m  (no pitch torque)\n');
fprintf('    Max pitch response     : %.4f deg  (from coupling only)\n', ...
        max_pitch_coupling_deg);
if check2
    fprintf('    Result                 : PASS  (coupling term is active)\n\n');
else
    fprintf('    Result                 : FAIL  (coupling appears to be zero)\n');
    fprintf('    Fix: Verify that the pitch equation in dynamics.m includes\n');
    fprintf('         the term (Izz - Ixx) * phi_dot * psi_dot.\n');
    fprintf('         Also verify that psi_dot (x(6)) is correctly unpacked.\n\n');
end

% ─────────────────────────────────────────────────────────
%  CHECK 3: Kinematic prediction match (first 1 second)
%  Method: In the first second, the coupling is very small because
%  phi_dot is still small. Roll angle must match 0.5*(tau/I)*t^2
%  to within 1e-5 radians (about 0.0006 degrees) in this window.
% ─────────────────────────────────────────────────────────
early_mask  = t1 <= 1.0;
error_early = max(abs(phi1(early_mask) - phi_analytical(early_mask)));
check3      = error_early < 1e-5;

fprintf('  CHECK 3 — Kinematic prediction match (first 1 second)\n');
fprintf('  -------------------------------------------------------\n');
fprintf('    Comparing simulation vs  0.5*(tau/Ixx)*t^2\n');
fprintf('    Max error (t = 0 to 1s) : %.2e rad  (%.2e deg)\n', ...
        error_early, rad2deg(error_early));
if check3
    fprintf('    Result                  : PASS  (error < 1e-5 rad)\n\n');
else
    fprintf('    Result                  : FAIL  (error too large)\n');
    fprintf('    Fix: Increase ODE solver accuracy.\n');
    fprintf('         Make sure opts = odeset(RelTol,1e-8,AbsTol,1e-10)\n');
    fprintf('         is passed to ode45. Default tolerances are too loose.\n\n');
end

% ─────────────────────────────────────────────────────────
%  OVERALL RESULT
% ─────────────────────────────────────────────────────────
fprintf('============================================================\n');
if check1 && check2 && check3
    fprintf('  OVERALL: ALL THREE CHECKS PASSED\n');
    fprintf('\n');
    fprintf('  Your dynamics.m model is physically correct.\n');
    fprintf('  Ready to proceed to Phase 2 (PID controller design).\n');
else
    fprintf('  OVERALL: ONE OR MORE CHECKS FAILED\n');
    fprintf('\n');
    fprintf('  Do NOT proceed to Phase 2 until all checks pass.\n');
    fprintf('  Common mistakes:\n');
    fprintf('    - Wrong sign on a coupling term in dynamics.m\n');
    fprintf('      (check the cyclic pattern in the comments)\n');
    fprintf('    - Coupling term missing or multiplied by wrong rates\n');
    fprintf('    - Division by wrong inertia value\n');
    fprintf('    - Mixing degrees and radians inside dynamics.m\n');
    fprintf('    - Loose ODE tolerances (RelTol must be 1e-8 or tighter)\n');
    fprintf('    - dxdt returned as row vector instead of column vector\n');
end
fprintf('============================================================\n\n');

% ─────────────────────────────────────────────────────────
%  NUMERICAL SUMMARY TABLE
% ─────────────────────────────────────────────────────────
fprintf('  QUICK REFERENCE — Key values from simulation:\n');
fprintf('  ----------------------------------------------\n');
fprintf('    Hover motor speed        : %.0f RPM  (%.1f rad/s)\n', ...
        p.rpm_hover, p.w_hover);
fprintf('    Roll alpha (expected)    : %.4f rad/s^2\n', alpha_expected);
fprintf('    Roll alpha (simulated)   : %.4f rad/s^2\n', alpha_simulated);
fprintf('    Roll angle at t=4s       : %.2f deg\n',     phi1_deg(end));
fprintf('    Roll rate  at t=4s       : %.2f deg/s\n',   phi_dot1_deg(end));
fprintf('    Pitch at t=4s (Test 1)   : %.6f deg  (zero, symmetric)\n', ...
        theta1_deg(end));
fprintf('    Pitch at t=4s (Test 2)   : %.2f deg  (nonzero, coupling)\n', ...
        theta2_deg(end));
fprintf('  ----------------------------------------------\n\n');

% ─────────────────────────────────────────────────────────
%  OPTIONAL: Save figures as PDF for report
%  Uncomment these lines when you are ready to save.
% ─────────────────────────────────────────────────────────
% exportgraphics(fig1, 'phase1_test1_pure_roll.pdf',    'Resolution', 600);
% exportgraphics(fig2, 'phase1_test2_coupling.pdf',     'Resolution', 600);
% fprintf('  Figures saved as PDF files.\n\n');
