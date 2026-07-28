%% pid_gains.m
%
% Phase 2 — PID Baseline Controller — Gains
% =========================================================
% PURPOSE:
%   Define PID gains for all three attitude axes.
%   Run this AFTER parameters.m so the parameter struct p
%   is already in the workspace (gains are derived from p).
%
% HOW TO RUN:
%   >> parameters      (loads struct p)
%   >> pid_gains        (loads struct g into workspace)
%
% HOW THE GAINS WERE CHOSEN:
%   At hover, all angles and rates are zero. The coupling terms
%   in the Newton-Euler equations vanish because they are products
%   of angular rates. Each axis decouples into a simple double
%   integrator:
%
%       phi_ddot   = tau_phi   / Ixx
%       theta_ddot = tau_theta / Iyy
%       psi_ddot   = tau_psi   / Izz
%
%   Transfer function for each axis: G(s) = 1 / (I * s^2)
%
%   A PD controller gives the closed-loop characteristic polynomial:
%       I*s^2 + Kd*s + Kp = 0
%
%   Matching to the standard second-order form s^2 + 2*zeta*wn*s + wn^2:
%       Kp = I * wn^2
%       Kd = I * 2 * zeta * wn
%
%   Design targets:
%     Roll / Pitch : wn = 5 rad/s, zeta = 0.8
%     Yaw          : wn = 3 rad/s, zeta = 0.8   (Izz larger -> naturally slower)
%
%   WHY Ki EXISTS HERE:
%   The plant I*s^2 already has two free integrators (it is "Type 2"),
%   which means pure PD control already gives ZERO steady-state error
%   for a step reference command. Ki is NOT needed for that.
%   Ki's real job is different: it removes steady-state error caused
%   by a CONSTANT EXTERNAL DISTURBANCE torque (wind, payload shift).
%   Without Ki, a constant disturbance torque would leave the drone
%   permanently tilted by a small constant angle. With Ki, that
%   residual angle is driven to exactly zero over time.
% =========================================================

if ~exist('p', 'var')
    error(['pid_gains requires the parameter struct p to be in the workspace. ', ...
           'Run parameters.m first.']);
end

% ─────────────────────────────────────────────────────────────────────────
%  ROLL AXIS  (uses Ixx)
% ─────────────────────────────────────────────────────────────────────────
wn_roll   = 5.0;    % desired closed-loop natural frequency [rad/s]
zeta_roll = 0.8;    % desired damping ratio

g.roll.Kp      = p.Ixx * wn_roll^2;                 % [N*m/rad]
g.roll.Kd      = p.Ixx * 2 * zeta_roll * wn_roll;   % [N*m*s/rad]
g.roll.Ki      = 0.02;                               % [N*m/(rad*s)]
g.roll.tau_max = 0.5;                                % [N*m] actuator saturation limit

% ─────────────────────────────────────────────────────────────────────────
%  PITCH AXIS  (uses Iyy — equal to Ixx for a symmetric frame)
% ─────────────────────────────────────────────────────────────────────────
wn_pitch   = 5.0;
zeta_pitch = 0.8;

g.pitch.Kp      = p.Iyy * wn_pitch^2;
g.pitch.Kd      = p.Iyy * 2 * zeta_pitch * wn_pitch;
g.pitch.Ki      = 0.02;
g.pitch.tau_max = 0.5;

% ─────────────────────────────────────────────────────────────────────────
%  YAW AXIS  (uses Izz — always larger, so yaw is naturally slower)
% ─────────────────────────────────────────────────────────────────────────
wn_yaw   = 3.0;
zeta_yaw = 0.8;

g.yaw.Kp      = p.Izz * wn_yaw^2;
g.yaw.Kd      = p.Izz * 2 * zeta_yaw * wn_yaw;
g.yaw.Ki      = 0.01;
g.yaw.tau_max = 0.3;

% ─────────────────────────────────────────────────────────────────────────
%  PRINT SUMMARY
% ─────────────────────────────────────────────────────────────────────────
fprintf('\n');
fprintf('=================================================================\n');
fprintf('   PID GAINS  —  LOADED\n');
fprintf('=================================================================\n');
fprintf('\n');
fprintf('  Axis       Kp         Ki         Kd         tau_max\n');
fprintf('  -------  ---------  ---------  ---------  ---------\n');
fprintf('  Roll     %8.4f   %8.4f   %8.4f   %8.4f  [N*m]\n', ...
        g.roll.Kp,  g.roll.Ki,  g.roll.Kd,  g.roll.tau_max);
fprintf('  Pitch    %8.4f   %8.4f   %8.4f   %8.4f  [N*m]\n', ...
        g.pitch.Kp, g.pitch.Ki, g.pitch.Kd, g.pitch.tau_max);
fprintf('  Yaw      %8.4f   %8.4f   %8.4f   %8.4f  [N*m]\n', ...
        g.yaw.Kp,   g.yaw.Ki,   g.yaw.Kd,   g.yaw.tau_max);
fprintf('\n');
fprintf('  Design targets:\n');
fprintf('    Roll / Pitch : wn = %.1f rad/s, zeta = %.1f\n', wn_roll, zeta_roll);
fprintf('    Yaw          : wn = %.1f rad/s, zeta = %.1f\n', wn_yaw,  zeta_yaw);
fprintf('\n');
fprintf('  Struct ''g'' is in your workspace.\n');
fprintf('  Next step: run  simulate_pid\n');
fprintf('=================================================================\n\n');
