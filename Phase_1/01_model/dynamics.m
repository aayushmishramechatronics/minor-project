function dxdt = dynamics(t, x, u, p)
%   DYNAMICS  Nonlinear rotational dynamics of a quadcopter (body frame).
%   what this function does:
%   implements the Newton-Euler rotational equations for a
%   rigid-body quadcopter and takes the current state and control
%   torques, returns the time derivative of the state.
%
%   ode45 calls this function hundreds of times per simulation.
%   it must be fast, have no side effects, and never print to
%   the console.
%   the governing equations (derived from I*w_dot = tau - w x (I*w)):
%
%   Roll:   Ixx * phi_ddot   = tau_phi   + (Iyy - Izz) * theta_dot * psi_dot
%   Pitch:  Iyy * theta_ddot = tau_theta + (Izz - Ixx) * phi_dot   * psi_dot
%   Yaw:    Izz * psi_ddot   = tau_psi   + (Ixx - Iyy) * phi_dot   * theta_dot
%
%   the right-hand coupling terms — e.g. (Iyy-Izz)*theta_dot*psi_dot —
%   come from the cross product w x (I*w). They represent gyroscopic
%   effects: rotation on one axis influences the others. These terms
%   are what make the system NONLINEAR and why SMC is needed.
%
%   cyclic pattern (self-check for correct signs):
%     roll  equation: coefficient (Iyy-Izz), rates are theta_dot * psi_dot
%     pitch equation: coefficient (Izz-Ixx), rates are phi_dot   * psi_dot
%     yaw   equation: coefficient (Ixx-Iyy), rates are phi_dot   * theta_dot
%   each equation uses the OTHER two inertias and OTHER two rates.
%   if your signs don't follow this pattern, there is a mistake.

% Inputs:
%   t   — current simulation time [s]
%          ode45 always passes time as the first argument.
%          The drone equations are time-invariant (they don't
%          explicitly depend on t), but the argument must be here.
%
%   x   — state vector [6x1], all in radians / rad/s
%            x(1) = phi       roll  angle   [rad]
%            x(2) = phi_dot   roll  rate    [rad/s]
%            x(3) = theta     pitch angle   [rad]
%            x(4) = theta_dot pitch rate    [rad/s]
%            x(5) = psi       yaw   angle   [rad]
%            x(6) = psi_dot   yaw   rate    [rad/s]
%
%   u   — control input [3x1], in Newton-metres
%            u(1) = tau_phi    commanded roll  torque [N*m]
%            u(2) = tau_theta  commanded pitch torque [N*m]
%            u(3) = tau_psi    commanded yaw   torque [N*m]
%
%   p   — parameter struct from parameters.m
%          Must contain fields: p.Ixx, p.Iyy, p.Izz
%
% Output:
%   dxdt — time derivative of state [6x1]
%            dxdt(1) = phi_dot      [rad/s]
%            dxdt(2) = phi_ddot     [rad/s^2]
%            dxdt(3) = theta_dot    [rad/s]
%            dxdt(4) = theta_ddot   [rad/s^2]
%            dxdt(5) = psi_dot      [rad/s]
%            dxdt(6) = psi_ddot     [rad/s^2]
%
% Important Rules:
%   1. everything inside this function must be in radians.
%      never convert to degrees here. conversion happens
%      only in plotting scripts.
%   2. dxdt must be a column vector [6x1]. If you return a
%      row vector [1x6], ode45 will crash with a size error.
%   3. do not drop the coupling terms. They are the reason
%      this project uses SMC instead of PID.
% =========================================================

% STEP 1: unpack state vector into named variables
% only the rates are needed for the equations below.
% the absolute angles (phi, theta, psi) do not appear in the rotational
% dynamics — they only matter for the translational dynamics (Phase 5).
phi_dot   = x(2);          % roll  rate  [rad/s]
theta_dot = x(4);          % pitch rate  [rad/s]
psi_dot   = x(6);          % yaw   rate  [rad/s]

% STEP 2: unpack control inputs
tau_phi   = u(1);          % commanded roll  torque [N*m]
tau_theta = u(2);          % commanded pitch torque [N*m]
tau_psi   = u(3);          % commanded yaw   torque [N*m]

% STEP 3: Unpack inertia parameters
Ixx = p.Ixx;
Iyy = p.Iyy;
Izz = p.Izz;

% STEP 4: Newton-Euler equations
% roll equation:
%   applied roll torque + gyroscopic coupling from pitch*yaw rates
phi_ddot   = ( tau_phi   + (Iyy - Izz) * theta_dot * psi_dot ) / Ixx;

% pitch equation:
%   applied pitch torque + gyroscopic coupling from roll*yaw rates
theta_ddot = ( tau_theta + (Izz - Ixx) * phi_dot   * psi_dot ) / Iyy;

% yaw equation:
%   applied yaw torque + gyroscopic coupling from roll*pitch rates
%   NOTE: If Ixx == Iyy (symmetric frame), (Ixx-Iyy) = 0, so the
%         yaw coupling term vanishes. This is physically correct.
psi_ddot   = ( tau_psi   + (Ixx - Iyy) * phi_dot   * theta_dot ) / Izz;

% STEP 5: Assemble state derivative vector [6x1]
% the first equation of each pair is trivial (rate = d/dt of angle).
% the second equation is the angular acceleration from Newton-Euler.
dxdt    = zeros(6, 1);     % pre-allocate as column vector — do not skip this
dxdt(1) = phi_dot;         % d(phi)/dt
dxdt(2) = phi_ddot;        % d(phi_dot)/dt
dxdt(3) = theta_dot;       % d(theta)/dt
dxdt(4) = theta_ddot;      % d(theta_dot)/dt
dxdt(5) = psi_dot;         % d(psi)/dt
dxdt(6) = psi_ddot;        % d(psi_dot)/dt

end