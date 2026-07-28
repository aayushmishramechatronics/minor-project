function dxdt = pid_closed_loop(t, x, ref, disturbance, p, g)
%PID_CLOSED_LOOP  Closed-loop quadcopter dynamics with PID controller.
%
% =========================================================
% WHAT THIS FUNCTION DOES:
%   Combines the drone dynamics (from Phase 1's dynamics.m) with the
%   PID controller into a single ODE function that ode45 can solve.
%
%   Uses an AUGMENTED STATE VECTOR of 9 elements:
%     x(1:6) = the 6 attitude states from Phase 1
%     x(7)   = integral of roll  error (accumulated over time)
%     x(8)   = integral of pitch error
%     x(9)   = integral of yaw   error
%
%   The integral is a genuine ODE state, not a manually-stepped
%   variable. This lets ode45 integrate it with the same numerical
%   accuracy as the angle and rate states — far better than doing
%   Euler integration (integral = integral + error*dt) by hand.
%
% =========================================================
% ANTI-WINDUP  (IMPORTANT — READ THIS)
%   A real motor cannot produce unlimited torque, so the commanded
%   torque is saturated (clamped) to tau_max. If the integral term
%   keeps growing while the output is already saturated, it builds
%   up a large "hidden" value that later causes a big overshoot once
%   the actuator comes out of saturation. This is called "windup".
%
%   The fix used here is CONDITIONAL INTEGRATION: if the raw
%   (pre-saturation) torque exceeds the limit AND the current error
%   would push it further into saturation, the integral's derivative
%   is frozen at zero for that instant. Otherwise it integrates
%   normally. This is a standard, simple, and effective anti-windup
%   method for continuous-time (ODE-based) PID implementations.
%
% =========================================================
% AUGMENTED STATE  x [9x1]:
%   x(1) = phi         roll  angle   [rad]
%   x(2) = phi_dot     roll  rate    [rad/s]
%   x(3) = theta       pitch angle   [rad]
%   x(4) = theta_dot   pitch rate    [rad/s]
%   x(5) = psi         yaw   angle   [rad]
%   x(6) = psi_dot     yaw   rate    [rad/s]
%   x(7) = I_phi       integral of roll  error  [rad*s]
%   x(8) = I_theta     integral of pitch error  [rad*s]
%   x(9) = I_psi       integral of yaw   error  [rad*s]
%
% INPUTS:
%   t            — current time [s]
%   x            — augmented state vector [9x1]
%   ref          — reference commands [3x1]: [phi_des; theta_des; psi_des] [rad]
%   disturbance  — struct for external torque injection. Must always
%                  have all three fields defined, even if inactive:
%                    disturbance.active  true or false
%                    disturbance.t_start time when disturbance begins [s]
%                    disturbance.torque  [3x1] disturbance torques [N*m]
%   p            — parameter struct from parameters.m
%   g            — gains struct from pid_gains.m
%
% OUTPUT:
%   dxdt         — time derivative of augmented state [9x1]
% =========================================================

% ── STEP 1: Unpack state ──────────────────────────────────────────────────
phi       = x(1);
phi_dot   = x(2);
theta     = x(3);
theta_dot = x(4);
psi       = x(5);
psi_dot   = x(6);
I_phi     = x(7);
I_theta   = x(8);
I_psi     = x(9);

% ── STEP 2: Unpack reference commands ────────────────────────────────────
phi_des   = ref(1);
theta_des = ref(2);
psi_des   = ref(3);

% ── STEP 3: Compute tracking errors ──────────────────────────────────────
e_phi   = phi_des   - phi;
e_theta = theta_des - theta;
e_psi   = psi_des   - psi;

% ── STEP 4: Compute RAW (pre-saturation) PID torques ──────────────────────
%
%   tau = Kp * error                       (proportional)
%       + Ki * integral_of_error            (integral)
%       - Kd * measured_rate                (derivative on measurement)
%
%   Derivative is taken from the measured rate rather than from
%   d(error)/dt. When the reference is constant these are equivalent
%   (d(error)/dt = -d(actual)/dt = -rate), but using the measured rate
%   directly avoids a large torque spike the instant the reference steps.
%
tau_phi_raw   = g.roll.Kp  * e_phi   + g.roll.Ki  * I_phi   - g.roll.Kd  * phi_dot;
tau_theta_raw = g.pitch.Kp * e_theta + g.pitch.Ki * I_theta - g.pitch.Kd * theta_dot;
tau_psi_raw   = g.yaw.Kp   * e_psi   + g.yaw.Ki   * I_psi   - g.yaw.Kd   * psi_dot;

% ── STEP 5: Saturate torques to actuator limits ───────────────────────────
%   This is the torque actually sent to the plant.
tau_phi   = max(-g.roll.tau_max,  min(g.roll.tau_max,  tau_phi_raw));
tau_theta = max(-g.pitch.tau_max, min(g.pitch.tau_max, tau_theta_raw));
tau_psi   = max(-g.yaw.tau_max,   min(g.yaw.tau_max,   tau_psi_raw));

% ── STEP 6: Add external disturbance torque ────────────────────────────────
%   During disturbance tests, an additional torque is injected directly
%   into the plant input (simulating wind or a payload shift).
%   This is added AFTER saturation of the PID output — the disturbance
%   is a physical force acting on the drone, not part of the controller.
if disturbance.active && t >= disturbance.t_start
    tau_phi   = tau_phi   + disturbance.torque(1);
    tau_theta = tau_theta + disturbance.torque(2);
    tau_psi   = tau_psi   + disturbance.torque(3);
end

% ── STEP 7: Pack torques into control input vector ────────────────────────
u = [tau_phi; tau_theta; tau_psi];

% ── STEP 8: Compute attitude derivatives using Phase 1 dynamics ───────────
%   Calls dynamics.m exactly as in Phase 1. The nonlinear equations
%   are completely unchanged — the PID controller only determines
%   what value of u gets passed in.
dxdt_attitude = dynamics(t, x(1:6), u, p);

% ── STEP 9: Integral state derivatives WITH ANTI-WINDUP ────────────────────
%
%   Normal case: d(I)/dt = error  (integral accumulates the error, and
%   ode45 integrates this state just like it integrates phi or theta).
%
%   Anti-windup case: if the RAW torque exceeds tau_max in the SAME
%   direction as the current error (meaning integrating further would
%   only push the output deeper into saturation with no real effect),
%   freeze the integral by setting its derivative to zero.
%
%   ROLL axis:
if (tau_phi_raw > g.roll.tau_max && e_phi > 0) || ...
   (tau_phi_raw < -g.roll.tau_max && e_phi < 0)
    dI_phi = 0;          % frozen — saturated, error would push deeper in
else
    dI_phi = e_phi;       % normal integration
end

%   PITCH axis:
if (tau_theta_raw > g.pitch.tau_max && e_theta > 0) || ...
   (tau_theta_raw < -g.pitch.tau_max && e_theta < 0)
    dI_theta = 0;
else
    dI_theta = e_theta;
end

%   YAW axis:
if (tau_psi_raw > g.yaw.tau_max && e_psi > 0) || ...
   (tau_psi_raw < -g.yaw.tau_max && e_psi < 0)
    dI_psi = 0;
else
    dI_psi = e_psi;
end

% ── STEP 10: Assemble full 9-element derivative vector ─────────────────────
dxdt      = zeros(9, 1);
dxdt(1:6) = dxdt_attitude;   % attitude derivatives from dynamics.m
dxdt(7)   = dI_phi;          % roll  integral derivative (with anti-windup)
dxdt(8)   = dI_theta;        % pitch integral derivative (with anti-windup)
dxdt(9)   = dI_psi;          % yaw   integral derivative (with anti-windup)

end
