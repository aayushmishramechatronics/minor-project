% PURPOSE:
%   defines every physical constant for the quadcopter and
%   packs them into a struct 'p'. Run this file first at the
%   start of every MATLAB session before any other Phase 1 script.
%   UNITS: all values are in SI units throughout.
%         Length: metres [m]
%         Mass:   kilograms [kg]
%         Time:   seconds [s]
%         Angle:  radians [rad]   (convert to degrees only when plotting)
%         Force:  Newtons [N]
%         Torque: Newton-metres [N*m]

clear; clc;

%  SECTION 1: PHYSICAL CONSTANTS
g = 9.81;   % gravitational acceleration [m/s^2]

%  SECTION 2: YOUR DRONE MEASUREMENTS
%  Change these two values to match your specific drone.

m = 0.5;                % total mass, fully assembled with battery [kg]
                        % HOW TO MEASURE: place the assembled drone on a
                        % kitchen scale. Include battery, propellers, everything.

L = 0.175;              % arm length [m]
                        % HOW TO MEASURE: distance from the centre of the
                        % frame to the centre of the motor shaft.
                        % For a 450mm frame this is approximately 0.175 m.

%  SECTION 3: MOMENTS OF INERTIA [kg*m^2]
%
%  what these mean:
%    Ixx = how hard it is to angularly accelerate the drone about the roll axis
%    Iyy = how hard it is to angularly accelerate about the pitch axis
%    Izz = how hard it is to angularly accelerate about the yaw axis
%
%  for a symmetric X-frame: Ixx = Iyy (roll and pitch behave identically)
%  Izz is always LARGER than Ixx because the motor masses are spread far
%  from the vertical z-axis when viewed from the top.
%
%  how to estimate:
%    Ixx ≈ 2 * motor_mass_kg * L^2
%    for 80g motors at L=0.175m: Ixx ≈ 2 * 0.08 * 0.175^2 = 4.9e-3 kg*m^2
%
%  how to measure accurately:
%    use the bifilar pendulum method (two strings, measure oscillation period).
%    replace these estimates with measured values before hardware testing.

Ixx = 4.856e-3;         % roll  axis moment of inertia [kg*m^2]
Iyy = 4.856e-3;         % pitch axis moment of inertia [kg*m^2]
                        % note: Iyy = Ixx for a symmetric X-frame.
                        % if your frame is not symmetric, measure both separately.
Izz = 8.801e-3;         % yaw axis moment of inertia [kg*m^2]

%  SECTION 4: AERODYNAMIC COEFFICIENTS
%  kF (thrust coefficient):
%    each motor spinning at angular velocity omega [rad/s] produces
%    an upward thrust force: F = kF * omega^2  [N]
%    this coefficient depends on the propeller size, pitch, and air density.
%    MUST be measured on a thrust stand for your specific motor + propeller.
%    typical range for 10-inch propellers: 2.5e-6 to 3.5e-6
%
%  kM (drag torque coefficient):
%    each motor at speed omega creates a reactive torque on the frame:
%    Q = kM * omega^2  [N*m]
%    the air resists the propeller's spin (drag), and by Newton's 3rd law
%    this drag torque acts on the drone frame in the opposite direction.
%    this is what allows yaw control.
%    typical ratio: kM ≈ 0.038 * kF  for most propellers.

kF = 2.980e-6;          % thrust coefficient      [N*s^2/rad^2]
kM = 1.140e-7;          % drag torque coefficient [N*m*s^2/rad^2]

%  SECTION 5: PACK INTO STRUCT
%  why use a struct?
%    if we passed each constant as a separate argument, every function
%    call would look like: dynamics(t, x, u, g, m, L, Ixx, Iyy, Izz, kF, kM)
%    that is messy and error-prone. with a struct we just pass 'p'.
%    also, changing a parameter in ONE place (here) updates everything.

p.g   = g;
p.m   = m;
p.L   = L;
p.Ixx = Ixx;
p.Iyy = Iyy;
p.Izz = Izz;
p.kF  = kF;
p.kM  = kM;

%  SECTION 6: DERIVED QUANTITIES
%  computed from the parameters above. Used for validation.

p.W         = m * g;                        % total drone weight        [N]
p.F_hover   = m * g / 4;                    % required thrust per motor [N]
p.w_hover   = sqrt(p.F_hover / kF);         % hover motor speed         [rad/s]
p.rpm_hover = p.w_hover * 60 / (2 * pi);    % hover motor speed         [RPM]


%  SECTION 7: PRINT SUMMARY TO CONSOLE

fprintf('\n');
fprintf('  QUADCOPTER PARAMETERS —\n');
fprintf('  physical properties:\n');
fprintf('    Mass (m)            : %.3f kg\n',      m);
fprintf('    Arm length (L)      : %.4f m\n',       L);
fprintf('    Weight (m*g)        : %.3f N\n',       p.W);
fprintf('\n');
fprintf('  moments of inertia:\n');
fprintf('    Ixx (roll axis)     : %.4e kg*m^2\n',  Ixx);
fprintf('    Iyy (pitch axis)    : %.4e kg*m^2\n',  Iyy);
fprintf('    Izz (yaw axis)      : %.4e kg*m^2\n',  Izz);
fprintf('    Ixx == Iyy          : %s\n', ...
        mat2str(abs(Ixx - Iyy) < 1e-10));
fprintf('\n');
fprintf('  aerodynamic coefficients:\n');
fprintf('    kF (thrust)         : %.4e N*s^2/rad^2\n',   kF);
fprintf('    kM (drag torque)    : %.4e N*m*s^2/rad^2\n', kM);
fprintf('    kM/kF ratio         : %.4f\n', kM / kF);
fprintf('\n');
fprintf('  hover calculations:\n');
fprintf('    Thrust per motor    : %.4f N\n',  p.F_hover);
fprintf('    Motor speed (hover) : %.1f rad/s\n', p.w_hover);
fprintf('    Motor speed (hover) : %.0f RPM\n',   p.rpm_hover);

%  SECTION 8: SANITY CHECKS
passed = true;

% Check 1: hover RPM should be physically realistic
if p.rpm_hover < 3000 || p.rpm_hover > 10000
    fprintf('\n  [WARNING] Hover RPM = %.0f is outside the expected range\n', p.rpm_hover);
    fprintf('           of 3000 to 10000 RPM for a 450mm drone.\n');
    fprintf('           double-check your kF value: currently %.4e\n', kF);
    passed = false;
end

% Check 2: Izz must be larger than Ixx and Iyy
if Izz <= Ixx || Izz <= Iyy
    fprintf('\n  [WARNING] Izz (%.4e) should be larger than Ixx (%.4e)\n', Izz, Ixx);
    fprintf('           and Iyy (%.4e) for a typical quadcopter.\n', Iyy);
    passed = false;
end

% Check 3: Ixx should equal Iyy for a symmetric X-frame
if abs(Ixx - Iyy) > 1e-6
    fprintf('\n  [NOTE] Ixx != Iyy. This means your frame is not perfectly symmetric.\n');
    fprintf('         gyroscopic coupling in yaw will be nonzero.\n');
end

if passed
    fprintf('\n  all sanity checks passed.\n');
    fprintf('  ready to run dynamics.m and simulate_openloop.m\n');
end
