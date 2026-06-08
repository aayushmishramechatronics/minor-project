QUADCOPTER PARAMETERS —
  physical properties:
    Mass (m)            : 0.500 kg
    Arm length (L)      : 0.1750 m
    Weight (m*g)        : 4.905 N
  moments of inertia:
    Ixx (roll axis)     : 4.8560e-03 kg*m^2
    Iyy (pitch axis)    : 4.8560e-03 kg*m^2
    Izz (yaw axis)      : 8.8010e-03 kg*m^2
    Ixx == Iyy          : true
  aerodynamic coefficients:
    kF (thrust)         : 2.9800e-06 N*s^2/rad^2
    kM (drag torque)    : 1.1400e-07 N*m*s^2/rad^2
    kM/kF ratio         : 0.0383
  hover calculations:
    Thrust per motor    : 1.2263 N
    Motor speed (hover) : 641.5 rad/s
    Motor speed (hover) : 6126 RPM
==============================================================
  all sanity checks passed.
  ready to run dynamics.m and simulate_openloop.m
==============================================================
Running Test Case 1 (pure roll torque from rest)...
  Done. 61 time steps computed.
Running Test Case 2 (roll torque + initial yaw rate)...
  Done. 165 time steps computed.
Phase 1 — Model Validation Results
  Check 1 — angular acceleration accuracy (Test 1)
    applied roll torque    : 0.0050 N*m
    expected alpha (tau/I) : 1.0297 rad/s^2
    simulated alpha (avg)  : 1.0297 rad/s^2
    percentage error       : 0.0000%
    Result                 : PASS  (error < 1%)
  Check 2 — gyroscopic coupling active (Test 2)
  initial yaw rate       : 0.50 rad/s  (28.6 deg/s)
    applied roll torque    : 0.0050 N*m
    tau_theta commanded    : 0 N*m  (no pitch torque)
    max pitch response     : 223.9165 deg  (from coupling only)
    Result                 : PASS  (coupling term is active)
  Check 3 — kinematic prediction match (first 1 second)
    comparing simulation vs  0.5*(tau/Ixx)*t^2
    max error (t = 0 to 1s) : 3.33e-16 rad  (1.91e-14 deg)
    Result                  : PASS  (error < 1e-5 rad)
  Overall: All Three Checks Passed
  your dynamics.m model is physically correct.
  ready to proceed to phase 2 (PID controller design).
  Quick Reference — key values from simulation:
    hover motor speed        : 6126 RPM  (641.5 rad/s)
    roll alpha (expected)    : 1.0297 rad/s^2
    roll alpha (simulated)   : 1.0297 rad/s^2
    roll angle at t=4s       : 471.96 deg
    roll rate  at t=4s       : 235.98 deg/s
    pitch at t=4s (Test 1)   : 0.000000 deg  (zero, symmetric)
    pitch at t=4s (Test 2)   : 223.92 deg  (nonzero, coupling)
figures saved as PDF files.
