# Phase 1 — Quadcopter System Modeling

## Goal
Build and validate the nonlinear rotational dynamics model of the quadcopter before implementing any controller.

---

# Learning Fundamentals

## MATLAB + Programming Basics
- [ ] Complete MATLAB Onramp
- [ ] Understand MATLAB Scripts vs Functions
- [ ] Learn Vectors, Matrices, and Indexing
- [ ] Practice Plotting Graphs in MATLAB
- [ ] Understand Function Inputs and Outputs
- [ ] Learn How to Create and use Structs

---

## Mathematics + Dynamics
- [ ] Understand Derivatives Physically
- [ ] Understand Ordinary Differential Equations
- [ ] Learn Basic Linear Algebra Intuition
- [ ] Understand Vectors and Matrices Geometrically
- [ ] Learn Rotational Newton's Law:
  \[
  \tau = I\alpha
  \]
- [ ] Understand Moment of Inertia Physically
- [ ] Understand Angular Velocity and Angular Acceleration

---

## Quadcopter Dynamics
- [ ] Learn body frame vs inertial frame
- [ ] Understand roll, pitch, and yaw
- [ ] Understand ZYX rotation order
- [ ] Derive rotation matrices
- [ ] Understand gyroscopic coupling
- [ ] Understand why nonlinear coupling exists
- [ ] Understand why SMC is needed for nonlinear systems

---

# Mathematical Modeling

## Reference Frames
- [ ] Define inertial frame
- [ ] Define body frame
- [ ] Understand Euler angles
- [ ] Derive full rotation matrix:
  \[
  R = R_zR_yR_x
  \]

---

## Inertia Tensor
- [ ] Define inertia matrix
- [ ] Understand diagonal inertia tensor
- [ ] Estimate realistic inertia values
- [ ] Understand why \(I_{zz} > I_{xx}\)

---

## Newton-Euler Equations
- [ ] Write rotational dynamics equation:
  \[
  I\dot{\omega} = \tau - \omega \times (I\omega)
  \]
- [ ] Expand roll equation
- [ ] Expand pitch equation
- [ ] Expand yaw equation
- [ ] Verify coupling term signs carefully

---

## State-Space Representation
- [ ] Define state vector
- [ ] Define input vector
- [ ] Convert second-order equations to first-order
- [ ] Write full nonlinear state equations

---

# MATLAB Implementation

## Project Setup
- [ ] Create project folder structure
- [ ] Create `parameters.m`
- [ ] Create `dynamics.m`
- [ ] Create `simulate_openloop.m`

---

## Parameters File
- [ ] Define drone mass
- [ ] Define arm length
- [ ] Define inertia values
- [ ] Define thrust coefficient
- [ ] Define drag coefficient
- [ ] Calculate hover RPM
- [ ] Verify hover RPM is realistic

---

## Dynamics Function
- [ ] Define state variables
- [ ] Define control inputs
- [ ] Implement roll dynamics
- [ ] Implement pitch dynamics
- [ ] Implement yaw dynamics
- [ ] Assemble state derivative vector
- [ ] Add detailed comments

---

## Open-Loop Simulation
- [ ] Configure `ode45`
- [ ] Set tight solver tolerances
- [ ] Define initial conditions
- [ ] Apply small roll torque input
- [ ] Run open-loop simulation
- [ ] Plot roll angle response
- [ ] Plot roll rate response
- [ ] Plot pitch coupling response
- [ ] Plot yaw coupling response

---

# Validation Checks

## Numerical Validation
- [ ] Verify roll acceleration matches theory
- [ ] Ensure error is below 1%
- [ ] Compare simulated vs analytical roll response

---

## Nonlinear Coupling Validation
- [ ] Verify pitch response is nonzero
- [ ] Verify yaw response is nonzero
- [ ] Confirm coupling terms are active

---

## Physical Validation
- [ ] Verify roll angle grows quadratically
- [ ] Verify roll rate grows linearly
- [ ] Verify system behaves physically correctly

---

# Documentation

## Report Preparation
- [ ] Save all plots
- [ ] Screenshot validation figures
- [ ] Document assumptions
- [ ] Document parameter values
- [ ] Write system modeling explanation
- [ ] Explain physical meaning of coupling terms
- [ ] Add equations to report

---

# Self-Understanding Checklist

Before moving to Phase 2, confirm you can explain:

- [ ] What each state variable means
- [ ] Why the state vector has 6 states
- [ ] Why equations are written in body frame
- [ ] Why radians are mandatory
- [ ] Why coupling terms exist
- [ ] Why nonlinear systems need robust control
- [ ] What `ode45` actually does
- [ ] What `dxdt = f(x,u)` means physically

---

# Phase 1 Completion Criteria

Phase 1 is complete only if:

- [ ] Open-loop model runs successfully
- [ ] Validation error is below 1%
- [ ] Coupling terms are visible
- [ ] All plots make physical sense
- [ ] Every equation is understood physically
- [ ] Every line of code is understood
- [ ] Results are documented properly

---
