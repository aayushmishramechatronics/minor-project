# Phase 1 — Quadcopter System Modeling

## Goal
Build and Validate the Nonlinear Rotational Dynamics Model of the Quadcopter before Implementing any Controller.

---

# Learning Fundamentals

## MATLAB + Programming Basics
- [ ] Complete MATLAB Onramp
- [ ] Understand MATLAB Scripts vs Functions
- [ ] Learn Vectors, Matrices, and Indexing
- [ ] Understand Function Inputs and Outputs
- [ ] Learn How to Create and use Structs

---

## Mathematics + Dynamics
- [ ] Understand Derivatives Physically
- [ ] Understand Ordinary Differential Equations
- [ ] Learn Basic Linear Algebra Intuition
- [ ] Understand Vectors and Matrices Geometrically
- [ ] Learn Rotational Newton's Law
- [ ] Understand Moment of Inertia Physically
- [ ] Understand Angular Velocity and Angular Acceleration

---

## Quadcopter Dynamics
- [ ] Learn Body Frame vs Inertial Frame
- [ ] Understand Roll, Pitch, and Yaw
- [ ] Understand ZYX Rotation Order
- [ ] Derive Rotation Matrices
- [ ] Understand Gyroscopic Coupling
- [ ] Understand Why Nonlinear Coupling exists
- [ ] Understand Why SMC is needed for Nonlinear Systems

---

# Mathematical Modeling

## Reference Frames
- [ ] Define Inertial Frame
- [ ] Define Body Frame
- [ ] Understand Euler Angles
- [ ] Derive Full Rotation Matrix
---

## Inertia Tensor
- [ ] Define Inertia Matrix
- [ ] Understand Diagonal Inertia Tensor
- [ ] Estimate Realistic Inertia Values
- [ ] Understand Why \(I_{zz} > I_{xx}\)

---

## Newton-Euler Equations
- [ ] Write Rotational Dynamics Equation
- [ ] Expand Roll Equation
- [ ] Expand Pitch Equation
- [ ] Expand Yaw Equation
- [ ] Verify Coupling Term Signs Carefully

---

## State-Space Representation
- [ ] Define State Vector
- [ ] Define Input Vector
- [ ] Convert Second-Order Equations to First-Order
- [ ] Write Full Nonlinear State Equations

---

# MATLAB Implementation

## Project Setup
- [ ] Create Project Folder Structure
- [ ] Create `parameters.m`
- [ ] Create `dynamics.m`
- [ ] Create `simulate_openloop.m`

---

## Parameters File
- [ ] Define Drone Mass
- [ ] Define Arm Length
- [ ] Define Inertia Values
- [ ] Define Thrust Coefficient
- [ ] Define Drag Coefficient
- [ ] Calculate Hover RPM
- [ ] Verify Hover RPM is Realistic

---

## Dynamics Function
- [ ] Define State Variables
- [ ] Define Control Inputs
- [ ] Implement Roll Dynamics
- [ ] Implement Pitch Dynamics
- [ ] Implement Yaw Dynamics
- [ ] Assemble State Derivative Vector
- [ ] Add Detailed Comments

---

## Open-Loop Simulation
- [ ] Configure `ode45`
- [ ] Set Tight Solver Tolerances
- [ ] Define Initial Conditions
- [ ] Apply Small Roll Torque Input
- [ ] Run Open-loop Simulation
- [ ] Plot Roll Angle Response
- [ ] Plot Roll Rate Response
- [ ] Plot Pitch Coupling Response
- [ ] Plot Yaw Coupling Response

---

# Validation Checks

## Numerical Validation
- [ ] Verify Roll Acceleration Matches Theory
- [ ] Ensure Error is Below 1%
- [ ] Compare Simulated vs Analytical Roll Response

---

## Nonlinear Coupling Validation
- [ ] Verify Pitch Response is Nonzero
- [ ] Verify Yaw Response is Nonzero
- [ ] Confirm Coupling Terms are Active

---

## Physical Validation
- [ ] Verify Roll Angle Grows Quadratically
- [ ] Verify Roll Rate Grows Linearly
- [ ] Verify System Behaves Physically Correctly

---

# Documentation

## Report Preparation
- [ ] Save All Plots
- [ ] Screenshot Validation Figures
- [ ] Document Assumptions
- [ ] Document Parameter Values
- [ ] Write System Modeling Explanation
- [ ] Explain Physical Meaning of Coupling Terms
- [ ] Add Equations to Report

---

# Self-Understanding Checklist

Before Moving to Phase 2, Confirm you can Explain:

- [ ] What Each State Variable means
- [ ] Why the State Vector has 6 States
- [ ] Why Equations are Written in Body Frame
- [ ] Why Radians are Mandatory
- [ ] Why Coupling Terms Exist
- [ ] Why Nonlinear Systems need Robust Control
- [ ] What `ode45` actually Does
- [ ] What `dxdt = f(x,u)` means Physically

---

# Phase 1 Completion Criteria

Phase 1 is Complete Only If:

- [ ] Open-loop Model Runs Successfully
- [ ] Validation Error is Below 1%
- [ ] Coupling Terms are Visible
- [ ] All Plots make Physical Sense
- [ ] Every Equation is Understood Physically
- [ ] Every Line of Code is Understood
- [ ] Results are Documented Properly

---
