# Simulation

## One-leg IK visualizer (`/one_leg`)

`IK_visualizer.m` is a MATLAB tool for validating the 3-DOF leg
inverse-kinematics solution (hip/thigh/shin) used on the Servo2040
before it ever touches hardware. It renders a single leg in 3D and
gives you live sliders for target X/Y/Z plus a knee-up/knee-down
toggle, redrawing the link chain and annotating each joint angle
(θ_H, θ_T, θ_S) in degrees as you move them. It shares the same
planar-IK-after-hip-rotation approach as `kinematics/servo2040/main.cpp`
(`solve_ik`), so it doubles as a sanity check when leg geometry
(`L2`, `L3`) or the knee convention changes.

```matlab
IK_visualizer
```

## Full hexapod (Simscape) — in progress

The one-leg model has been scaled up into a full hexapod Simscape
Multibody simulation (all 6 legs, body dynamics, gait), currently in
progress. It'll land in this folder once the model is ready, along
with the CAD it's built from.
