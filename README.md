# 2-Stage Ballistic Missile GNC Simulator

> Closed-loop 2DOF GNC simulator for a two-stage ballistic missile, implementing a full Guidance, Navigation, and Control architecture in MATLAB & Simulink.

---

## Table of Contents

- [Project Goal](#project-goal)
- [Technical Architecture](#technical-architecture)
  - [Layer 1 — Simulation Orchestration & Initialization](#layer-1--simulation-orchestration--initialization)
  - [Layer 2 — Plant / Vehicle Dynamics](#layer-2--plant--vehicle-dynamics)
  - [Layer 3 — GNC Loop](#layer-3--gnc-loop)

---

## Project Goal

Design and implementation of a **closed-loop 2DOF GNC** *(Guidance, Navigation, and Control)* simulator for a **two-stage ballistic missile**. The primary objective is to track an optimal ascent trajectory using a **Sine Pitch Program**, ensuring that TVC actuator deflection limits are not violated at hypersonic velocities. The physical plant and control loop are implemented in **Simulink**, with simulation orchestration and telemetry generation handled via **MATLAB scripting**.

---

## Technical Architecture

The software is organized into three interconnected layers:

```
┌──────────────────────────────────────────────────────────────┐
│  Layer 1 · Simulation Orchestration & Initialization (MATLAB)│
├──────────────────────────────────────────────────────────────┤
│  Layer 2 · Plant / Vehicle Dynamics              (Simulink)  │
├──────────────────────────────────────────────────────────────┤
│  Layer 3 · GNC Loop                              (Simulink)  │
└──────────────────────────────────────────────────────────────┘
```

---

### Layer 1 — Simulation Orchestration & Initialization

**Toolchain:** MATLAB (`main.m`)

Central scripting layer responsible for setting up and running the full simulation pipeline.

| Element | Description |
|---|---|
| **Initialization** | Planetary constants, atmospheric model parameters, vehicle mass properties |
| **Simulation deployment** | Automated Simulink model execution from script |
| **Telemetry generation** | Extraction of plant states and automated generation of flight and actuator effort reports |

---

### Layer 2 — Plant / Vehicle Dynamics

**Toolchain:** Simulink

#### Translational Dynamics

2DOF point-mass equations of motion over a **spherical, rotating Earth**:
- Variable gravity model $g(z)$
- ISA standard atmosphere $\rho(z)$
- Aerodynamic drag force $D$

#### Thrust Vectoring Kinematics

The TVC nozzle deflection $\delta_c$ is projected directly onto the translational equations of motion, modifying the flight path angle rate $\dot{\gamma}$. A zero angle-of-attack assumption ($\alpha = 0$) is applied throughout, meaning the vehicle is assumed to remain aerodynamically aligned with the velocity vector at all times.

#### Mass Management

- Continuous propellant consumption via mass flow rate $\dot{m}$
- Discrete **staging event**: instantaneous structural mass jettison at Stage 1 → Stage 2 separation

---

### Layer 3 — GNC Loop

**Toolchain:** Simulink

#### Guidance

Generates the reference signal $\gamma_{ref}$ (desired flight path angle) using two sequential strategies:

- **Stage 1 — Gravity Turn:** open-loop attitude profile driven by velocity vector rotation
- **Stage 2 — Sine Pitch Program:** trigonometric pitch profile that commands maximum turn rates at low dynamic pressure and drives the turn rate to zero at MECO, preventing TVC actuator saturation at hypersonic velocities

#### Control

A **PID controller** with **Clamping anti-windup** commands the TVC actuator, computing the required nozzle deflection $\delta_c$ to track $\gamma_{ref}$. Physical actuator limits of $\pm 10°$ are enforced via **Saturation blocks**, preventing integrator wind-up and ensuring safe operation across the full flight envelope, including the high dynamic pressure regime.

---

