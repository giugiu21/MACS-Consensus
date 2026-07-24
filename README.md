# MACS-Consensus

Event-triggered consensus control for a network of second-order
(mass-spring-damper) agents, in MATLAB. Each agent measures only its own
position and exchanges information with its neighbours over a communication
graph. The project compares **continuous** communication with
**event-triggered** communication, where a message is sent only when a local
error crosses a threshold, showing that consensus is reached while using only a
small fraction of the messages.

## Main features

- Consensus of mass-spring-damper agents over a graph (path, cycle or complete
  topology).
- Distributed consensus controller, in **leaderless** and
  **leader-follower** configurations.
- Communication modes: continuous, event-triggered with **Zero-Order Hold**,
  and event-triggered **model-based** (the last received state is predicted
  with the open-loop model between events, Garcia-style).
- Four trigger rules: `absolute`, `relative`, `state-relative`,
  `state-disagreement`.
- Damped and undamped plants; bringing the agents to a set-point,
  and trajectory tracking.

## Requirements

MATLAB with the Control System Toolbox.

## How to run

Open MATLAB and run any main script, for example:

```matlab
cd Continuous_linear_system
main_undamped
```

Outputs (figures and videos) are written to `Continuous_linear_system/results/`.

## Main scripts: what each one lets you test

| Script | What it tests |
|--------|---------------|
| `main_damped.m` | Damped agents reaching consensus at the equilibrium: open-loop vs continuous vs event-triggered, leaderless and leader-follower. |
| `main_damped_position.m` | Damped agents reaching consensus at a **desired position** (set-point different from zero), leaderless and leader-follower. |
| `main_undamped.m` | Undamped agents : continuous vs Zero-Order Hold vs **model-based** hold, all trigger rules, leaderless and leader-follower. |
| `main_undamped_trajectory.m` | Undamped **leader-follower** case tracking a desired trajectory. |
| `compare_damped_trigger_thresholds.m` | Side-by-side comparison of the four trigger rules (number of events vs final consensus error) for the damped plant. |
| `heterogeneous agents (leader-follower)/main_chung_kia.m` | Heterogeneous LTI followers tracking an active nonlinear leader from sampled measurements (Chung & Kia, controllability-Gramian based). |

## Repository layout

```
Continuous_linear_system/
  agents/         plant definitions (damped, undamped, heterogeneous)
  graph/          communication graph and local disagreement
  control/        LQR gain and control laws
  dynamics/       closed-loop right-hand sides
  triggers/       event-trigger rules and updates
  simulation/     open-loop, continuous and event-triggered runners
  analysis/       consensus and convergence checks
  visualization/  MP4 animation of the agents
  results/        generated figures and videos
  heterogeneous agents (leader-follower)/   Chung & Kia experiment
```
