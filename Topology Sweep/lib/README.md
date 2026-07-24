# lib/ — vendored core functions

Copie delle funzioni di cui `main_topology_sweep.m` (e i suoi helper in
`functions/`) hanno bisogno, raccolte qui per rendere la cartella
`Topology Sweep` **autonoma**: lo sweep gira senza aggiungere al path nessun
altro modulo del repository.

14 file provengono da `Continuous Linear System/` (sottocartella d'origine tra
parentesi):

| File | Origine |
|---|---|
| `init_mass_spring_damper_agent.m` | `agents/` |
| `build_consensus_graph.m` | `graphs/` |
| `compute_local_disagreement.m` | `graphs/` |
| `compute_lqr_consensus_gain.m` | `control/` |
| `compute_event_triggered_control.m` | `control/` |
| `compute_continuous_control.m` | `control/` |
| `check_consensus_modes.m` | `analysis/` |
| `event_triggered_linear_rhs.m` | `dynamics/` |
| `continuous_linear_rhs.m` | `dynamics/` |
| `update_event_triggers.m` | `triggers/` |
| `evaluate_trigger_condition.m` | `triggers/` |
| `run_event_triggered_consensus.m` | `simulation/` |
| `run_continuous_consensus.m` | `simulation/` |
| `make_random_initial_condition.m` | `simulation/` |

2 file provengono da `Experiments/Utilities/` (servono per il ramo `undamped`
e per la classificazione degli esiti):

| File | Origine |
|---|---|
| `init_undamped_agent.m` | `Experiments/Utilities/` |
| `classify_run_outcome.m` | `Experiments/Utilities/` |

Restano invece **locali alla cartella** (non in `lib/`): gli helper in
`functions/` — `sweep_network_topology.m`, `measure_convergence_time.m`,
`plot_topology_sweep.m`, `plot_ring_gallery.m` — e la subfunction
`make_topology` definita dentro `main_topology_sweep.m`.

**Unica dipendenza esterna non eliminabile:** `compute_lqr_consensus_gain`
usa `lqr`, quindi serve il Control System Toolbox di MATLAB (è una toolbox,
non un file del progetto).

Essendo copie, questi file non si aggiornano automaticamente se cambiano gli
originali. Per risincronizzarli, ricopiare i file dalle rispettive origini.
