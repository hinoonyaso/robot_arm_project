# Experiment Protocol

1. Launch the simulation and task manager.
2. Execute pick & place trials with preset object poses (5 presets × 4 repeats).
3. Log all events to `events.jsonl` for each run.
4. Generate `summary.json` with `parse_metrics.py`.
5. Render plots with `make_plots.py`.

## Metrics
- Success rate
- Planning time distribution
- Failure type ratio
- Recovery success rate
- Max joint velocity (proxy for smoothness)
