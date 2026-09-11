# CA-PIBT — Execution-Driven Congestion-Adaptive Guidance for Lifelong MAPF

Code, data and paper for *Execution-Driven Congestion-Adaptive Guidance for Lifelong Multi-Robot Warehouse Navigation* (Parth Oza and Meet Patel, 2026). Paper: [`paper/paper.pdf`](paper/paper.pdf).

CA-PIBT is PIBT with edge costs derived from what the fleet actually did: exponentially-decayed, direction-resolved traversal rates (to penalise head-on traffic) and a wait-heat map (to penalise hot spots). No offline optimisation, no learning, no per-map tuning. Per-robot distance tables are rebuilt on a staggered schedule so the amortised cost stays at a few milliseconds per step on small maps.

## Results in one line
On the League of Robot Runners `warehouse_small` layout with 400 robots: **8.77** tasks/step vs **4.49** for hand-designed highways and **2.91** for plain PIBT. On `random-32-32-20` it more than doubles both. On `kiva` and `sortation_small`, whose wide corridors already suit static lanes, it ties with highways. Under a 20% per-step execution-delay probability it retains 60% of throughput where highway-PIBT retains 23%. On the 140x500 map at 1,000-4,000 robots the gain over highways is only 1-3% and planning cost reaches ~1.35 s/step — reported in the paper as a limitation.

## Layout
```
code/     sim.cpp (all four planners), analyze.py, gen_jobs.py, jobs_all.txt, sweep*.csv
maps/     LoRR 2023/24 maps, RHCR kiva.map, MovingAI random-32-32-20
results/  main.csv (1015 runs), delay.csv (240), large.csv (36) — one row per run
paper/    paper.pdf
```

## Build and run
```bash
cd code
g++ -O2 -std=c++17 -o sim sim.cpp
./sim --map ../maps/warehouse_small.map --planner capibt --agents 400 --steps 1000 --seed 1
./sim --map ../maps/warehouse_small.map --planner hpibt  --agents 400 --steps 1000 --seed 1
./sim --map ../maps/warehouse_small.map --planner wpp    --agents 200 --steps 1000 --seed 1 --window 10 --horizon 20
```
Planners: `pibt`, `hpibt` (static highways), `wpp` (windowed prioritised planning, a simplified RHCR), `capibt` (this work).

Flags: `--alpha` head-on flow weight (default 10), `--gamma` wait-heat weight (5), `--decay` lambda (0.97), `--refresh` table refresh interval K (20), `--stallw` stall window W (30), `--cahw 0|1` highway base cost, `--recover 0|1` stall recovery, `--delay p` per-step execution-delay probability, `--out file.csv --label NAME` append a result row.

CSV columns: `map,planner,agents,steps,seed,delay,throughput,wait_frac,stall_events,recoveries,conflicts_resolved,delays_injected,ms_per_step,task_std,alpha,gamma,stallW,window`.

## Reproduce every number in the paper
```bash
cd code
python3 gen_jobs.py                                    # writes jobs_all.txt (1291 runs)
cat jobs_all.txt | xargs -P $(nproc) -I{} sh -c "{}"
python3 analyze.py                                     # figures, tables and macros into ../tex
```
`analyze.py` is the single source of truth: every figure, table and inline number in the paper is generated from `results/*.csv` by that script. Nothing is typed by hand. A few CPU-hours for the small maps (WPP dominates), a few more for the 140x500 runs.

## Model
4-connected grid, unit-speed holonomic agents, vertex and swap conflicts forbidden, following allowed. Tasks alternate between a random service (`S`) and emitter (`E`) cell on LoRR maps, `e`/`r` on `kiva.map`, uniform free cells otherwise. Throughput = goals reached per timestep. Every planner runs through the same execution layer, which resolves residual conflicts and can inject delays. A run aborts with exit code 2 if two agents ever share a cell (never observed).

## Maps
`warehouse_small`, `sortation_small`, `warehouse_large`, `sortation_*` from the [League of Robot Runners](https://github.com/MAPF-Competition) benchmark archive; `kiva.map` from [RHCR](https://github.com/Jiaoyang-Li/RHCR); `random-32-32-20` from the [MovingAI MAPF benchmark](https://movingai.com/benchmarks/mapf.html). Redistributed here for reproducibility under their original terms.

## License
MIT (see `LICENSE`). Maps belong to their original authors.
