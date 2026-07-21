#!/usr/bin/env bash
# Compute per-trajectory ATE mean and spread from all floor_<traj>_* runs.
# Works for any number of runs per trajectory (discovers whatever exists).
cd ~/sensor-fault-benchmark

for t in box straight figure_eight; do
  vals=()
  # find every est file for this trajectory, however many there are
  for f_est in results/floor_${t}_*_est.tum; do
    [ -f "$f_est" ] || continue
    f_ref="${f_est/_est.tum/_ref.tum}"
    [ -f "$f_ref" ] || continue
    r=$(evo_ape tum "$f_ref" "$f_est" --t_max_diff 0.02 2>/dev/null | awk '/rmse/{print $2}')
    [ -n "$r" ] && vals+=("$r")
  done

  python3 -c "
import statistics as st
v = [float(x) for x in '${vals[*]}'.split()]
if not v:
    print('$t: no runs found')
else:
    m = st.mean(v)
    s = st.pstdev(v) if len(v) > 1 else 0.0
    spread = 100*(max(v)-min(v))/m if len(v) > 1 else 0.0
    print(f'$t (n={len(v)}): mean={m:.4f}  std={s:.4f}  spread={spread:.1f}%  runs={[round(x,4) for x in v]}')
"
done
