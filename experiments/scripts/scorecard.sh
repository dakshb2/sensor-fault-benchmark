#!/usr/bin/env bash
# Fault scorecard: sweep every fault condition across every trajectory, score
# each against the same-session clean run, and mark whether the degradation
# clears that trajectory's noise floor.
#
#   ./experiments/scripts/scorecard.sh
#
# Produces results/scorecard.md and prints it.
#
# TIMING: fault windows are set PER TRAJECTORY so they always land inside the
# driving phase. The trajectories differ a lot in length (straight drives for
# 8s, box 33s, figure_eight 24s), so a single absolute start time would fire
# after a short trajectory had already finished -- producing runs that look
# clean because no fault ever occurred.

set -eo pipefail
cd ~/sensor-fault-benchmark

RUNNER="./experiments/scripts/run_trial.sh"
OUT="results/scorecard.md"
mkdir -p results

# --- conditions: label | sensor | fault spec | bounded window? ---
# Faults marked 'window' get an explicit duration; 'persist' faults run from
# their start time to the end of the trajectory.
CONDITIONS=(
  "clean|||"
  "yaw_bias:0.2|imu|yaw_bias:0.2|persist"
  "drift:0.02|imu|drift:0.02|persist"
  "dropout|wheel|dropout|window"
  "freeze|wheel|freeze|window"
  "slip:0.5|wheel|slip:0.5|persist"
)

TRAJECTORIES=(box straight figure_eight)

# --- per-trajectory fault timing (sim seconds from trial start) ---
declare -A START=(  [box]=10 [straight]=3 [figure_eight]=12 )
declare -A WINDOW=( [box]=9  [straight]=3 [figure_eight]=9  )

# --- per-trajectory noise floor (fraction), from measured n=5..10 baselines ---
declare -A FLOOR=( [box]=0.25 [straight]=0.22 [figure_eight]=0.01 )

declare -A ATE
for traj in "${TRAJECTORIES[@]}"; do
  for cond in "${CONDITIONS[@]}"; do
    IFS='|' read -r label sensor spec kind <<< "$cond"

    if [ -z "$sensor" ]; then
      flags=""
    elif [ "$kind" = "window" ]; then
      flags="--$sensor ${spec}@${START[$traj]}:${WINDOW[$traj]}"
    else
      flags="--$sensor ${spec}@${START[$traj]}"
    fi

    prefix="sc_$(echo "${traj}_${label}" | tr -c 'a-zA-Z0-9_' '_')"
    echo "=== $traj / $label  ${flags:-(clean)} ==="
    # shellcheck disable=SC2086
    $RUNNER "experiments/trajectories/${traj}.yaml" "$prefix" $flags >/dev/null 2>&1 || true

    ref="results/${prefix}_ref.tum"
    est="results/${prefix}_est.tum"
    if [ -s "$ref" ] && [ -s "$est" ]; then
      rmse=$(evo_ape tum "$ref" "$est" --t_max_diff 0.02 2>/dev/null \
             | awk '/rmse/{print $2}')
    else
      rmse=""
    fi
    ATE["$label|$traj"]="$rmse"
    echo "    ATE = ${rmse:-FAILED}"
  done
done

{
  echo "# Fault scorecard"
  echo
  echo "ATE RMSE (m), one run per cell. A fault is marked significant (\`Y\`)"
  echo "when its increase over the same-session clean run exceeds that"
  echo "trajectory's measured noise floor, otherwise \`n\`."
  echo
  echo "Noise floors: box 25%, straight 22%, figure_eight 1%."
  echo "Fault start / window (sim s): box 10/9, straight 3/3, figure_eight 12/9."
  echo
  echo "| fault | box | straight | figure_eight |"
  echo "|-------|-----|----------|--------------|"

  for cond in "${CONDITIONS[@]}"; do
    IFS='|' read -r label _ _ _ <<< "$cond"
    row="| \`$label\` |"
    for traj in "${TRAJECTORIES[@]}"; do
      val="${ATE[$label|$traj]}"
      clean="${ATE[clean|$traj]}"
      if [ -z "$val" ]; then
        cell="FAILED"
      elif [ "$label" = "clean" ]; then
        cell="$val"
      else
        sig=$(python3 -c "
v=float('$val'); c=float('$clean'); f=${FLOOR[$traj]}
print('Y' if (v-c)/c > f else 'n')
")
        cell="$val ($sig)"
      fi
      row="$row $cell |"
    done
    echo "$row"
  done
} | tee "$OUT"

echo
echo "Wrote $OUT"