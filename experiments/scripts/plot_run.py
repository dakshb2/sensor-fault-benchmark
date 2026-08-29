#!/usr/bin/env python3
"""Plot a benchmark run: trajectory overlay and position error over time.

    python3 experiments/scripts/plot_run.py <prefix> [--fault-start SECONDS]

<prefix> is the run prefix without the _est.tum / _ref.tum suffix, e.g.

    python3 experiments/scripts/plot_run.py results/sc_figure_eight_yaw_bias_0_2_

Writes <prefix>plot.png next to the input files.

Deliberately 2D-only: it never imports mpl_toolkits.mplot3d, which is broken in
this environment (system matplotlib and pip matplotlib disagree about the
`docstring` module, so evo's own plotting crashes). Everything here uses plain
pyplot, so it is unaffected.
"""

import sys
import argparse

import numpy as np
import matplotlib
matplotlib.use('Agg')          # no display needed; write straight to file
import matplotlib.pyplot as plt


def load_tum(path):
    """Read a TUM file -> (timestamps, Nx3 positions)."""
    data = np.loadtxt(path)
    if data.ndim == 1:
        data = data.reshape(1, -1)
    return data[:, 0], data[:, 1:4]


def associate(t_ref, p_ref, t_est, p_est, max_diff=0.02):
    """Match each estimate pose to the nearest reference pose in time.

    Returns the matched times (relative to the first) and both position sets,
    dropping estimates with no reference within max_diff seconds.
    """
    idx = np.searchsorted(t_ref, t_est)
    idx = np.clip(idx, 1, len(t_ref) - 1)
    left, right = t_ref[idx - 1], t_ref[idx]
    idx = np.where(np.abs(t_est - left) < np.abs(t_est - right), idx - 1, idx)

    keep = np.abs(t_ref[idx] - t_est) <= max_diff
    t = t_est[keep]
    return t - t[0], p_ref[idx[keep]], p_est[keep]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('prefix', help='run prefix, without _est.tum/_ref.tum')
    parser.add_argument('--fault-start', type=float, default=None,
                        help='mark the fault onset at this many seconds in')
    parser.add_argument('--title', default=None)
    args = parser.parse_args()

    prefix = args.prefix
    ref_path = f'{prefix}ref.tum' if prefix.endswith('_') else f'{prefix}_ref.tum'
    est_path = f'{prefix}est.tum' if prefix.endswith('_') else f'{prefix}_est.tum'

    try:
        t_ref, p_ref = load_tum(ref_path)
        t_est, p_est = load_tum(est_path)
    except OSError as exc:
        sys.exit(f'Could not read run files: {exc}')

    t, ref, est = associate(t_ref, p_ref, t_est, p_est)
    if len(t) == 0:
        sys.exit('No poses matched between the two files.')

    error = np.linalg.norm(ref - est, axis=1)
    rmse = float(np.sqrt(np.mean(error ** 2)))

    # cumulative RMSE: the score as it would stand if the run ended here
    cumulative = np.sqrt(np.cumsum(error ** 2) / np.arange(1, len(error) + 1))

    fig, (ax_xy, ax_err) = plt.subplots(1, 2, figsize=(13, 5.5))

    # --- trajectory overlay ---
    ax_xy.plot(ref[:, 0], ref[:, 1], linewidth=2.0,
               label='ground truth', color='#2b2b2b')
    ax_xy.plot(est[:, 0], est[:, 1], linewidth=1.8,
               label='estimate', color='#c1440e')
    ax_xy.plot(ref[0, 0], ref[0, 1], 'o', color='#2b2b2b', markersize=7)
    ax_xy.plot(est[-1, 0], est[-1, 1], 's', color='#c1440e', markersize=7,
               label='estimate end')
    ax_xy.plot(ref[-1, 0], ref[-1, 1], 's', color='#2b2b2b', markersize=7,
               label='true end')

    if args.fault_start is not None:
        k = int(np.searchsorted(t, args.fault_start))
        if 0 <= k < len(t):
            ax_xy.plot(ref[k, 0], ref[k, 1], '*', color='#1f6feb',
                       markersize=16, label=f'fault at {args.fault_start:g}s')

    ax_xy.set_aspect('equal', adjustable='datalim')
    ax_xy.set_xlabel('x (m)')
    ax_xy.set_ylabel('y (m)')
    ax_xy.set_title('Trajectory')
    ax_xy.legend(fontsize=8, loc='best')
    ax_xy.grid(alpha=0.3)

    # --- error over time ---
    ax_err.plot(t, error, linewidth=1.2, color='#c1440e',
                label='instantaneous error')
    ax_err.plot(t, cumulative, linewidth=2.0, color='#1f6feb',
                label='cumulative RMSE')
    if args.fault_start is not None:
        ax_err.axvline(args.fault_start, color='#1f6feb', linestyle='--',
                       alpha=0.7, label=f'fault at {args.fault_start:g}s')
    ax_err.set_xlabel('time since trial start (s)')
    ax_err.set_ylabel('position error (m)')
    ax_err.set_title(f'Error over time  (final RMSE {rmse:.3f} m)')
    ax_err.legend(fontsize=8, loc='best')
    ax_err.grid(alpha=0.3)

    fig.suptitle(args.title or prefix.rstrip('_').split('/')[-1])
    fig.tight_layout()

    out = f'{prefix}plot.png' if prefix.endswith('_') else f'{prefix}_plot.png'
    fig.savefig(out, dpi=130)
    print(f'RMSE {rmse:.6f}  ({len(t)} matched poses)')
    print(f'Wrote {out}')


if __name__ == '__main__':
    main()