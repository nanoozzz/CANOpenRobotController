#!/usr/bin/env python3
"""
block2_force_sharing.py - who pushed, and how much? Force sharing between the participant and the robot in
Block 2 (M2FittsRobotHumanMachine), per trial and per autonomy level alpha.

Usage
    python3 block2_force_sharing.py PATH [PATH ...] [--window move|all] [--phase block|warmup|all] [--out DIR]
                                    [--trace TRIAL [TRIAL ...]]

    PATH is the raw log written by the app, M2FittsRobotHuman_<participant>_B2_<date-time>_raw.csv (in results_dir,
    ../logs by default), or a folder: every M2FittsRobotHuman_*_raw.csv in it is analysed. Without any PATH, the
    files listed in RAW_FILES below are used (handy with an editor's Run button).

    --window move  target onset -> final entry into the target (the MT window; default)
             all   the whole reach, including the dwell in the target
    Next to each raw log, the files of the same session are found by name and read when present:
    <same name>_parameters.txt (control parameters) and <same name>_trials.csv (A, W, ID, MT, success).

Outputs (in --out, default: a "force_sharing" folder next to the first raw file), for each participant Pxx:
    force_sharing_Pxx_trials.csv    one row per trial
    force_sharing_Pxx_by_alpha.csv  mean and SD per alpha
    force_sharing_Pxx_work.png      positive work of each side, and the participant's share, against alpha
    force_sharing_Pxx_force.png     force each side put into the movement, and the participant's share, against alpha
    force_sharing_Pxx_profiles.png  mean force of each side over the normalised window, one curve per alpha
    force_sharing_Pxx_saturation.png  time spent at each force limit, and the peak hand force, against alpha
    force_sharing_Pxx_trialNN_trace.png  with --trace NN: forces, limits and saturated samples of trial NN over time
and, when several participants are analysed in one run:
    force_sharing_ALL_trials.csv    all trials, with a participant column
    force_sharing_ALL_by_alpha.csv  mean and SD per participant and alpha
    force_sharing_ALL.png           one line per participant: push force, positive work and the participant's share
The participant ID is taken from the trials file, or else from the raw file name (M2FittsRobotHuman_<Pxx>_B2_...).
Several sessions of the same participant are pooled under that participant.

Definitions (task axis x; every force is projected so that + = towards the target)
    F_h    force the participant applies to the handle, as measured (FhumanEst (N)_1). It is the total hand
           force: voluntary push plus the arm's passive reaction (inertia, stiffness) when the robot moves it.
    R_eff  robot's own contribution, i.e. its policy u_r scaled by alpha:  alpha*Fpd + stiction term
    H_eff  participant's effective contribution once the robot has removed its share of the hand force:
           F_h + RobotM2 force assist + (Fcmd - R_eff), where Fcmd - R_eff is the cancellation actually applied
           (so saturations are accounted for). Ideally H_eff = (1 - alpha) * k_h * F_h.
    H_eff + R_eff is the net force the two put on the handle. RobotM2's velocity-only friction feed-forward
    is plant compensation (present in every condition) and is attributed to neither side.

Measures (over the window): time-averaged force [N]; force put into the movement, "push" [N] = time average of
the part of a force directed towards the target (braking or resisting parts count as 0); impulse [N.s], positive
and negative mechanical work on
the handle [J] (integral of F * v dt; positive = energy put into the movement towards the target, negative =
braking or resisting), the participant's share of the positive work, the participant's physical effort
(integral of |F_h| dt), the robot's cancellation effort (integral of |Fcmd - R_eff| dt), and conflict_frac
(samples where H_eff and R_eff push in opposite directions, both above 1 N).

Saturation (always over the whole reach, dwell included). In the raw log:
    BlendSaturated          1 on samples where the cancellation cap or the command cap was hit (the app's own flag)
    Fpd (N)_1 = +-pd.f_max  the robot's PD at its limit: normal during the transport, part of the calibrated u_r
    Fcmd (N)_1 = +-command_f_max            the total command was clipped
    |alpha*k_h*FhumanEst| > cancel_f_max    the cancellation was capped: the excess of the hand force passed through
    |InteractionForce| > reach_force_limit  the hand force exceeded the limit that ends a reach after force_limit_time
    The limits are read from <same name>_parameters.txt. Per trial the script reports, for the PD, the cancellation
    and the command, the fraction of the reach and the time [s] spent saturated, the largest excess of the hand
    force over the cancellation cap, the peak interaction force and the time above reach_force_limit; trials
    with any cancellation or command saturation, or time above the force limit, are listed on screen.

Limits: a handle force sensor cannot separate a voluntary push from the arm's passive reaction, so a
participant who lets the robot drag a relaxed arm shows negative work (the arm absorbs energy), not zero.
"""
import argparse
import glob
import os
import re
import sys

import numpy as np
import pandas as pd

# RobotM2::setEndEffForceWithCompensation(F, true) (commit 017f81d): friction feed-forward while moving.
# Used only to check the logged motor force; the attribution needs just the force-assist gain (parameters file).
PLATFORM_COULOMB = 3.0  # alpha [N]
PLATFORM_VISCOUS = 0.5  # beta [N.s/m]
PLATFORM_BLEND = 0.5    # blend

# Raw logs (or folders of raw logs) analysed when the script is run without a PATH
RAW_FILES = [
    # '/home/nle/Fitts/CANOpenRobotController/logs/M2FittsRobotHuman_P01_B2_20260921-101500_raw.csv',
    'logs/M2FittsRobotHuman_P0n_B2_20260922-142022_raw.csv'
]

REACH_STATE = 3
COLS = {'Time (s)': 't', 'Position (m)_1': 'x', 'Velocity (m/s)_1': 'v', 'Force (N)_1': 'Fmotor',
        'InteractionForce (N)_1': 'Fint', 'InteractionForce (N)_2': 'Fint_y', 'StateCode': 'state', 'Phase': 'phase', 'Trial': 'trial',
        'TargetX (m)': 'xt', 'TargetHalfWidth (m)': 'hw', 'Alpha': 'alpha', 'Fpd (N)_1': 'Fpd',
        'FhumanEst (N)_1': 'Fh', 'Fcmd (N)_1': 'Fcmd', 'BlendSaturated': 'sat'}


def read_params(raw_path):
    """Control parameters of the session (defaults = app defaults of the version without them)."""
    p = {'human_force_sign': -1.0, 'platform_assist_gain': 0.2, 'platform_assist_force_threshold_N': 0.05,
         'platform_assist_velocity_threshold_mps': 0.05, 'cancel_f_max_N': 30.0,
         'stiction_N': 0.0, 'stiction_rest_speed': 0.01, 'stiction_deadband': 0.0005,
         'pd.f_max_N': 30.0, 'command_f_max_N': 50.0, 'reach_force_limit_N': 40.0}
    path = re.sub(r'_raw\.csv$', '_parameters.txt', raw_path)
    if not os.path.exists(path):
        print(f'  parameters: none found ({os.path.basename(path)}) - app defaults used, including the force limits '
              f'(pd.f_max 30 N, cancel_f_max 30 N, command_f_max 50 N, reach_force_limit 40 N)')
        return p
    print(f'  parameters: {os.path.basename(path)}')
    for line in open(path):
        m = re.match(r'\s*([\w.]+)\s*:\s*(.*)', line)
        if not m:
            continue
        key, nums = m.group(1), re.findall(r'[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?', m.group(2))
        if not nums:
            continue
        if key in p:
            p[key] = float(nums[0])
        elif key in ('pd.stiction_comp_x_N', 'robot_stiction_comp_N'):  # "2 (x alpha, below 0.01 m/s, outside 0.0005 m ...)"
            p['stiction_N'] = float(nums[0])
            if len(nums) >= 3:
                p['stiction_rest_speed'], p['stiction_deadband'] = float(nums[1]), float(nums[2])
    return p


def read_raw(path):
    df = pd.read_csv(path, skipinitialspace=True, low_memory=False)
    df.columns = [c.strip() for c in df.columns]
    missing = [c for c in COLS if c not in df.columns and c not in ('Force (N)_1', 'InteractionForce (N)_2')]
    if missing:
        sys.exit(f'{path}: missing columns {missing} - is this an M2FittsRobotHumanMachine raw log?')
    df = df[[c for c in COLS if c in df.columns]].rename(columns=COLS)
    df = df.apply(pd.to_numeric, errors='coerce').dropna(subset=['t']).reset_index(drop=True)
    return df


def reach_segments(df):
    idx = np.flatnonzero((df['state'] == REACH_STATE).to_numpy())
    if idx.size == 0:
        return []
    cut = np.flatnonzero(np.diff(idx) > 1)
    starts, ends = np.r_[idx[0], idx[cut + 1]], np.r_[idx[cut], idx[-1]]
    return [(s, e) for s, e in zip(starts, ends) if e - s >= 10]


def analyse_trial(seg, p, window, keep_trace=False):
    seg = seg[np.isfinite(seg['Fpd']) & np.isfinite(seg['Fcmd']) & np.isfinite(seg['Fh']) & np.isfinite(seg['alpha'])]
    if len(seg) < 10:
        return None
    t, x, v = seg['t'].to_numpy(), seg['x'].to_numpy(), seg['v'].to_numpy()
    a, Fh, Fint = seg['alpha'].to_numpy(), seg['Fh'].to_numpy(), seg['Fint'].to_numpy()
    Fpd, Fcmd, xt, hw = seg['Fpd'].to_numpy(), seg['Fcmd'].to_numpy(), seg['xt'].to_numpy(), seg['hw'].to_numpy()
    d = np.sign(xt[0] - x[0])
    if d == 0:
        return None

    # --- decomposition of the command (same rules as fsc::SharedControlLaw)
    stuck = (np.abs(v) < p['stiction_rest_speed']) & (np.abs(xt - x) > p['stiction_deadband']) & (Fpd != 0)
    stic = np.where(stuck & (p['stiction_N'] > 0), a * p['stiction_N'] * np.sign(Fpd), 0.)
    R = a * Fpd + stic                                   # robot's own contribution
    cancel = Fcmd - R                                    # cancellation actually applied (after saturations)
    active = (np.abs(Fint) > p['platform_assist_force_threshold_N']) & \
             (np.abs(v) > p['platform_assist_velocity_threshold_mps'])
    assist = np.where(active, -p['platform_assist_gain'] * Fint, 0.)   # RobotM2's force-proportional term
    H = Fh + assist + cancel                             # participant's effective contribution

    # consistency: cancellation expected from the logged hand force, unsaturated
    k_h = np.where(active, 1. - p['platform_assist_gain'] * p['human_force_sign'], 1.)
    expected = np.clip(-a * k_h * Fh, -p['cancel_f_max_N'], p['cancel_f_max_N'])

    # --- saturation, over the whole reach (same rules as fsc::SharedControlLaw)
    wa = np.clip(np.diff(t, append=t[-1]), 0., 0.05)
    cancel_unsat = -a * k_h * Fh                                       # cancellation the law asked for
    cancel_sat = np.abs(cancel_unsat) > p['cancel_f_max_N']
    cmd_unsat = R + np.clip(cancel_unsat, -p['cancel_f_max_N'], p['cancel_f_max_N'])
    cmd_sat = np.abs(cmd_unsat) > p['command_f_max_N']
    pd_sat = np.abs(Fpd) >= p['pd.f_max_N'] - 1e-3
    fint_norm = np.hypot(Fint, seg['Fint_y'].to_numpy()) if 'Fint_y' in seg else np.abs(Fint)
    over_limit = fint_norm > p['reach_force_limit_N']
    # samples where the command was clipped are excluded: there Fcmd - R also carries the command clipping
    mismatch = float(np.mean((np.abs(cancel - expected) > 0.05) & ~cmd_sat))
    sat = {
        'pd_sat_frac': float(np.mean(pd_sat)), 'pd_sat_s': float(np.sum(wa[pd_sat])),
        'cancel_sat_frac': float(np.mean(cancel_sat)), 'cancel_sat_s': float(np.sum(wa[cancel_sat])),
        'cancel_excess_peak_N': float(np.max(np.clip(np.abs(cancel_unsat) - p['cancel_f_max_N'], 0, None))),
        'cmd_sat_frac': float(np.mean(cmd_sat)), 'cmd_sat_s': float(np.sum(wa[cmd_sat])),
        'blend_flag_frac': float(np.mean(seg['sat'].to_numpy() > 0.5)),
        'Fint_peak_N': float(np.max(fint_norm)), 'over_force_limit_s': float(np.sum(wa[over_limit])),
        'cancel_f_max_N': p['cancel_f_max_N'], 'command_f_max_N': p['command_f_max_N'],
        'reach_force_limit_N': p['reach_force_limit_N'],
    }

    # --- window
    inside = np.abs(x - xt) <= hw
    outside = np.flatnonzero(~inside)
    captured = bool(inside[-1])
    final_entry = (outside[-1] + 1 if outside.size else 0) if captured else len(x) - 1
    end = final_entry if window == 'move' else len(x) - 1
    sl = slice(0, max(end, 1) + 1)
    w = np.clip(np.diff(t, append=t[-1])[sl], 0., 0.05)
    if w.sum() <= 0:
        return None
    Hd, Rd, Fhd, vd = d * H[sl], d * R[sl], d * Fh[sl], d * v[sl]
    pw_h, pw_r = float(np.sum(np.clip(Hd * vd, 0, None) * w)), float(np.sum(np.clip(Rd * vd, 0, None) * w))
    both = (np.abs(Hd) > 1.) & (np.abs(Rd) > 1.)
    res = {
        'alpha': float(np.round(np.median(a), 4)), 'captured': captured,
        'window_s': float(t[end] - t[0]), 'final_entry_s': float(t[final_entry] - t[0]) if captured else np.nan,
        'mean_Fh_N': float(np.average(Fhd, weights=w)),
        'mean_Heff_N': float(np.average(Hd, weights=w)), 'mean_Reff_N': float(np.average(Rd, weights=w)),
        'imp_Heff_Ns': float(np.sum(Hd * w)), 'imp_Reff_Ns': float(np.sum(Rd * w)),
        'posW_Heff_J': pw_h, 'negW_Heff_J': float(np.sum(np.clip(Hd * vd, None, 0) * w)),
        'posW_Reff_J': pw_r, 'negW_Reff_J': float(np.sum(np.clip(Rd * vd, None, 0) * w)),
        'share_posW_participant': pw_h / (pw_h + pw_r) if pw_h + pw_r > 0 else np.nan,
        'push_Fh_N': float(np.average(np.clip(Fhd, 0, None), weights=w)),
        'push_Heff_N': float(np.average(np.clip(Hd, 0, None), weights=w)),
        'push_Reff_N': float(np.average(np.clip(Rd, 0, None), weights=w)),
        'effort_Fh_Ns': float(np.sum(np.abs(Fh[sl]) * w)), 'peak_Fh_N': float(np.max(np.abs(Fh[sl]))),
        'cancel_effort_Ns': float(np.sum(np.abs(cancel[sl]) * w)),
        'conflict_frac': float(np.mean(both & (Hd * Rd < 0))),
        'saturated_frac': float(np.mean(seg['sat'].to_numpy()[sl] > 0.5)),
        'decomposition_mismatch_frac': mismatch,
    }
    res.update(sat)
    if keep_trace:                                       # per-sample series for --trace (towards the target)
        res['_trace'] = dict(t=t - t[0], R=d * R, cancel=d * cancel, cmd=d * Fcmd, Fh=d * Fh, fint=fint_norm,
                             err_mm=1000. * d * (xt - x), hw_mm=1000. * hw[0], pd_sat=pd_sat, cancel_sat=cancel_sat,
                             cmd_sat=cmd_sat, final_entry_s=res['final_entry_s'], alpha=res['alpha'], limits=sat)
    # normalised profiles (101 points) for the figure
    tn = (t[sl] - t[0]) / max(t[end] - t[0], 1e-9)
    grid = np.linspace(0., 1., 101)
    res['_prof_H'], res['_prof_R'] = np.interp(grid, tn, Hd), np.interp(grid, tn, Rd)

    # check of the logged motor force against command + RobotM2 compensation (diagnostic only)
    if 'Fmotor' in seg and np.isfinite(seg['Fmotor']).any():
        ff = np.where(active, (1 - PLATFORM_BLEND) * (PLATFORM_COULOMB * np.sign(v) + PLATFORM_VISCOUS * v), 0.)
        res['motor_force_residual_N'] = float(np.nanmedian(np.abs(seg['Fmotor'].to_numpy() - (Fcmd + assist + ff))))
    return res


def expand(items):
    """Raw log paths from files and/or folders."""
    files = []
    for item in items:
        item = os.path.expanduser(item)
        if os.path.isdir(item):
            found = sorted(glob.glob(os.path.join(item, 'M2FittsRobotHuman_*_raw.csv')))
            if not found:
                sys.exit(f'no M2FittsRobotHuman_*_raw.csv in {item}')
            files += found
        elif os.path.isfile(item):
            files.append(item)
        else:
            sys.exit(f'not found: {item}')
    return files


def participant_of(path, meta):
    """Participant ID: from the trials file, else from the raw file name (M2FittsRobotHuman_<Pxx>_B<n>_...)."""
    if meta is not None and 'participant' in meta and meta['participant'].notna().any():
        pid = str(meta['participant'].dropna().iloc[0])
    else:
        m = re.match(r'M2FittsRobotHuman_(.+?)_B\d+', os.path.basename(path))
        pid = m.group(1) if m else os.path.splitext(os.path.basename(path))[0]
    return re.sub(r'[^A-Za-z0-9_-]', '_', pid)


SUMMARY_KEYS = ['mean_Fh_N', 'mean_Heff_N', 'mean_Reff_N', 'imp_Heff_Ns', 'imp_Reff_Ns', 'posW_Heff_J', 'posW_Reff_J',
                'negW_Heff_J', 'share_posW_participant', 'push_Fh_N', 'push_Heff_N', 'push_Reff_N',
                'share_push_participant', 'effort_Fh_Ns', 'peak_Fh_N', 'cancel_effort_Ns', 'conflict_frac',
                'pd_sat_frac', 'cancel_sat_frac', 'cmd_sat_frac', 'cancel_excess_peak_N', 'Fint_peak_N',
                'over_force_limit_s']


def summarise(tr, by):
    g = tr.groupby(by)
    return pd.concat([g.size().rename('n_trials')] + [g[k].mean().rename(k + '_mean') for k in SUMMARY_KEYS] +
                     [g[k].std().rename(k + '_sd') for k in SUMMARY_KEYS], axis=1)


def plot_participant(plt, tr, summ, prof, out, pid):
    al = summ.index.to_numpy()

    def side_by_side(series, share_key, ylabel, title, share_label, share_title, name):
        """Left: each side against alpha (trials as dots, mean +- SD); right: participant's share against alpha."""
        fig, ax = plt.subplots(1, 2, figsize=(11, 4))
        for k, lab, col, ls in series:
            if ls == '-':
                ax[0].scatter(tr['alpha'], tr[k], s=10, alpha=.35, color=col)
            ax[0].errorbar(al, summ[k + '_mean'], yerr=summ[k + '_sd'], color=col, ls=ls, marker='o', capsize=3,
                           label=lab)
        ax[0].set(xlabel='alpha', ylabel=ylabel, title=f'{pid}: {title}')
        ax[0].legend(fontsize=8)
        ax[1].scatter(tr['alpha'], tr[share_key], s=10, alpha=.35)
        ax[1].errorbar(al, summ[share_key + '_mean'], yerr=summ[share_key + '_sd'], marker='o', capsize=3,
                       label='measured')
        ax[1].plot([0, 1], [1, 0], 'k--', lw=1, label='1 - alpha (reference)')
        ax[1].set(xlabel='alpha', ylabel=share_label, ylim=(-0.05, 1.05), title=f'{pid}: {share_title}')
        ax[1].legend(fontsize=8)
        fig.tight_layout()
        fig.savefig(os.path.join(out, f'force_sharing_{pid}_{name}.png'), dpi=150)
        plt.close(fig)

    side_by_side([('posW_Heff_J', 'participant', 'tab:blue', '-'), ('posW_Reff_J', 'robot', 'tab:orange', '-')],
                 'share_posW_participant', 'positive work on the handle [J]', 'energy put into the movement',
                 "participant's share of positive work", 'who moved the handle (energy)', 'work')
    side_by_side([('push_Heff_N', 'participant (effective)', 'tab:blue', '-'), ('push_Reff_N', 'robot', 'tab:orange', '-'),
                  ('push_Fh_N', 'participant (hand force, before cancellation)', 'tab:blue', '--')],
                 'share_push_participant', 'mean force towards the target [N]', 'force put into the movement',
                 "participant's share of the pushing force", 'who pushed the handle (force)', 'force')

    fig, ax = plt.subplots(1, 2, figsize=(11, 4), sharey=True)
    grid, cmap = np.linspace(0, 100, 101), plt.get_cmap('viridis')
    for i, (alpha, grp) in enumerate(prof.groupby('alpha')):
        c = cmap(i / max(len(al) - 1, 1))
        ax[0].plot(grid, np.mean(np.vstack(grp['_prof_H']), axis=0), color=c, label=f'alpha {alpha:g}')
        ax[1].plot(grid, np.mean(np.vstack(grp['_prof_R']), axis=0), color=c, label=f'alpha {alpha:g}')
    ax[0].set(xlabel='% of window', ylabel='force towards the target [N]', title=f'{pid}: participant (effective)')
    ax[1].set(xlabel='% of window', title=f'{pid}: robot (alpha*PD + stiction)')
    for a_ in ax:
        a_.axhline(0, color='k', lw=.5)
    ax[1].legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(out, f'force_sharing_{pid}_profiles.png'), dpi=150)
    plt.close(fig)


def plot_saturation(plt, tr, summ, out, pid):
    """Left: fraction of the reach spent at each force limit; right: peak interaction force and the force limit."""
    al = summ.index.to_numpy()
    fig, ax = plt.subplots(1, 2, figsize=(11, 4))
    for k, lab, col in (('pd_sat_frac', 'robot PD at f_max (normal in transport)', 'tab:orange'),
                        ('cancel_sat_frac', 'cancellation capped (hand force leaks through)', 'tab:blue'),
                        ('cmd_sat_frac', 'total command clipped', 'tab:purple')):
        ax[0].scatter(tr['alpha'], tr[k], s=10, alpha=.35, color=col)
        ax[0].errorbar(al, summ[k + '_mean'], yerr=summ[k + '_sd'], color=col, marker='o', capsize=3, label=lab)
    ax[0].set(xlabel='alpha', ylabel='fraction of the reach', ylim=(-0.02, 1.02), title=f'{pid}: time at a force limit')
    ax[0].legend(fontsize=8)
    ax[1].scatter(tr['alpha'], tr['Fint_peak_N'], s=10, alpha=.35, color='tab:red')
    ax[1].errorbar(al, summ['Fint_peak_N_mean'], yerr=summ['Fint_peak_N_sd'], color='tab:red', marker='o', capsize=3,
                   label='peak interaction force')
    for lim in sorted(tr['reach_force_limit_N'].unique()):
        ax[1].axhline(lim, color='k', ls='--', lw=1, label=f'reach_force_limit {lim:g} N')
    for lim in sorted(tr['cancel_f_max_N'].unique()):
        ax[1].axhline(lim, color='tab:blue', ls=':', lw=1, label=f'cancel_f_max {lim:g} N')
    ax[1].set(xlabel='alpha', ylabel='[N]', title=f'{pid}: peak hand force per trial')
    ax[1].legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(out, f'force_sharing_{pid}_saturation.png'), dpi=150)
    plt.close(fig)


def plot_trace(plt, tc, out, pid, trial):
    """Forces of one trial over time, with the limits and the saturated samples shaded."""
    L = tc['limits']
    fig, ax = plt.subplots(3, 1, figsize=(10, 9), sharex=True)
    t = tc['t']
    ax[0].plot(t, tc['R'], color='tab:orange', label='robot term alpha*Fpd + stiction')
    ax[0].plot(t, tc['cancel'], color='tab:blue', label='cancellation applied')
    ax[0].plot(t, tc['cmd'], color='tab:purple', lw=1.8, label='Fcmd (sent to the robot)')
    ax[0].plot(t, tc['Fh'], color='tab:green', alpha=.8, label='hand force F_h')
    for lim, col, lab in ((L['cancel_f_max_N'], 'tab:blue', 'cancel_f_max'), (L['command_f_max_N'], 'tab:purple', 'command_f_max')):
        ax[0].axhline(lim, color=col, ls=':', lw=1)
        ax[0].axhline(-lim, color=col, ls=':', lw=1)
        ax[0].text(t[-1], lim, f' {lab}', va='center', fontsize=7, color=col)
    for mask, col in ((tc['pd_sat'], 'tab:orange'), (tc['cancel_sat'], 'tab:blue'), (tc['cmd_sat'], 'tab:purple')):
        ax[0].fill_between(t, 0, 1, where=mask, color=col, alpha=.12, transform=ax[0].get_xaxis_transform(), step='post')
    ax[0].set(ylabel='force towards the target [N]',
              title=f'{pid}, trial {trial}, alpha {tc["alpha"]:g}: shaded = PD (orange), cancellation (blue), '
                    f'command (purple) saturated')
    ax[0].legend(fontsize=7, loc='lower right')
    ax[1].plot(t, tc['fint'], color='tab:red', label='|interaction force|')
    ax[1].axhline(L['reach_force_limit_N'], color='k', ls='--', lw=1, label='reach_force_limit')
    ax[1].set(ylabel='[N]')
    ax[1].legend(fontsize=7)
    ax[2].plot(t, tc['err_mm'], color='k', label='distance to the target centre')
    ax[2].axhspan(-tc['hw_mm'], tc['hw_mm'], color='tab:green', alpha=.15, label='target')
    if np.isfinite(tc['final_entry_s']):
        for a_ in ax:
            a_.axvline(tc['final_entry_s'], color='0.5', lw=.8)
    ax[2].set(xlabel='time from target onset [s]', ylabel='[mm] (+ = short)')
    ax[2].legend(fontsize=7)
    for a_ in ax:
        a_.grid(alpha=.3)
    fig.tight_layout()
    fig.savefig(os.path.join(out, f'force_sharing_{pid}_trial{trial}_trace.png'), dpi=150)
    plt.close(fig)


def plot_group(plt, summ, out):
    """One line per participant: push force and positive work of each side, and the participant's share."""
    fig, ax = plt.subplots(2, 3, figsize=(15, 8))
    rows = (('push_Heff_N', 'push_Reff_N', 'share_push_participant', 'mean force towards the target [N]', 'force'),
            ('posW_Heff_J', 'posW_Reff_J', 'share_posW_participant', 'positive work [J]', 'energy'))
    for pid, sp in summ.groupby(level=0):
        a = sp.index.get_level_values(1).to_numpy()
        for r, (kh, kr, ks, _, _) in enumerate(rows):
            line, = ax[r, 0].plot(a, sp[kh + '_mean'], marker='o', label=pid)
            ax[r, 1].plot(a, sp[kr + '_mean'], marker='o', color=line.get_color(), label=pid)
            ax[r, 2].plot(a, sp[ks + '_mean'], marker='o', color=line.get_color(), label=pid)
    for r, (_, _, _, ylab, what) in enumerate(rows):
        ax[r, 0].set(xlabel='alpha', ylabel=ylab, title=f'Participant ({what})')
        ax[r, 1].set(xlabel='alpha', ylabel=ylab, title=f'Robot ({what})')
        ax[r, 2].plot([0, 1], [1, 0], 'k--', lw=1, label='1 - alpha')
        ax[r, 2].set(xlabel='alpha', ylabel="participant's share", ylim=(-0.05, 1.05), title=f"Participant's share ({what})")
    ax[0, 2].legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(out, 'force_sharing_ALL.png'), dpi=150)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    ap.add_argument('raw', nargs='*', help='raw log(s) M2FittsRobotHuman_<tag>_raw.csv, or folder(s) holding them '
                                           '(default: RAW_FILES at the top of this script)')
    ap.add_argument('--window', choices=['move', 'all'], default='move')
    ap.add_argument('--phase', choices=['block', 'warmup', 'all'], default='block')
    ap.add_argument('--out', default=None)
    ap.add_argument('--trace', type=int, nargs='+', default=[], metavar='TRIAL',
                    help='trial index(es) for which to plot forces, limits and saturation over time')
    args = ap.parse_args()
    paths = expand(args.raw or RAW_FILES)
    if not paths:
        ap.error('give the path of a raw log (or a folder), or list it in RAW_FILES at the top of the script')
    out = args.out or os.path.join(os.path.dirname(os.path.abspath(paths[0])), 'force_sharing')
    os.makedirs(out, exist_ok=True)

    rows = []
    for path in paths:
        print(f'raw log: {path}')
        p, df = read_params(path), read_raw(path)
        trials_path = re.sub(r'_raw\.csv$', '_trials.csv', path)
        meta = pd.read_csv(trials_path, skipinitialspace=True) if os.path.exists(trials_path) else None
        print(f'  trials: {os.path.basename(trials_path) if meta is not None else "none found - no A, W, ID, MT columns"}')
        pid = participant_of(path, meta)
        print(f'  participant: {pid}')
        for s, e in reach_segments(df):
            seg = df.iloc[s:e + 1]
            phase = 'warmup' if int(round(seg['phase'].median())) == 0 else 'block'
            if args.phase != 'all' and phase != args.phase:
                continue
            trial_index = int(round(seg['trial'].median()))
            r = analyse_trial(seg, p, args.window, keep_trace=trial_index in args.trace)
            if r is None:
                continue
            r.update({'participant': pid, 'file': os.path.basename(path), 'phase': phase, 'trial_index': trial_index})
            if meta is not None:
                m = meta[(meta['phase'] == phase) & (meta['trial_index'] == r['trial_index'])]
                if len(m):
                    for c in ('A_cm', 'W_cm', 'ID_bits', 'MT_s', 'success', 'reach_abort'):
                        if c in m:
                            r[c] = m.iloc[0][c]
            rows.append(r)
    if not rows:
        sys.exit('no reach trial found')
    missing_traces = sorted(set(args.trace) - {r['trial_index'] for r in rows})
    if missing_traces:
        print(f'--trace: trial(s) {missing_traces} not found in the selected phase ({args.phase})')

    trials = pd.DataFrame(rows)
    tot = trials['push_Heff_N'] + trials['push_Reff_N']
    trials['share_push_participant'] = np.where(tot > 0, trials['push_Heff_N'] / tot.where(tot > 0, 1.), np.nan)
    profiles = trials[['participant', 'alpha', '_prof_H', '_prof_R']]
    traces = trials[['participant', 'trial_index', '_trace']].dropna() if '_trace' in trials else None
    trials = trials.drop(columns=[c for c in ('_prof_H', '_prof_R', '_trace') if c in trials])
    front = [c for c in ('participant', 'file', 'phase', 'trial_index', 'alpha', 'A_cm', 'W_cm', 'ID_bits', 'MT_s',
                         'success') if c in trials]
    trials = trials[front + [c for c in trials if c not in front]]

    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        plt = None
        print('(matplotlib not installed: figures skipped)')

    pd.set_option('display.width', 160)
    written = []
    for pid, tr in trials.groupby('participant'):
        summ = summarise(tr, 'alpha')
        tr.to_csv(os.path.join(out, f'force_sharing_{pid}_trials.csv'), index=False)
        summ.to_csv(os.path.join(out, f'force_sharing_{pid}_by_alpha.csv'))
        print(f'\n{pid}, per alpha (means over trials; + = towards the target; window = {args.window}):')
        print(summ[['n_trials', 'push_Fh_N_mean', 'push_Heff_N_mean', 'push_Reff_N_mean', 'share_push_participant_mean',
                    'posW_Heff_J_mean', 'posW_Reff_J_mean', 'share_posW_participant_mean']].round(3).to_string())
        print(f'{pid}, force limits per alpha (fraction of the reach at the limit; peak |interaction force|):')
        print(summ[['n_trials', 'pd_sat_frac_mean', 'cancel_sat_frac_mean', 'cmd_sat_frac_mean',
                    'cancel_excess_peak_N_mean', 'Fint_peak_N_mean', 'over_force_limit_s_mean']].round(3).to_string())
        flagged = tr[(tr['cancel_sat_s'] > 0) | (tr['cmd_sat_s'] > 0) | (tr['over_force_limit_s'] > 0)]
        if len(flagged):
            cols = [c for c in ('trial_index', 'alpha', 'ID_bits', 'cancel_sat_s', 'cancel_excess_peak_N', 'cmd_sat_s',
                                'Fint_peak_N', 'over_force_limit_s', 'success', 'reach_abort') if c in flagged]
            print(f'{pid}: {len(flagged)} trial(s) with the cancellation or command capped, or above the force limit:')
            print(flagged[cols].round(3).to_string(index=False))
        else:
            print(f'{pid}: no trial with the cancellation or command capped, or above the force limit.')
        if plt is not None:
            plot_participant(plt, tr, summ, profiles[profiles['participant'] == pid], out, pid)
            plot_saturation(plt, tr, summ, out, pid)
            if traces is not None:
                for _, row in traces[traces['participant'] == pid].iterrows():
                    plot_trace(plt, row['_trace'], out, pid, row['trial_index'])
        written.append(pid)
    if trials['participant'].nunique() > 1:
        trials.to_csv(os.path.join(out, 'force_sharing_ALL_trials.csv'), index=False)
        summ_all = summarise(trials, ['participant', 'alpha'])
        summ_all.to_csv(os.path.join(out, 'force_sharing_ALL_by_alpha.csv'))
        if plt is not None:
            plot_group(plt, summ_all, out)
        written.append('ALL')
    if trials['decomposition_mismatch_frac'].max() > 0.05:
        print('\nNOTE: for some trials the logged command does not match the expected cancellation on more than 5% of '
              'the samples (column decomposition_mismatch_frac): check the parameters file (stiction, caps, sign).')
    print(f'\nWritten to {out}/ for: {", ".join(written)}')


if __name__ == '__main__':
    main()