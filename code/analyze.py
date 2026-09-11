#!/usr/bin/env python3
"""Generate figures and LaTeX tables from results CSVs. All numbers in the paper come from here."""
import pandas as pd, numpy as np, os, sys
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt

R = os.path.join(os.path.dirname(__file__), '..', 'results'); T = os.path.join(os.path.dirname(__file__), '..', 'tex')
os.makedirs(T, exist_ok=True)
cols = 'map,planner,agents,steps,seed,delay,thr,waits,stalls,recov,conf,delays,ms,tstd,alpha,gamma,stallw,window'.split(',')
def load(name):
    p = os.path.join(R, name)
    if not os.path.exists(p): return None
    d = pd.read_csv(p, names=cols); d['map'] = d['map'].str.replace('../maps/', '', regex=False).str.replace('.map', '', regex=False); return d

MAPS = {'warehouse_small': 'warehouse\\_small (33$\\times$57)', 'kiva': 'kiva (33$\\times$46)', 'sortation_small': 'sortation\\_small (33$\\times$57)', 'random-32-32-20': 'random-32-32-20'}
ORDER = ['PIBT', 'H-PIBT', 'WPP', 'CA-PIBT', 'CA-PIBT-noRecover', 'CA-PIBT-noHighway', 'H-PIBT+Recover']
COL = {'PIBT': '#7f7f7f', 'H-PIBT': '#1f77b4', 'WPP': '#2ca02c', 'CA-PIBT': '#d62728', 'CA-PIBT-noRecover': '#ff7f0e', 'CA-PIBT-noHighway': '#9467bd', 'H-PIBT+Recover': '#17becf'}
MK = {'PIBT': 'o', 'H-PIBT': 's', 'WPP': '^', 'CA-PIBT': 'D', 'CA-PIBT-noRecover': 'v', 'CA-PIBT-noHighway': 'x', 'H-PIBT+Recover': '+'}

def ci95(x):
    x = np.asarray(x, float); n = len(x)
    return 1.96 * x.std(ddof=1) / np.sqrt(n) if n > 1 else 0.0

macros = []
def M(name, val): macros.append(f'\\newcommand{{\\{name}}}{{{val}}}')

main = load('main.csv')
if main is not None:
    g = main.groupby(['map', 'agents', 'planner'])
    agg = g.agg(thr=('thr', 'mean'), thr_ci=('thr', ci95), waits=('waits', 'mean'), stalls=('stalls', 'mean'), ms=('ms', 'mean'), n=('thr', 'size')).reset_index()
    agg.to_csv(os.path.join(R, 'main_summary.csv'), index=False)
    # Figure 1: throughput vs agents, main planners
    fig, axes = plt.subplots(1, 4, figsize=(14, 3.2))
    for ax, (m, title) in zip(axes, MAPS.items()):
        for pl in ['PIBT', 'H-PIBT', 'WPP', 'CA-PIBT']:
            s = agg[(agg['map'] == m) & (agg.planner == pl)].sort_values('agents')
            if len(s) == 0: continue
            ax.errorbar(s.agents, s.thr, yerr=s.thr_ci, label=pl, color=COL[pl], marker=MK[pl], ms=4, lw=1.4, capsize=2)
        ax.set_title(title.replace('\\_', '_').replace('$\\times$', 'x'), fontsize=10); ax.set_xlabel('number of robots'); ax.grid(alpha=.3)
    axes[0].set_ylabel('throughput (tasks / step)'); axes[0].legend(fontsize=8, frameon=False)
    plt.tight_layout(); plt.savefig(os.path.join(T, 'fig_throughput.pdf')); plt.close()
    # Figure 2: ablation on warehouse_small & kiva
    fig, axes = plt.subplots(1, 2, figsize=(7.2, 3.0))
    for ax, m in zip(axes, ['warehouse_small', 'kiva']):
        for pl in ['H-PIBT', 'H-PIBT+Recover', 'CA-PIBT-noHighway', 'CA-PIBT-noRecover', 'CA-PIBT']:
            s = agg[(agg['map'] == m) & (agg.planner == pl)].sort_values('agents')
            if len(s) == 0: continue
            ax.errorbar(s.agents, s.thr, yerr=s.thr_ci, label=pl, color=COL[pl], marker=MK[pl], ms=4, lw=1.2, capsize=2)
        ax.set_title(m, fontsize=10); ax.set_xlabel('number of robots'); ax.grid(alpha=.3)
    axes[0].set_ylabel('throughput (tasks / step)'); axes[1].legend(fontsize=7, frameon=False)
    plt.tight_layout(); plt.savefig(os.path.join(T, 'fig_ablation.pdf')); plt.close()
    # Figure 3: waits and stalls at each density (warehouse_small)
    fig, axes = plt.subplots(1, 2, figsize=(7.2, 3.0))
    for pl in ['PIBT', 'H-PIBT', 'WPP', 'CA-PIBT']:
        s = agg[(agg['map'] == 'warehouse_small') & (agg.planner == pl)].sort_values('agents')
        if len(s) == 0: continue
        axes[0].plot(s.agents, s.waits, label=pl, color=COL[pl], marker=MK[pl], ms=4)
        axes[1].plot(s.agents, s.stalls, label=pl, color=COL[pl], marker=MK[pl], ms=4)
    axes[0].set_ylabel('wait fraction'); axes[1].set_ylabel('stall events (>=30 steps no progress)'); axes[1].set_yscale('symlog')
    for ax in axes: ax.set_xlabel('number of robots'); ax.grid(alpha=.3)
    axes[0].legend(fontsize=8, frameon=False); plt.tight_layout(); plt.savefig(os.path.join(T, 'fig_waits.pdf')); plt.close()
    # Table: main results at highest common density per map + runtime
    rows = []
    for m in MAPS:
        sub = agg[agg['map'] == m]
        if len(sub) == 0: continue
        # pick the largest fleet where all 4 main planners exist
        ok = [a for a in sorted(sub.agents.unique()) if all(((sub.agents == a) & (sub.planner == p)).any() for p in ['PIBT', 'H-PIBT', 'WPP', 'CA-PIBT'])]
        for a in ([ok[len(ok)//2], ok[-1]] if ok else []):
            cells = []
            for p in ['PIBT', 'H-PIBT', 'WPP', 'CA-PIBT']:
                r = sub[(sub.agents == a) & (sub.planner == p)].iloc[0]
                cells.append(f"{r.thr:.2f}$\\pm${r.thr_ci:.2f} / {r.ms:.2f}")
            rows.append(f"{MAPS[m]} & {a} & " + ' & '.join(cells) + ' \\\\')
    with open(os.path.join(T, 'tab_main.tex'), 'w') as f:
        f.write('\\begin{tabular}{llcccc}\\toprule\nMap & $n$ & PIBT & H-PIBT & WPP & CA-PIBT (ours) \\\\\\midrule\n' + '\n'.join(rows) + '\n\\bottomrule\\end{tabular}\n')
    # headline macros: gains at the densest common fleet on each map
    for m in MAPS:
        sub = agg[agg['map'] == m]
        if len(sub) == 0: continue
        ok = [a for a in sorted(sub.agents.unique()) if all(((sub.agents == a) & (sub.planner == p)).any() for p in ['PIBT', 'H-PIBT', 'CA-PIBT'])]
        if not ok: continue
        a = ok[-1]; key = m.replace('_', '').replace('-', '').replace('323220', 'map')
        r = {p: sub[(sub.agents == a) & (sub.planner == p)].iloc[0] for p in ['PIBT', 'H-PIBT', 'CA-PIBT'] + (['WPP'] if ((sub.agents == a) & (sub.planner == 'WPP')).any() else [])}
        M(f'{key}N', a); M(f'{key}GainPIBT', f"{100*(r['CA-PIBT'].thr/r['PIBT'].thr-1):.0f}"); M(f'{key}GainHPIBT', f"{100*(r['CA-PIBT'].thr/r['H-PIBT'].thr-1):.0f}")
        M(f'{key}ThrCA', f"{r['CA-PIBT'].thr:.2f}"); M(f'{key}ThrPIBT', f"{r['PIBT'].thr:.2f}"); M(f'{key}ThrHPIBT', f"{r['H-PIBT'].thr:.2f}"); M(f'{key}MsCA', f"{r['CA-PIBT'].ms:.1f}")
        if 'WPP' in r: M(f'{key}ThrWPP', f"{r['WPP'].thr:.2f}"); M(f'{key}MsWPP', f"{r['WPP'].ms:.0f}")
    # best-over-all-densities gain per map (max over fleet sizes of CA vs H-PIBT)
    best = []
    for m in MAPS:
        sub = agg[agg['map'] == m]
        pv = sub.pivot(index='agents', columns='planner', values='thr')
        if 'CA-PIBT' in pv and 'H-PIBT' in pv:
            gain = (pv['CA-PIBT'] / pv['H-PIBT'] - 1) * 100; best.append((m, gain.max(), gain.idxmax(), gain.min(), gain.idxmin()))
    with open(os.path.join(T, 'gains.txt'), 'w') as f:
        for b in best: f.write(f"{b[0]}: max +{b[1]:.1f}% at n={b[2]}, min {b[3]:+.1f}% at n={b[4]}\n")
    # ablation table at densest fleet, warehouse_small and kiva
    rows = []
    for m in ['warehouse_small', 'kiva']:
        sub = agg[agg['map'] == m]; a = sub.agents.max()
        for p in ['H-PIBT', 'H-PIBT+Recover', 'CA-PIBT-noHighway', 'CA-PIBT-noRecover', 'CA-PIBT']:
            s = sub[(sub.agents == a) & (sub.planner == p)]
            if len(s): r = s.iloc[0]; mm=m.replace('_','\\_'); rows.append(f"{mm} & {a} & {p} & {r.thr:.2f}$\\pm${r.thr_ci:.2f} & {r.waits:.3f} & {r.stalls:.1f} & {r.ms:.2f} \\\\")
    with open(os.path.join(T, 'tab_ablation.tex'), 'w') as f:
        f.write('\\begin{tabular}{llllccc}\\toprule\nMap & $n$ & Variant & Throughput & Wait frac. & Stalls & ms/step \\\\\\midrule\n' + '\n'.join(rows) + '\n\\bottomrule\\end{tabular}\n')
    for m in ['warehouse_small', 'kiva']:
        sub = agg[agg['map'] == m]; a = sub.agents.max(); key = m.replace('_','').replace('-','')
        for p, tag in [('H-PIBT','HP'),('H-PIBT+Recover','HPR'),('CA-PIBT-noHighway','CAnoHW'),('CA-PIBT-noRecover','CAnoRec'),('CA-PIBT','CA')]:
            s_ = sub[(sub.agents == a) & (sub.planner == p)]
            if len(s_): M(f'abl{key}{tag}', f"{s_.iloc[0].thr:.2f}")
    M('numSeeds', int(main.seed.nunique())); M('numSteps', int(main.steps.iloc[0])); M('numRunsMain', len(main))

delay = load('delay.csv')
if delay is not None:
    agg = delay.groupby(['map', 'agents', 'delay', 'planner']).agg(thr=('thr', 'mean'), thr_ci=('thr', ci95), conf=('conf', 'mean')).reset_index()
    # add delay 0 reference from main
    if main is not None:
        ref = main[main.planner.isin(['PIBT', 'H-PIBT', 'WPP', 'CA-PIBT']) & main.agents.isin([200, 300]) & main['map'].isin(['warehouse_small', 'kiva'])]
        ref = ref.groupby(['map', 'agents', 'delay', 'planner']).agg(thr=('thr', 'mean'), thr_ci=('thr', ci95), conf=('conf', 'mean')).reset_index()
        agg = pd.concat([ref, agg])
    agg.to_csv(os.path.join(R, 'delay_summary.csv'), index=False)
    fig, axes = plt.subplots(1, 2, figsize=(7.2, 3.0))
    for ax, m in zip(axes, ['warehouse_small', 'kiva']):
        for pl in ['PIBT', 'H-PIBT', 'WPP', 'CA-PIBT']:
            s = agg[(agg['map'] == m) & (agg.agents == 300) & (agg.planner == pl)].sort_values('delay')
            if len(s) == 0: continue
            ax.errorbar(s.delay, s.thr, yerr=s.thr_ci, label=pl, color=COL[pl], marker=MK[pl], ms=4, capsize=2)
        ax.set_title(f'{m}, n=300', fontsize=10); ax.set_xlabel('per-step delay probability $p$'); ax.grid(alpha=.3)
    axes[0].set_ylabel('throughput (tasks / step)'); axes[0].legend(fontsize=8, frameon=False)
    plt.tight_layout(); plt.savefig(os.path.join(T, 'fig_delay.pdf')); plt.close()
    rows = []
    for m in ['warehouse_small', 'kiva']:
        for pl in ['PIBT', 'H-PIBT', 'WPP', 'CA-PIBT']:
            cells = []
            for dl in [0.0, 0.05, 0.1, 0.2]:
                s = agg[(agg['map'] == m) & (agg.agents == 300) & (agg.planner == pl) & (np.isclose(agg.delay, dl))]
                cells.append(f"{s.iloc[0].thr:.2f}" if len(s) else '--')
            mm=m.replace('_','\\_'); rows.append(f"{mm} & {pl} & " + ' & '.join(cells) + ' \\\\')
    with open(os.path.join(T, 'tab_delay.tex'), 'w') as f:
        f.write('\\begin{tabular}{llcccc}\\toprule\nMap ($n$=300) & Planner & $p$=0 & $p$=0.05 & $p$=0.1 & $p$=0.2 \\\\\\midrule\n' + '\n'.join(rows) + '\n\\bottomrule\\end{tabular}\n')
    # retention macro: CA-PIBT throughput at p=0.2 relative to p=0 (warehouse_small)
    for m in ['warehouse_small', 'kiva']:
        for pl in ['PIBT', 'H-PIBT', 'WPP', 'CA-PIBT']:
            s0 = agg[(agg['map'] == m) & (agg.agents == 300) & (agg.planner == pl) & np.isclose(agg.delay, 0)]
            s2 = agg[(agg['map'] == m) & (agg.agents == 300) & (agg.planner == pl) & np.isclose(agg.delay, 0.2)]
            if len(s0) and len(s2): M(f"ret{m.replace('_','').replace('-','')}{pl.replace('-','')}", f"{100*s2.iloc[0].thr/s0.iloc[0].thr:.0f}")

large = load('large.csv')
if large is not None:
    agg = large.groupby(['agents', 'planner']).agg(thr=('thr', 'mean'), thr_ci=('thr', ci95), ms=('ms', 'mean'), stalls=('stalls', 'mean'), n=('thr', 'size')).reset_index()
    agg.to_csv(os.path.join(R, 'large_summary.csv'), index=False)
    fig, axes = plt.subplots(1, 2, figsize=(7.2, 3.0))
    for pl in ['PIBT', 'H-PIBT', 'CA-PIBT']:
        s = agg[agg.planner == pl].sort_values('agents')
        if len(s) == 0: continue
        axes[0].errorbar(s.agents, s.thr, yerr=s.thr_ci, label=pl, color=COL[pl], marker=MK[pl], ms=4, capsize=2)
        axes[1].plot(s.agents, s.ms, label=pl, color=COL[pl], marker=MK[pl], ms=4)
    axes[0].set_ylabel('throughput (tasks / step)'); axes[1].set_ylabel('planning time (ms / step)'); axes[1].set_yscale('log')
    for ax in axes: ax.set_xlabel('number of robots'); ax.grid(alpha=.3); ax.set_title('warehouse_large (140x500)', fontsize=10)
    axes[0].legend(fontsize=8, frameon=False); plt.tight_layout(); plt.savefig(os.path.join(T, 'fig_large.pdf')); plt.close()
    rows = []
    for a in sorted(agg.agents.unique()):
        cells = []
        for pl in ['PIBT', 'H-PIBT', 'CA-PIBT']:
            s = agg[(agg.agents == a) & (agg.planner == pl)]
            cells.append(f"{s.iloc[0].thr:.1f}$\\pm${s.iloc[0].thr_ci:.1f} / {s.iloc[0].ms:.0f}" if len(s) else '--')
        rows.append(f"{a} & " + ' & '.join(cells) + ' \\\\')
    with open(os.path.join(T, 'tab_large.tex'), 'w') as f:
        f.write('\\begin{tabular}{lccc}\\toprule\n$n$ & PIBT & H-PIBT & CA-PIBT (ours) \\\\\\midrule\n' + '\n'.join(rows) + '\n\\bottomrule\\end{tabular}\n')
    for a in sorted(agg.agents.unique()):
        s = agg[agg.agents == a].set_index('planner')
        if all(p in s.index for p in ['PIBT', 'H-PIBT', 'CA-PIBT']):
            W={1000:'OneK',2000:'TwoK',3000:'ThreeK',4000:'FourK'}[a]
            M(f'scale{W}GainPIBT', f"{100*(s.loc['CA-PIBT'].thr/s.loc['PIBT'].thr-1):.0f}"); M(f'scale{W}GainHPIBT', f"{100*(s.loc['CA-PIBT'].thr/s.loc['H-PIBT'].thr-1):.0f}"); M(f'scale{W}MsCA', f"{s.loc['CA-PIBT'].ms:.0f}"); M(f'scale{W}ThrCA', f"{s.loc['CA-PIBT'].thr:.1f}"); M(f'scale{W}MsHPIBT', f"{s.loc['H-PIBT'].ms:.0f}")
    M('scaleSeeds', int(large.seed.nunique()))

with open(os.path.join(T, 'macros.tex'), 'w') as f: f.write('\n'.join(macros) + '\n')
print('\n'.join(macros)); print(open(os.path.join(T, 'gains.txt')).read() if os.path.exists(os.path.join(T, 'gains.txt')) else '')
