#!/usr/bin/env python3
"""Plot the frozen paired regressions and held-out measured-angle errors."""
import argparse
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('profile',type=Path);p.add_argument('validation',type=Path)
p.add_argument('--output',required=True,type=Path)
a=p.parse_args()
profile=json.loads(a.profile.read_text()); validation=json.loads(a.validation.read_text())
reg=profile['regression']; countreg=profile['count_regression']; pairs=reg['paired_means']
u=np.array([q['command_us'] for q in pairs]); y=np.array([q['mean_angle_deg'] for q in pairs])
lookup={b:{r['command_us']:r for r in rows} for b,rows in profile['branches'].items()}
c=np.array([(lookup['increasing_pwm'][q]['corrected_counts']+lookup['decreasing_pwm'][q]['corrected_counts'])/2 for q in u])
fit=reg['free_intercept']; command_fit=fit['slope_deg_per_us']*(u-reg['physical_middle_us'])+fit['intercept_deg']
count_fit=countreg['slope_deg_per_count']*c+countreg['intercept_deg']
fig,axes=plt.subplots(2,2,figsize=(11,8),constrained_layout=True)
axes[0,0].errorbar(u,y,yerr=[q['half_branch_separation_deg'] for q in pairs],fmt='o',ms=3,lw=.8,label='Paired means ± half approach separation')
axes[0,0].plot(u,command_fit,color='black',lw=1,label='Least-squares line')
axes[0,0].set(xlabel='Command PWM (µs)',ylabel='Basket roll from flat (deg)',title=f"Command fit: r={reg['pearson_r']:.6f}, RMS={fit['rms']:.3f}°")
order=np.argsort(c)
axes[0,1].plot(c,y,'o',ms=3,label='Paired means')
axes[0,1].plot(c[order],count_fit[order],color='black',lw=1,label='Least-squares line')
axes[0,1].set(xlabel='Mean PB0−PC0 (ADC counts)',ylabel='Basket roll from flat (deg)',title=f"Count fit: r={countreg['pearson_r']:.6f}, RMS={countreg['rms']:.3f}°")
axes[1,0].plot(u,y-command_fit,'o-',ms=3,lw=.8,label='Command-line residual')
axes[1,0].plot(u,y-count_fit,'o-',ms=3,lw=.8,label='Count-line residual')
axes[1,0].set(xlabel='Paired command PWM (µs)',ylabel='Paired mean minus fitted angle (deg)',title='Training residuals; one equal-weight point per command')
good=[r for r in validation['points'] if r['qualified_for_scoring'] and r['in_common_count_domain']]
for context,color in [('continuation','tab:blue'),('direction_reversal','tab:orange'),('initial','black')]:
 rows=[r for r in good if r['context']==context]
 axes[1,1].scatter([r['corrected_counts'] for r in rows],[r['error_deg'] for r in rows],s=18,color=color,label=context.replace('_',' '))
axes[1,1].set(xlabel='Measured PB0−PC0 (ADC counts)',ylabel='Frozen mean map minus ADXL angle (deg)',title='Held-out reversals; failed stationary window excluded')
for ax in axes.flat:
 ax.axhline(0,color='gray',lw=.7,ls='--');ax.grid(alpha=.25);ax.legend(fontsize=7)
fig.suptitle('Measured basket calibration — operator-defined flat zero, motor absent\nRegression uses 29 means of exactly two approach samples; no held-out refitting',fontsize=11)
if a.output.exists():raise FileExistsError(a.output)
fig.savefig(a.output,dpi=155);plt.close(fig)
