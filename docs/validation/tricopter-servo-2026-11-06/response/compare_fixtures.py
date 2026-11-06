import argparse,csv,json,pathlib
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from analysis import load_csv,stats
B=pathlib.Path(__file__).resolve().parent
p=argparse.ArgumentParser();p.add_argument('free_results');p.add_argument('clamped_results');p.add_argument('--output-dir',default=str(B/'fixture-comparison'));a=p.parse_args();out=pathlib.Path(a.output_dir);out.mkdir(parents=True,exist_ok=True)
data={'FREE_CASE':json.loads(pathlib.Path(a.free_results).read_text()),'CLAMPED_CASE':json.loads(pathlib.Path(a.clamped_results).read_text())}
selections=[('low-pairs6',1420,1100,'reversal'),('high-pairs6',1420,1700,'reversal'),('high-pairs6',1700,1420,'reversal')]
keys=['baseline_robust_sigma_adc','onset_threshold_adc','onset_threshold_percent_step','step_span_adc','raw_first_any_onset_ms','raw_first_toward_onset_ms','established_toward_onset_ms','raw_first_wrongway_onset_ms','raw_wrongway_peak_adc','raw_wrongway_peak_percent_step','raw_wrongway_sustained5_peak_percent_step','raw_wrongway_longest_run_ms','raw_wrongway_total_above_threshold_duration_ms','movement10_90_ms','t90_ms','peak_derivative_adc_per_s','sensitivity21_peak_derivative_adc_per_s','settling2_ms','settling5_ms']
rows=[];cache={}
for fixture,d in data.items():
 assert d['fixture']==fixture
 cache[fixture]={}
 for item in d['inputs']:
  names,arr=load_csv(item['csv']);cache[fixture][item['block']]=(names,arr)
 for sel in selections:
  trials=[z for z in d['trials'] if (z['block'],z['from_pwm_us'],z['to_pwm_us'],z['history'])==sel]
  result={'fixture':fixture,'block':sel[0],'from_pwm_us':sel[1],'to_pwm_us':sel[2],'history':sel[3],'n_trials':len(trials),'first_any_wrongway_trials':sum(z['raw_first_any_direction']=='wrong-way' for z in trials),'initial_sustained_wrongway_trials':sum(z['raw_first_wrongway_onset_ms'] is not None and z['raw_first_wrongway_onset_ms']<=z['raw_wrongway_window_end_ms'] for z in trials),'metrics':{}}
  for key in keys:result['metrics'][key]=stats([z[key] for z in trials if z.get(key) is not None])
  rows.append(result)
summary={'conditions':rows,'case_comparison_caveat':'Descriptive matched transition/history comparison, not proof that every early wiper excursion is electrical or mechanical. Raw signal is preserved without cleaning. FirstANY is noise-thresholded same-polarity5sample onset; shorter real motion may not meet it. Old onset is establishedtoward response. Thresholds depend on measuredbaselineMAD andspan and can differ betweenfixtures.','provenance':{'free_results':str(pathlib.Path(a.free_results).resolve()),'clamped_results':str(pathlib.Path(a.clamped_results).resolve())}}
(out/'comparison.json').write_text(json.dumps(summary,indent=2)+'\n')
flat=[]
for row in rows:
 z={k:row[k] for k in ('fixture','block','from_pwm_us','to_pwm_us','history','n_trials','first_any_wrongway_trials','initial_sustained_wrongway_trials')}
 for key,v in row['metrics'].items():
  for metric in ('median','q25','q75','min','max'):z[key+'_'+metric]=v.get(metric)
 flat.append(z)
with (out/'comparison.csv').open('w') as f:w=csv.DictWriter(f,fieldnames=list(flat[0]));w.writeheader();w.writerows(flat)
plt.rcParams.update({'font.size':10,'axes.spines.top':False,'axes.spines.right':False});fig,axs=plt.subplots(1,3,figsize=(14,4.8),sharey=True);colors={'FREE_CASE':'#bf6b17','CLAMPED_CASE':'#175d9a'};grid=np.arange(0,.0401,.001);curverows=[];extremes=[]
for ax,sel in zip(axs,selections):
 for fixture,d in data.items():
  trials=[z for z in d['trials'] if (z['block'],z['from_pwm_us'],z['to_pwm_us'],z['history'])==sel];values=[];thresholds=[]
  for trial in trials:
   source='paired6plus6' if trial['block'] in ('low-pairs6','high-pairs6') else trial['block'];names,arr=cache[fixture][source]
   tt=arr[:,names.index('time (us)')]/1e6-trial['fc_command_timestamp_us']/1e6;raw=arr[:,names.index('debug[0]')];mask=(tt>=-.003)&(tt<=.045)
   q=(raw[mask]-trial['baseline_adc'])/trial['signed_step_adc'];v=np.interp(grid,tt[mask],q);values.append(v);extremes.extend(v)
   ax.plot(grid*1000,v,color=colors[fixture],lw=.4,alpha=.13)
   thresholds.append(trial['onset_threshold_adc']/trial['step_span_adc'])
  values=np.array(values);med=np.median(values,axis=0);lo,hi=np.percentile(values,[25,75],axis=0)
  ax.plot(grid*1000,med,color=colors[fixture],lw=1.5,label=fixture.replace('_',' ').title()+f' (n={len(trials)})');ax.fill_between(grid*1000,lo,hi,color=colors[fixture],alpha=.15)
  threshold=float(np.median(thresholds));ax.axhline(threshold,color=colors[fixture],lw=.55,ls=':');ax.axhline(-threshold,color=colors[fixture],lw=.55,ls=':')
  for t,m,l,h in zip(grid,med,lo,hi):curverows.append({'fixture':fixture,'from_pwm_us':sel[1],'to_pwm_us':sel[2],'history':sel[3],'time_ms':t*1000,'raw_progress_median':m,'q25':l,'q75':h,'median_noise_threshold_progress':threshold})
 ax.axhline(0,color='#555',lw=.7);ax.set(title=f'{sel[1]}→{sel[2]} µs; reversal history',xlabel='Time since PWM register write (ms)',xlim=(0,40));ax.grid(alpha=.2);ax.legend(fontsize=8,loc='upper left')
axs[0].set_ylabel('Raw wiper change / measured trial span\nnegative = initially away from target')
for ax in axs:ax.set_ylim(min(extremes)-.04,max(extremes)+.04)
fig.suptitle('First 40 ms: raw wiper traces, free versus clamped servo case',fontsize=14);fig.subplots_adjust(left=.075,right=.985,top=.81,bottom=.24,wspace=.20)
fig.text(.013,.10,'Thin lines: individual trials; solid: median; shading: IQR. No deglitching, median replacement or derivative smoothing is applied to these curves.',fontsize=8,color='#555')
fig.text(.013,.05,'Dotted lines: median ±noise threshold for each fixture. Wiper voltage is not an independently calibrated shaft angle; brief excursions may be genuine motion.',fontsize=8,color='#555')
fig.savefig(out/'first40ms-raw-free-vs-clamped.png',dpi=165,bbox_inches='tight')
with (out/'first40ms-raw-curves.csv').open('w') as f:w=csv.DictWriter(f,fieldnames=list(curverows[0]));w.writeheader();w.writerows(curverows)
print(json.dumps(rows,indent=2))
