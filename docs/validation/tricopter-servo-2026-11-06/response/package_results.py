import argparse,csv,hashlib,json,pathlib
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from analysis import load_csv,clean_impulses,local_linear
base=pathlib.Path(__file__).resolve().parent;bench=base.parent
p=argparse.ArgumentParser();p.add_argument('--output-dir');p.add_argument('--raw-input',action='append',nargs=2);args=p.parse_args();r=pathlib.Path(args.output_dir) if args.output_dir else base;d=json.loads((r/'results.json').read_text());fixture=d.get('fixture','FREE_CASE')
raws=[]
for name,sha in (args.raw_input or [('servo-characterization.bbl','23389c9e594ca370390787c77e75c87adb7e833c07dcfcf868d685d578345660'),('servo-paired.bbl','759d65b2d467775e32c84e98b4f60ee4e940cb6ab13b132017d9afcace3eea4c')]):
 p=pathlib.Path(name);p=p if p.is_absolute() else bench/p;actual=hashlib.sha256(p.read_bytes()).hexdigest();assert actual==sha
 raws.append({'file':str(p),'bytes':p.stat().st_size,'sha256':actual})
d['raw_inputs']=raws
d['data_integrity']={'frames_by_input':[x['record_frame_count'] for x in d['inputs']],'both_clean_end':True,'reported_corrupt_frames':0,'all_steps_credible':all(x['credible_response'] for x in d['trials']),'all_native_servo_write_match':all(x['servo_native_mismatch_count']==0 for x in d['trials']),'physical_angle_calibrated':False,'scope_edge_timing_measured':False}
d['overshoot_noise_review']={'trials_with_filtered_sustained_excursion_over_3raw_baseline_sigma':sum(x['overshoot_exceeds_3sigma'] for x in d['trials']),'interpretation':'Small filtered peak-selection excursions are not resolved physical overshoot under the declared3sigma raw-noise criterion. Raw excursions and impulse replacements remain available.'}
d['settling_noise_review']={str(pct):{'trials_strict_band_below_3raw_sigma':sum(x[f'settling{pct}_band_below_3sigma'] for x in d['trials']),'right_censored_trials':sum(x[f'settling{pct}_right_censored'] for x in d['trials'])} for pct in (2,5)}
d['derivative_window_sensitivity_review']={'percent_change_21sample_vs11sample':{'min':min(x['peak_derivative_window_sensitivity_percent'] for x in d['trials']),'median':float(np.median([x['peak_derivative_window_sensitivity_percent'] for x in d['trials']])),'max':max(x['peak_derivative_window_sensitivity_percent'] for x in d['trials'])},'interpretation':'Window-dependent electrical slope estimate within movement support; not a calibrated instantaneous physical velocity.'}
(r/'results.json').write_text(json.dumps(d,indent=2)+'\n')
metrics=['raw_first_any_onset_ms','raw_first_toward_onset_ms','raw_first_wrongway_onset_ms','established_toward_onset_ms','raw_wrongway_peak_adc','raw_wrongway_peak_percent_step','raw_wrongway_total_above_threshold_duration_ms','onset_ms','t10_ms','t50_ms','t90_ms','movement10_90_ms','peak_derivative_adc_per_s','peak_derivative_fullspan_per_s','peak_derivative_window_sensitivity_percent','sustained_filtered_overshoot_percent','settling2_ms','settling5_ms']
rows=[]
for g in d['per_block_groups']:
 row={k:g[k] for k in ('block','from_pwm_us','to_pwm_us','history','n_trials')}
 for metric in metrics:
  for stat in ('median','q25','q75','min','max'):row[metric+'_'+stat]=g['metrics'][metric].get(stat)
 rows.append(row)
with (r/'per-block-summary.csv').open('w') as f:
 w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)

# Resample electrical response trajectories only for median/IQR display.
cache={}
for info in d['inputs']:
 names,a=load_csv(info['csv']);cache[info['block']]=(names,a)
grid=np.arange(0,.251,.001)
def groupcurves(block,fromp,top,history):
 trials=[x for x in d['trials'] if x['block']==block and x['from_pwm_us']==fromp and x['to_pwm_us']==top and x['history']==history]
 curves=[]
 for trial in trials:
  source='paired6plus6' if block in ('low-pairs6','high-pairs6') else block
  names,a=cache[source];t=a[:,names.index('time (us)')]/1e6-trial['fc_command_timestamp_us']/1e6
  window=(t>=-.200)&(t<=.35);tt=t[window];yy=a[window,names.index('debug[0]')]
  clean,_=clean_impulses(yy,trial['baseline_robust_sigma_adc']);ts,level,_,_,_=local_linear(tt,clean,11)
  q=(level-trial['baseline_adc'])/trial['signed_step_adc'];curves.append(np.interp(grid,ts,q))
 vals=np.array(curves)
 return trials,np.median(vals,axis=0),np.percentile(vals,25,axis=0),np.percentile(vals,75,axis=0)
plt.rcParams.update({'font.size':10,'axes.spines.top':False,'axes.spines.right':False})
fig,axs=plt.subplots(2,2,figsize=(12,8.5));colors=['#175d9a','#bf6b17','#278568']
selected=[('cyclic12',1420,1100,'continuation'),('cyclic12',1420,1700,'continuation'),('cyclic12',1700,1420,'reversal')]
curve_csv=[]
for group,color in zip(selected,colors):
 trials,med,lo,hi=groupcurves(*group);label=f'{group[1]}→{group[2]} µs ({group[3]}, n={len(trials)})'
 axs[0,0].plot(grid*1000,med,color=color,label=label,lw=1.3);axs[0,0].fill_between(grid*1000,lo,hi,color=color,alpha=.15)
 for tt,m,l,h in zip(grid,med,lo,hi):curve_csv.append({'block':group[0],'from_pwm':group[1],'to_pwm':group[2],'history':group[3],'time_ms':tt*1000,'median_progress':m,'q25_progress':l,'q75_progress':h})
axs[0,0].set(title='Requested transitions: smoothed electrical response',xlabel='Time since PWM write (ms)',ylabel='Fraction of each measured ADC step',xlim=(0,210),ylim=(-.04,1.06));axs[0,0].legend(fontsize=7,loc='lower right')
for group,color in [(('high-pairs6',1420,1700,'reversal'),colors[1]),(('high-pairs6',1700,1420,'reversal'),colors[2])]:
 trials,med,lo,hi=groupcurves(*group);axs[0,1].plot(grid*1000,med,color=color,label=f'{group[1]}→{group[2]} µs (reversal, n={len(trials)})',lw=1.3);axs[0,1].fill_between(grid*1000,lo,hi,color=color,alpha=.15)
axs[0,1].set(title='Matched 280 µs steps, same reversal history',xlabel='Time since PWM write (ms)',ylabel='Fraction of each measured ADC step',xlim=(0,210),ylim=(-.04,1.06));axs[0,1].legend(fontsize=7,loc='lower right')
for ax in axs[0]:
 for y in (.1,.9):ax.axhline(y,color='#777',lw=.5,ls=':')
# Sampling-window sensitivity rather than implying exact physical peak speed.
for idx,group in enumerate(selected):
 tt=[x for x in d['trials'] if (x['block'],x['from_pwm_us'],x['to_pwm_us'],x['history'])==group]
 for shift,key,color,label in [(-.13,'peak_derivative_adc_per_s','#175d9a','11 samples'),(.13,'sensitivity21_peak_derivative_adc_per_s','#829db3','21 samples')]:
  vals=np.array([x[key] for x in tt]);med=np.median(vals);q=np.percentile(vals,[25,75]);axs[1,0].errorbar(idx+shift,med/1000,yerr=[[float(med-q[0])/1000],[float(q[1]-med)/1000]],fmt='o',color=color,capsize=4,label=label if idx==0 else None)
axs[1,0].set(xticks=range(3),xticklabels=['1420→1100','1420→1700','1700→1420'],ylabel='Peak electrical slope (1000 ADC counts/s)',title='Peak estimate changes with smoothing window',ylim=(0,None));axs[1,0].legend(fontsize=8)
# Compact timing table for same-history paired comparison.
axs[1,1].axis('off');table=[]
for spec in [('high-pairs6',1420,1700,'reversal'),('high-pairs6',1700,1420,'reversal'),('low-pairs6',1420,1100,'reversal')]:
 g=next(x for x in d['per_block_groups'] if (x['block'],x['from_pwm_us'],x['to_pwm_us'],x['history'])==spec)
 table.append([f'{spec[1]}→{spec[2]}',str(g['n_trials'])]+[f"{g['metrics'][k]['median']:.1f}" for k in ('raw_first_any_onset_ms','established_toward_onset_ms','movement10_90_ms','t90_ms','settling5_ms')])
tbl=axs[1,1].table(cellText=table,colLabels=['PWM (µs)','n','ANY','Toward*','10–90','t90','5% settle'],cellLoc='center',loc='center',colWidths=[.22,.06,.12,.16,.16,.12,.16]);tbl.auto_set_font_size(False);tbl.set_fontsize(8);tbl.scale(1,1.9)
axs[1,1].set_title('Paired-block medians (ms); all reversal histories',pad=15)
for ax in (axs[0,0],axs[0,1],axs[1,0]):ax.grid(alpha=.2)
fig.subplots_adjust(left=.075,right=.98,top=.92,bottom=.115,wspace=.30,hspace=.43)
fig.suptitle(('FREE CASE — provisional electrical response' if fixture=='FREE_CASE' else 'CLAMPED CASE — linkage disconnected, 5 V')+' — 72 step trials',fontsize=14)
fig.text(.015,.04,'ANY: sustained wiper excursion of either sign. Toward*: established target-directed response. Neither is physical deadtime or scope timing.',fontsize=8,color='#555')
fig.text(.015,.018,'No angle calibration. Shading: IQR. Filtering can attenuate genuine short motion; raw early excursions are preserved and analyzed separately.',fontsize=8,color='#555')
fig.savefig(r/'servo-characterization.png',dpi=170,bbox_inches='tight')
with (r/'median-response-curves.csv').open('w') as f:
 w=csv.DictWriter(f,fieldnames=list(curve_csv[0]));w.writeheader();w.writerows(curve_csv)
print(json.dumps({'integrity':d['data_integrity'],'overshoot':d['overshoot_noise_review'],'settling':d['settling_noise_review'],'sensitivity':d['derivative_window_sensitivity_review']},indent=2))
