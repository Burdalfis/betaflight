import argparse,csv,hashlib,json,pathlib
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
R=pathlib.Path(__file__).resolve().parent

def load(path):
 with pathlib.Path(path).open() as f:rows=list(csv.DictReader(f,skipinitialspace=True))
 names=list(rows[0]);a=np.array([[float(x[k]) for k in names] for x in rows]);return names,a

def summarize(x):
 x=np.asarray(x,dtype=float)
 return {'n':len(x),'median':float(np.median(x)),'q25':float(np.percentile(x,25)),'q75':float(np.percentile(x,75)),'min':float(x.min()),'max':float(x.max())}

def analyze(path,output=None,fixture="FREE_CASE",raw_bbl=None,expected_sha=None):
 out=pathlib.Path(output) if output else R;out.mkdir(parents=True,exist_ok=True)
 names,a=load(path);c=lambda name:a[:,names.index(name)];t=c('time (us)')/1e6;pwm=c('debug[1]');adc=c('debug[0]')
 changes=np.flatnonzero(abs(np.diff(pwm))==2)+1;assert len(changes)>=120,len(changes);changes=changes[:120]
 expected=(list(range(1402,1441,2))+list(range(1438,1399,-2)))*3
 assert [int(pwm[s]) for s in changes]==expected,'Observed small-step schedule differs from planned sequence'
 allchanges=np.flatnonzero(np.diff(pwm))+1;rows=[]
 for ix,s in enumerate(changes):
  nxt=allchanges[allchanges>s];end=int(nxt[0]) if len(nxt) else len(t)
  hold=(np.arange(len(t))>=s)&(np.arange(len(t))<end);last=t[end-1]
  steady=hold&(t>=last-.100);late=hold&(t>=last-.050);early=hold&(t>=last-.100)&(t<last-.050)
  x=adc[steady];med=float(np.median(x));sigma=float(1.4826*np.median(abs(x-med)))
  row={'cycle':ix//40+1,'direction':'PWM-up' if (ix%40)<20 else 'PWM-down','pwm_us':int(pwm[s]),'from_pwm_us':int(pwm[s-1]),'fc_timestamp_us':int(round(t[s]*1e6)),'hold_duration_ms':float((t[end-1]-t[s])*1000),'last100ms_samples':int(len(x)),'steady_adc_median':med,'steady_adc_mean':float(x.mean()),'raw_adc_robust_sigma':sigma,'raw_adc_std':float(x.std(ddof=1)),'raw_adc_min':int(x.min()),'raw_adc_max':int(x.max()),'late50_minus_early50_adc':float(np.median(adc[late])-np.median(adc[early]))}
  rows.append(row)
 # Separate cycle/branch intercepts; only slope is shared.
 groups=sorted(set((x['cycle'],x['direction']) for x in rows));x=np.array([z['pwm_us']-1420 for z in rows],dtype=float);y=np.array([z['steady_adc_median'] for z in rows]);X=np.zeros((len(rows),len(groups)+1));X[:,0]=x
 for i,z in enumerate(rows):X[i,1+groups.index((z['cycle'],z['direction']))]=1
 fit=np.linalg.lstsq(X,y,rcond=None)[0];slope=float(fit[0]);assert abs(slope)>.05,'No usable static slope'
 branchfits=[]
 for cyc,direction in groups:
  rr=[z for z in rows if (z['cycle'],z['direction'])==(cyc,direction)];xx=np.array([z['pwm_us'] for z in rr]);yy=np.array([z['steady_adc_median'] for z in rr]);co=np.polyfit(xx-1420,yy,1)
  branchfits.append({'cycle':cyc,'direction':direction,'slope_adc_per_us':float(co[0]),'adc_at1420_intercept':float(co[1]),'fit_rms_residual_adc':float(np.sqrt(np.mean((np.polyval(co,xx-1420)-yy)**2)))})
 pairs=[];levels=list(range(1402,1439,2))
 for cyc in range(1,4):
  for p in levels:
   up=next(z for z in rows if z['cycle']==cyc and z['direction']=='PWM-up' and z['pwm_us']==p);down=next(z for z in rows if z['cycle']==cyc and z['direction']=='PWM-down' and z['pwm_us']==p);gap=down['steady_adc_median']-up['steady_adc_median']
   pairs.append({'cycle':cyc,'pwm_us':p,'up_adc':up['steady_adc_median'],'down_adc':down['steady_adc_median'],'descending_minus_ascending_adc':gap,'equivalent_input_us':gap/slope})
 perlevel=[]
 for p in levels:
  rr=[z for z in pairs if z['pwm_us']==p];perlevel.append({'pwm_us':p,'gap_adc':summarize([z['descending_minus_ascending_adc'] for z in rr]),'equivalent_input_us':summarize([z['equivalent_input_us'] for z in rr])})
 cycles=[]
 for cyc in range(1,4):
  rr=[z for z in pairs if z['cycle']==cyc];cycles.append({'cycle':cyc,'gap_adc':summarize([z['descending_minus_ascending_adc'] for z in rr]),'equivalent_input_us':summarize([z['equivalent_input_us'] for z in rr])})
 medgap=np.array([z['equivalent_input_us']['median'] for z in perlevel]);dt=np.diff(c('time (us)'))
 result={'methods_sha256':hashlib.sha256((R/'methods.json').read_bytes()).hexdigest(),'input_csv':str(pathlib.Path(path).resolve()),'input_csv_sha256':hashlib.sha256(pathlib.Path(path).read_bytes()).hexdigest(),'frames':len(a),'record_duration_s':float(t[-1]-t[0]),'timestamp_step_us':{'min':float(dt.min()),'median':float(np.median(dt)),'max':float(dt.max())},'native_servo_write_mismatches':int(np.count_nonzero(pwm!=c('servo[5]'))),'adc_status_flag_values':np.unique(c('debug[3]')).astype(int).tolist(),'rpm_max_erpm':[int(c(f'eRPM[{i}]').max()) for i in range(3)],'plateaus_analyzed':len(rows),'cycles':3,'command_increment_us':2,'common_pwm_levels':levels,'common_static_slope_adc_per_us':slope,'branch_fits':branchfits,'joint_fit_rms_residual_adc':float(np.sqrt(np.mean((X@fit-y)**2))),'per_cycle_gap_summary':cycles,'per_pwm_gap_summary':perlevel,'equivalent_input_summary':{'median_signed_across_pwm_medians_us':float(np.median(medgap)),'median_absolute_across_pwm_medians_us':float(np.median(abs(medgap))),'max_absolute_pwm_median_us':float(max(abs(medgap))),'pwm_median_range_us':[float(medgap.min()),float(medgap.max())]},'steady_noise_summary_adc':summarize([z['raw_adc_robust_sigma'] for z in rows]),'late_window_drift_summary_adc':summarize([z['late50_minus_early50_adc'] for z in rows]),'hold_duration_ms':summarize([z['hold_duration_ms'] for z in rows]),'limitations':['This is command/wiper directional hysteresis at one small range and unloaded5Vcondition, not uniquely mechanicalgearlash.','Direction also tracks measurement order/time within eachcycle; drift/thermal/supply effects are not independently separated.','Only3cycle pairs at eachPWM; median/IQR/range are descriptive, not a high-confidence physical backlashparameter.','Equivalentinput uses a local linear static fit; potentiometer-to-angle calibration and motor-side motion are absent.','No additional experiment was collected by the analysis agent.'],'plateaus':rows,'paired_gaps':pairs}
 result['fixture']=fixture;result['fixture_interpretation']='PROVISIONAL: unsecured/free servo case; this is fixture-dependent wiper hysteresis, not fixed-body dynamics or identified gearlash.' if fixture=='FREE_CASE' else 'Servo case clamped; linkage disconnected,5V. Wiper hysteresis still does not uniquely identify gearlash.'
 raw=pathlib.Path(raw_bbl) if raw_bbl else R.parent/'servo-dither.bbl'
 if raw.exists():
  result['raw_bbl_sha256']=hashlib.sha256(raw.read_bytes()).hexdigest();result['raw_bbl_bytes']=raw.stat().st_size
  if expected_sha: assert result['raw_bbl_sha256']==expected_sha
 result['decoder_clean_end']='Log clean end' in pathlib.Path(path).with_suffix('.event').read_text()
 result['pipeline_sha256']=hashlib.sha256(pathlib.Path(__file__).read_bytes()).hexdigest()
 (out/'results.json').write_text(json.dumps(result,indent=2)+'\n')
 for name,data in [('plateaus.csv',rows),('paired-gaps.csv',pairs)]:
  with (out/name).open('w') as f:w=csv.DictWriter(f,fieldnames=list(data[0]));w.writeheader();w.writerows(data)
 plt.rcParams.update({'font.size':10,'axes.spines.top':False,'axes.spines.right':False});fig,axs=plt.subplots(1,2,figsize=(12,4.7));colors={'PWM-up':'#175d9a','PWM-down':'#bf6b17'}
 for direction in ('PWM-up','PWM-down'):
  for cyc in range(1,4):
   rr=sorted([z for z in rows if z['cycle']==cyc and z['direction']==direction],key=lambda z:z['pwm_us']);axs[0].plot([z['pwm_us'] for z in rr],[z['steady_adc_median'] for z in rr],color=colors[direction],alpha=.25,lw=.8)
  levels2=sorted(set(z['pwm_us'] for z in rows if z['direction']==direction));vals=np.array([[z['steady_adc_median'] for z in rows if z['direction']==direction and z['pwm_us']==p] for p in levels2]);med=np.median(vals,axis=1);lo,hi=np.percentile(vals,[25,75],axis=1);axs[0].plot(levels2,med,'o-',color=colors[direction],ms=3,lw=1.5,label={'PWM-up':'Increasing PWM','PWM-down':'Decreasing PWM'}[direction]+' median');axs[0].fill_between(levels2,lo,hi,color=colors[direction],alpha=.15)
 axs[0].set(xlabel='Servo PWM write (µs)',ylabel='Median ADC count in final 100 ms',title='Small-step static input/wiper response');axs[0].legend(fontsize=8)
 for cyc in range(1,4):
  rr=[z for z in pairs if z['cycle']==cyc];axs[1].plot([z['pwm_us'] for z in rr],[z['equivalent_input_us'] for z in rr],lw=.8,alpha=.3,color='#278568')
 med=np.array([z['equivalent_input_us']['median'] for z in perlevel]);lo=np.array([z['equivalent_input_us']['q25'] for z in perlevel]);hi=np.array([z['equivalent_input_us']['q75'] for z in perlevel]);axs[1].plot(levels,med,'o-',color='#278568',ms=3,label='Paired cycle median');axs[1].fill_between(levels,lo,hi,color='#278568',alpha=.2,label='Cycle IQR');axs[1].axhline(0,color='#777',lw=.7,ls='--');axs[1].set(xlabel='Compared PWM (µs)',ylabel='(Descending−ascending ADC) / static slope (µs)',title='Observed directional gap in input units');axs[1].legend(fontsize=8)
 for ax in axs:ax.grid(alpha=.2)
 fig.suptitle(('FREE CASE — provisional' if fixture=='FREE_CASE' else 'CLAMPED CASE')+' — 1400–1440 µs staircase, 2 µs steps, 3 cycles',fontsize=13);fig.subplots_adjust(left=.08,right=.985,bottom=.21,top=.80,wspace=.34);fig.text(.012,.045,'Lines: individual cycles; shading: IQR. This measures command/wiper hysteresis, without uniquely identifying mechanical backlash.',fontsize=8,color='#555');fig.savefig(out/'servo-hysteresis.png',dpi=170,bbox_inches='tight')
 print(json.dumps({k:result[k] for k in ('frames','plateaus_analyzed','common_static_slope_adc_per_us','equivalent_input_summary','steady_noise_summary_adc','late_window_drift_summary_adc','hold_duration_ms')},indent=2))

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('csv');p.add_argument('--output-dir');p.add_argument('--fixture',choices=['FREE_CASE','CLAMPED_CASE'],default='FREE_CASE');p.add_argument('--raw-bbl');p.add_argument('--expected-sha');a=p.parse_args();analyze(a.csv,a.output_dir,a.fixture,a.raw_bbl,a.expected_sha)
