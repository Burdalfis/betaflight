"""Declared offline electrical-response metrics; no angle or pulse-edge calibration."""
import argparse,csv,hashlib,json,pathlib
import numpy as np

HERE=pathlib.Path(__file__).resolve().parent

def first_run(mask,n=5):
    if len(mask)<n:return None
    hit=np.flatnonzero(np.convolve(np.asarray(mask,dtype=int),np.ones(n,dtype=int),'valid')==n)
    return int(hit[0]) if len(hit) else None

def raw_onset(t,y,base,delta,sigma):
    direction=np.sign(delta); span=abs(delta); threshold=max(3*sigma,.02*span,3.)
    post=t>=0;tt=t[post];distance=direction*(y[post]-base)
    above=distance>=threshold; starts=np.flatnonzero(above&~np.r_[False,above[:-1]])
    rejected=0
    for start in starts:
        if start+5>len(tt) or not np.all(above[start:start+5]):continue
        hit=first_run(distance[start:]>=.1*span)
        if hit is None:continue
        end10=start+hit; end=int(np.searchsorted(tt,tt[end10]+.010,side='right'))
        if end>len(tt) or tt[-1]-tt[end10]<.010:continue
        if np.any(distance[start:end]<threshold/2):rejected+=1;continue
        return float(tt[start]*1000),threshold,rejected
    return None,threshold,rejected

def raw_directional_events(t,y,base,delta,sigma):
    post=t>=0;tt=t[post];distance=np.sign(delta)*(y[post]-base);span=abs(delta)
    threshold=max(3*sigma,.02*span,3.)
    toward=first_run(distance>=threshold);wrong=first_run(distance<=-threshold)
    anyindex=min([i for i in (toward,wrong) if i is not None],default=None)
    t10=first_run(distance>=.1*span);cut=len(tt)-1 if t10 is None else t10
    xx=tt[:cut+1];dd=distance[:cut+1];negative=dd<=-threshold
    starts=np.flatnonzero(negative&~np.r_[False,negative[:-1]])
    ends=np.flatnonzero(negative&~np.r_[negative[1:],False])
    dt=float(np.median(np.diff(tt)));runs=[]
    for a,b in zip(starts,ends):
        runs.append({'start_ms':float(xx[a]*1000),'end_ms':float(xx[b]*1000),'duration_ms':float((xx[b]-xx[a]+dt)*1000),'samples':int(b-a+1),'peak_adc':float(np.max(-dd[a:b+1])),'sustained5samples':bool(b-a+1>=5)})
    rawpeak=float(max(0.,-dd.min())) if len(dd) else 0.
    sustained=0.
    if len(dd)>=5:sustained=float(max(0.,np.min(np.lib.stride_tricks.sliding_window_view(-dd,5),axis=1).max()))
    excess=np.maximum(-dd-threshold,0.)
    area=float(np.sum((excess[1:]+excess[:-1])*.5*np.diff(xx))) if len(xx)>1 else 0.
    return {'raw_first_any_onset_ms':None if anyindex is None else float(tt[anyindex]*1000),'raw_first_any_direction':None if anyindex is None else ('toward' if anyindex==toward else 'wrong-way'),'raw_first_toward_onset_ms':None if toward is None else float(tt[toward]*1000),'raw_first_wrongway_onset_ms':None if wrong is None else float(tt[wrong]*1000),'raw_direction_threshold_adc':threshold,'raw_wrongway_window_end_ms':float(tt[cut]*1000),'raw_wrongway_window_definition':'From logged write until first raw5-sample sustained10%target crossing; all shorter opposite excursions remain in runlist.','raw_wrongway_peak_adc':rawpeak,'raw_wrongway_peak_percent_step':rawpeak/span*100,'raw_wrongway_sustained5_peak_adc':sustained,'raw_wrongway_sustained5_peak_percent_step':sustained/span*100,'raw_wrongway_total_above_threshold_duration_ms':float(np.count_nonzero(negative)*dt*1000),'raw_wrongway_longest_run_ms':max([z['duration_ms'] for z in runs],default=0.),'raw_wrongway_threshold_excess_area_adc_s':area,'raw_wrongway_runs':runs}

def clean_impulses(y,sigma):
    med=np.median(np.lib.stride_tricks.sliding_window_view(np.pad(y,(3,3),mode='edge'),7),axis=1)
    outlier=abs(y-med)>max(6*sigma,6.)
    z=y.copy();z[outlier]=med[outlier]
    return z,outlier

def local_linear(t,y,n):
    tw=np.lib.stride_tricks.sliding_window_view(t,n);yw=np.lib.stride_tricks.sliding_window_view(y,n)
    mt=tw.mean(axis=1);my=yw.mean(axis=1);x=tw-mt[:,None]
    slope=np.sum(x*(yw-my[:,None]),axis=1)/np.sum(x*x,axis=1)
    center=t[n//2:len(t)-n//2]
    return center,my+slope*(center-mt),slope,tw[:,0],tw[:,-1]

def crossing(t,progress,fraction):
    idx=first_run(progress>=fraction)
    return None if idx is None else float(t[idx]*1000)

def settling(t,level,endpoint,band):
    outside=np.flatnonzero(abs(level-endpoint)>band)
    idx=int(outside[-1]+1) if len(outside) else 0
    if idx>=len(t) or t[-1]-t[idx]<.100:return None
    return float(t[idx]*1000)

def trial_metrics(t,y):
    pre=(t<0)&(t>=-.200); assert np.count_nonzero(pre)>=100
    base=float(np.median(y[pre]));sigma=float(1.4826*np.median(abs(y[pre]-base)))
    tail=t>=t[-1]-.300;end=float(np.median(y[tail]));delta=end-base;span=abs(delta)
    out={'baseline_adc':base,'baseline_robust_sigma_adc':sigma,'endpoint_adc':end,'endpoint_robust_sigma_adc':float(1.4826*np.median(abs(y[tail]-end))),'step_span_adc':span,'signed_step_adc':delta,'observed_hold_ms':float(t[-1]*1000),'credible_response':span>=max(10*sigma,10.)}
    if not out['credible_response']:
        out['invalid_reason']='Endpoint displacement is below predeclared response/noise threshold';return out
    onset,threshold,rejected=raw_onset(t,y,base,delta,sigma)
    out.update(onset_ms=onset,established_toward_onset_ms=onset,onset_threshold_adc=threshold,onset_threshold_percent_step=float(threshold/span*100),rejected_onset_candidates=rejected)
    out.update(raw_directional_events(t,y,base,delta,sigma))
    clean,impulse=clean_impulses(y,sigma);out['impulse_replacement_count']=int(impulse.sum());out['impulse_replacement_times_ms']=(t[impulse]*1000).tolist()
    direct=np.sign(delta);rawprogress=(y[t>=0]-base)/delta
    out['raw_single_peak_overshoot_percent']=float(max(0.,rawprogress.max()-1)*100)
    for n in (11,21):
        ts,level,slopes,support_start,support_end=local_linear(t,clean,n);post=ts>=0;ts=ts[post];level=level[post];slopes=slopes[post];support_start=support_start[post];support_end=support_end[post];progress=(level-base)/delta
        prefix='' if n==11 else 'sensitivity21_'
        for pct in (10,50,90):
            out[prefix+f'unrestricted_t{pct}_ms']=crossing(ts,progress,pct/100)
            accepted=ts>=((onset or 0)/1000-1e-9)
            out[prefix+f't{pct}_ms']=None if onset is None else crossing(ts[accepted],progress[accepted],pct/100)
        t10=out[prefix+'t10_ms'];t90=out[prefix+'t90_ms'];movement=None if t10 is None or t90 is None else t90-t10
        out[prefix+'movement10_90_ms']=movement
        out[prefix+'mean10_90_slew_adc_per_s']=None if movement is None or movement<=0 else .8*span/(movement/1000)
        out[prefix+'unrestricted_peak_absolute_derivative_adc_per_s']=float(np.max(abs(slopes)))
        center_only=ts>=((onset or 0)/1000-1e-9)
        out[prefix+'center_only_peak_derivative_adc_per_s']=float(np.max(direct*slopes[center_only]))
        eligible=support_start>=((onset or 0)/1000-1e-9)
        out[prefix+'first_eligible_derivative_center_ms']=float(ts[np.flatnonzero(eligible)[0]]*1000)
        indices=np.flatnonzero(eligible);idx=int(indices[np.argmax(direct*slopes[eligible])])
        out[prefix+'peak_derivative_adc_per_s']=float(direct*slopes[idx]);out[prefix+'peak_signed_derivative_adc_per_s']=float(slopes[idx]);out[prefix+'peak_derivative_at_ms']=float(ts[idx]*1000)
        out[prefix+'maximum_absolute_derivative_adc_per_s']=float(np.max(abs(slopes[eligible])))
        out[prefix+'peak_search_full_window_after_detected_onset']=onset is not None
        sustained=np.min(np.lib.stride_tricks.sliding_window_view(progress,5),axis=1)
        overshoot=max(0.,float(sustained.max()-1));out[prefix+'sustained_filtered_overshoot_percent']=100*overshoot
        out[prefix+'overshoot_exceeds_3sigma']=bool(overshoot*span>3*sigma)
        out[prefix+'usable_hold_ms']=float(ts[-1]*1000)
        for pct in (2,5):
            band=pct/100*span;out[prefix+f'settling{pct}_ms']=settling(ts,level,end,band)
            out[prefix+f'settling{pct}_right_censored']=out[prefix+f'settling{pct}_ms'] is None
            out[prefix+f'settling{pct}_band_adc']=band
            out[prefix+f'settling{pct}_band_below_3sigma']=bool(band<3*sigma)
            out[prefix+f'noise_adjusted_settling{pct}_ms']=settling(ts,level,end,max(band,3*sigma))
    a=out['peak_derivative_adc_per_s'];b=out['sensitivity21_peak_derivative_adc_per_s']
    out['peak_derivative_window_sensitivity_percent']=100*(b/a-1) if a else None
    out['centered_filter_lookahead_ms']=5*float(np.median(np.diff(t)))*1000
    return out

def load_csv(p):
    with pathlib.Path(p).open() as f:rows=list(csv.DictReader(f,skipinitialspace=True))
    names=list(rows[0]);a=np.array([[float(x[k]) for k in names] for x in rows]);return names,a

def extract_trials(csvpath,block,planned_steps):
    names,a=load_csv(csvpath);c=lambda name:a[:,names.index(name)]
    time=c('time (us)')/1e6;pwm=c('debug[1]');adc=c('debug[0]')
    # Characterization commands are large steps. Ignore small restored-PID tail jitter.
    changes=np.flatnonzero(abs(np.diff(pwm))>=100)+1
    if len(changes)<planned_steps:raise ValueError(f'{block}: only{len(changes)} largechanges, expected{planned_steps}')
    changes=changes[:planned_steps]
    trials=[]
    for ix,s in enumerate(changes):
        old=int(round(pwm[s-1]));new=int(round(pwm[s]));assert old in (1100,1420,1700) and new in (1100,1420,1700),(old,new)
        next_change=int(changes[ix+1]) if ix+1<len(changes) else len(time)
        # Last trial may include a restored-PID tail; use intended1.2s dwell only.
        # The event schedule holds at least1.2s. Earlier commands end at actual nextwrite.
        end=next_change if ix+1<len(changes) else min(next_change,int(np.searchsorted(time,time[s]+1.2,side='left')))
        start=int(np.searchsorted(time,time[s]-.2,side='left'));tt=time[start:end]-time[s];yy=adc[start:end]
        result=trial_metrics(tt,yy)
        if ix:
            previous=int(round(pwm[changes[ix-1]-1]));prior_delta=old-previous
            history='continuation' if np.sign(new-old)==np.sign(prior_delta) else 'reversal'
        else:history='unknown'
        trialblock=('low-pairs6' if ix<12 else 'high-pairs6') if block=='paired6plus6' else block
        result.update(block=trialblock,trial=ix+1,from_pwm_us=old,to_pwm_us=new,history=history,fc_command_timestamp_us=int(round(time[s]*1e6)),sample_count=len(yy),servo_native_mismatch_count=int(np.count_nonzero(pwm[start:end]!=c('servo[5]')[start:end])))
        trials.append(result)
    return trials,{'csv':str(pathlib.Path(csvpath).resolve()),'sha256':hashlib.sha256(pathlib.Path(csvpath).read_bytes()).hexdigest(),'block':block,'large_steps_available':len(np.flatnonzero(abs(np.diff(pwm))>=100)),'planned_steps_analyzed':planned_steps,'ignored_possible_restore_tail':True,'record_frame_count':len(time),'dt_us_min':float(np.diff(time).min()*1e6),'dt_us_max':float(np.diff(time).max()*1e6)}

def stats(values):
    values=np.array(values,dtype=float);n=len(values)
    if not n:return {'n':0}
    out={'n':n,'median':float(np.median(values)),'q25':float(np.percentile(values,25)),'q75':float(np.percentile(values,75)),'min':float(values.min()),'max':float(values.max())}
    if n>=5:
        rng=np.random.default_rng(20261106);boots=np.median(rng.choice(values,size=(10000,n),replace=True),axis=1);out['median_bootstrap95_ci']=[float(x) for x in np.percentile(boots,[2.5,97.5])]
    return out

def summarize(trials):
    endpoints={p:[x['endpoint_adc'] for x in trials if x['to_pwm_us']==p and x['credible_response']] for p in (1100,1700)}
    fullspan=abs(np.median(endpoints[1100])-np.median(endpoints[1700]))
    for x in trials:
        if x['credible_response']:
            for prefix in ('','sensitivity21_'):
                x[prefix+'peak_derivative_fullspan_per_s']=x[prefix+'peak_derivative_adc_per_s']/fullspan
            x['step_fraction_fullspan']=x['step_span_adc']/fullspan
            x['movement10_90_ms_per_fullspan']=x['movement10_90_ms']/x['step_fraction_fullspan'] if x['movement10_90_ms'] is not None else None
    keys=['raw_first_any_onset_ms','raw_first_toward_onset_ms','raw_first_wrongway_onset_ms','established_toward_onset_ms','raw_wrongway_peak_adc','raw_wrongway_peak_percent_step','raw_wrongway_sustained5_peak_percent_step','raw_wrongway_total_above_threshold_duration_ms','raw_wrongway_longest_run_ms','onset_ms','movement10_90_ms','t10_ms','t50_ms','t90_ms','peak_derivative_adc_per_s','peak_derivative_fullspan_per_s','peak_derivative_window_sensitivity_percent','raw_single_peak_overshoot_percent','sustained_filtered_overshoot_percent','settling2_ms','settling5_ms','noise_adjusted_settling2_ms','noise_adjusted_settling5_ms','movement10_90_ms_per_fullspan']
    groups=[]
    for group in sorted(set((x['from_pwm_us'],x['to_pwm_us'],x['history']) for x in trials)):
        rows=[x for x in trials if (x['from_pwm_us'],x['to_pwm_us'],x['history'])==group]; valid=[x for x in rows if x['credible_response']]
        out={'from_pwm_us':group[0],'to_pwm_us':group[1],'history':group[2],'n_trials':len(rows),'n_valid':len(valid),'blocks':sorted(set(x['block'] for x in rows)),'metrics':{}}
        for key in keys:out['metrics'][key]=stats([x[key] for x in valid if x.get(key) is not None])
        for pct in (2,5):out[f'settling{pct}_right_censored_count']=sum(x[f'settling{pct}_right_censored'] for x in valid)
        groups.append(out)
    blockgroups=[]
    for block in sorted(set(x['block'] for x in trials)):
        for g in groups:
            rr=[x for x in trials if x['block']==block and (x['from_pwm_us'],x['to_pwm_us'],x['history'])==(g['from_pwm_us'],g['to_pwm_us'],g['history'])]
            if not rr:continue
            bg={'block':block,'from_pwm_us':g['from_pwm_us'],'to_pwm_us':g['to_pwm_us'],'history':g['history'],'n_trials':len(rr),'metrics':{k:stats([x[k] for x in rr if x.get(k) is not None]) for k in keys}}
            blockgroups.append(bg)
    return {'per_block_groups':blockgroups,'methods_amendments':[{'file':x.name,'sha256':hashlib.sha256(x.read_bytes()).hexdigest()} for x in sorted(HERE.glob('methods-amendment*.json'))],'pipeline_sha256':hashlib.sha256(pathlib.Path(__file__).read_bytes()).hexdigest(),'pooled_observed_fullspan_adc':float(fullspan),'pooled_endpoint_adc':{str(p):float(np.median(v)) for p,v in endpoints.items()},'groups':groups,'trials':trials,'methods_sha256':hashlib.sha256((HERE/'methods.json').read_bytes()).hexdigest(),'metric_caveat':'Old onset_ms is established target-directed response, not first-any motion. New raw_first_any_onset_ms uses same-polarity5sample noise threshold in either direction; shorter raw excursions are retained. Initial wrong-way motion may be genuine, particularly with a free case. Median replacement/filtering is not proof of an electrical artifact. Counts/fullspan, not calibrated shaft angles or scope-edge timing.'}

def synthetic_tests():
    t=np.arange(-.2,1.2,.001); onset=.02;duration=.1
    ramp=np.clip((t-onset)/duration,0,1)*1000
    ramp[(t>=.001)&(t<.006)]=150 #5ms impulse must not create accepted early onset
    y=1500+ramp
    pos=trial_metrics(t,y);neg=trial_metrics(t,3000-y)
    pulse_results=[]
    for amplitude in (150,500,1000):
        z=1500+np.clip((t-onset)/duration,0,1)*1000
        z[(t>=.001)&(t<.006)]=1500+amplitude
        q=trial_metrics(t,z)
        assert abs(q['movement10_90_ms']-80)<2,(amplitude,q['movement10_90_ms'])
        assert .020<=q['onset_ms']/1000<=.024,(amplitude,q['onset_ms'])
        pulse_results.append({'pulse_percent_step':amplitude/10,'onset_ms':q['onset_ms'],'t10_ms':q['t10_ms'],'movement10_90_ms':q['movement10_90_ms'],'unrestricted_t10_ms':q['unrestricted_t10_ms']})
    assert .020<=pos['onset_ms']/1000<=.024,pos['onset_ms']
    assert abs(pos['movement10_90_ms']-80)<2,pos['movement10_90_ms']
    assert abs(pos['peak_derivative_adc_per_s']-10000)<1,pos['peak_derivative_adc_per_s']
    late=1500+np.clip((t-onset)/duration,0,1)*1000
    late[(t>=.015)&(t<.020)]=1000
    late_result=trial_metrics(t,late)
    assert abs(late_result['peak_derivative_adc_per_s']-10000)<1,late_result['peak_derivative_adc_per_s']
    assert abs(late_result['movement10_90_ms']-80)<2,late_result['movement10_90_ms']
    assert abs(pos['raw_first_any_onset_ms']-1)<.01,pos['raw_first_any_onset_ms']
    assert pos['raw_first_any_direction']=='toward'
    assert abs(late_result['raw_first_any_onset_ms']-15)<.01,late_result['raw_first_any_onset_ms']
    assert late_result['raw_first_any_direction']=='wrong-way'
    assert late_result['raw_wrongway_peak_adc']>=499
    assert pos['onset_ms']==neg['onset_ms']
    assert abs(pos['peak_derivative_adc_per_s']-neg['peak_derivative_adc_per_s'])<1e-6
    assert pos['sustained_filtered_overshoot_percent']<1e-6
    # A genuine sustained20% overshoot, followed by exact steady endpoint.
    ov=1500+np.where(t<.02,0,np.where(t<.12,(t-.02)/.1*1000,np.where(t<.14,1000+(t-.12)/.02*200,np.where(t<.18,1200,np.where(t<.3,1200-(t-.18)/.12*200,1000)))))
    result=trial_metrics(t,ov)
    assert 19<result['sustained_filtered_overshoot_percent']<21,result
    assert .275<result['settling2_ms']/1000<.31,result['settling2_ms']
    assert not trial_metrics(t,np.full(len(t),1500.))['credible_response']
    return {'passed':True,'near_onset_opposite_pulse_validation':{k:late_result[k] for k in ('onset_ms','peak_derivative_adc_per_s','center_only_peak_derivative_adc_per_s','movement10_90_ms')},'impulse_amplitude_validation':pulse_results,'tests':['known20ms onset plus3count/2%threshold','80ms10–90linear ramp','known10000ADCcounts/s slope','5ms synthetic pulse retained as firstANY but excluded from establishedtoward bycontinuity','positive/negative symmetry','no-overshoot ramp','sustained20%overshoot and2%settling','flat no-response rejection'],'linear_ramp_metrics':{k:pos[k] for k in ('onset_ms','movement10_90_ms','peak_derivative_adc_per_s','settling2_ms')},'overshoot_metrics':{k:result[k] for k in ('sustained_filtered_overshoot_percent','settling2_ms')}}

def main():
    p=argparse.ArgumentParser();p.add_argument('--dataset',action='append',nargs=3,metavar=('CSV','BLOCK','NSTEPS'));p.add_argument('--synthetic-only',action='store_true');p.add_argument('--output-dir');p.add_argument('--fixture',choices=['FREE_CASE','CLAMPED_CASE'],default='FREE_CASE');a=p.parse_args()
    output=pathlib.Path(a.output_dir) if a.output_dir else HERE;output.mkdir(parents=True,exist_ok=True)
    tests=synthetic_tests();(output/'synthetic-validation.json').write_text(json.dumps(tests,indent=2)+'\n')
    if a.synthetic_only:print(json.dumps(tests,indent=2));return
    trials=[];inputs=[]
    for path,block,nsteps in a.dataset or []:
        tt,info=extract_trials(path,block,int(nsteps));trials+=tt;inputs.append(info)
    result=summarize(trials);result['inputs']=inputs;result['synthetic_validation_passed']=True;result['fixture']=a.fixture;result['conditions']={'servo_supply_V':5,'linkage':'DISCONNECTED','servo_case':a.fixture,'motors':'STOPPED','interpretation':'Provisional fixture-dependent electrical wiper response; free-case data cannot identify fixed-body dynamics or gearlash.'}
    (output/'results.json').write_text(json.dumps(result,indent=2)+'\n')
    columns=sorted(set(k for x in trials for k,v in x.items() if not isinstance(v,(list,dict))))
    with (output/'per-trial.csv').open('w') as f:
        wr=csv.DictWriter(f,fieldnames=columns,extrasaction='ignore');wr.writeheader();wr.writerows(trials)
    print('Analyzed',len(trials),'trials in',len(result['groups']),'transition/history groups; fullspan',result['pooled_observed_fullspan_adc'])

if __name__=='__main__':main()
