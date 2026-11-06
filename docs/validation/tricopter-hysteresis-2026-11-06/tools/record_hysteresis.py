import sys,pathlib,struct,time,json,csv,signal
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[2]/'m1-bench-2026-11-06'))
from bench_io import MSP
r=pathlib.Path(__file__).resolve().parent
plan=json.loads((r/'servo-hysteresis-plan.json').read_text())['plan']
m=MSP();orig=None;logging=False;events=[]
summary={'conditions':'Case clamped; arm free; linkage disconnected; props removed; servo5V; aircraft7V2A current-limited; all motors stopped','debug_mode':'TRI_GROUND','position_source':'signed PB0-PC0 debug[3] only','firmware_under_test':'a15ed3065','original_servo5':None,'sequence_complete':False}
signal.signal(signal.SIGTERM,lambda *a: (_ for _ in ()).throw(KeyboardInterrupt()))
try:
 assert m.request(1)==bytes((0,1,48));m.request(99,b'\x01\x00')
 arm_index=list(m.request(119)).index(0)
 orig=m.request(120);record=struct.unpack_from('<HHHbBI',orig,60)
 assert record==(1100,1700,1420,100,255,0),record
 summary['original_servo5']=record
 def position(target):
  assert 1100<=target<=1700
  m.request(212,struct.pack('<BHHHbBI',5,record[0],record[1],target,0,255,record[5]))
 def guard():
  status=m.request(150);assert not struct.unpack_from('<I',status,6)[0]&(1<<arm_index),'Armed'
  motors=struct.unpack('<8H',m.request(104));assert motors[:3]==(1000,)*3,motors
  v=struct.unpack('<BHHhH',m.request(110))[-1];assert v>=650,v
  flash=struct.unpack('<BIII',m.request(70));assert flash[2]-flash[3]>500000,flash
  return v
 guard();position(1420)
 summary['flash_before']=struct.unpack('<BIII',m.request(70));assert summary['flash_before'][2]-summary['flash_before'][3]>2200000
 m.request(0x3010,b'blackbox_mode = ALWAYS');logging=True;t0=time.monotonic();n=0;volts=[]
 with open(r/'servo-hysteresis-mspsamples.csv','w',newline='') as f:
  w=csv.writer(f);w.writerow(['host_s','segment','target_us']+[f'debug{i}' for i in range(8)])
  for index,entry in enumerate(plan):
   target=entry['target_us'];position(target)
   events.append(dict(entry,host_s=time.monotonic()-t0,segment=index))
   end=time.monotonic()+entry['duration_s'];counter=0
   while time.monotonic()<end:
    d=struct.unpack('<8h',m.request(254));assert d[1]==target and d[5]&7==7,(target,d)
    assert 0<d[0]<4095 and 0<=d[2]<4095 and d[3]==d[0]-d[2],d
    w.writerow([time.monotonic()-t0,index,target,*d]);n+=1;counter+=1
    if counter%10==0:volts.append(guard())
   print(index,entry['phase'],target,'position_counts',d[3],flush=True)
 summary['host_samples']=n;summary['min_reported_voltage_cV']=min(volts)
 summary['actual_host_duration_s']=time.monotonic()-t0
 summary['sequence_complete']=True
finally:
 try:
  if orig is not None:
   m.request(212,bytes((5,))+orig[60:72]);summary['servo_restored_exact']=m.request(120)==orig
  if logging:
   m.request(0x3010,b'blackbox_mode = MOTOR_TEST');time.sleep(1)
   m.request(0x3010,b'blackbox_mode = NORMAL');time.sleep(.1)
  summary['flash_after']=struct.unpack('<BIII',m.request(70))
  summary['final_motors']=struct.unpack('<8H',m.request(104))
  summary['final_debug']=struct.unpack('<8h',m.request(254))
 finally:
  (r/'servo-hysteresis-events.json').write_text(json.dumps(events,indent=2)+'\n')
  (r/'servo-hysteresis-run.json').write_text(json.dumps(summary,indent=2)+'\n')
  print(json.dumps(summary,indent=2),flush=True);m.close()
