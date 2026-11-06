import sys,pathlib,struct,time,json,csv,signal
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[2]/'m1-bench-2026-11-06'))
from bench_io import MSP
r=pathlib.Path(__file__).resolve().parent
m=MSP();orig=None;logging=False;events=[]
summary={'conditions':'Servo case clamped, arm free, linkage disconnected, props removed; servo 5V; aircraft 7V 2A current-limited; motors stopped','cycles':3,'hold_seconds':1.2,'debug_mode':'TRI_GROUND'}
signal.signal(signal.SIGTERM,lambda *a: (_ for _ in ()).throw(KeyboardInterrupt()))
try:
 assert m.request(1)==bytes((0,1,48));m.request(99,b'\x01\x00')
 arm_index=list(m.request(119)).index(0)
 orig=m.request(120);record=struct.unpack_from('<HHHbBI',orig,60)
 assert record==(1100,1700,1420,100,255,0),record
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
 summary['flash_before']=struct.unpack('<BIII',m.request(70));summary['original_servo5']=record
 m.request(0x3010,b'blackbox_mode = ALWAYS');logging=True;t0=time.monotonic();n=0;volts=[]
 plan=[(-1,1420,3.)]+[(cycle,target,1.2) for cycle in range(3) for target in (1100,1420,1700,1420)]
 with open(r/'servo-ground-mspsamples.csv','w',newline='') as f:
  w=csv.writer(f);w.writerow(['host_s','cycle','target_us']+[f'debug{i}' for i in range(8)])
  for cycle,target,duration in plan:
   position(target);events.append({'host_s':time.monotonic()-t0,'cycle':cycle,'target_us':target})
   end=time.monotonic()+duration;counter=0
   while time.monotonic()<end:
    d=struct.unpack('<8h',m.request(254))
    assert d[1]==target and d[5]&7==7,(target,d)
    assert 0<d[0]<4095 and 0<=d[2]<4095 and d[3]==d[0]-d[2],d
    w.writerow([time.monotonic()-t0,cycle,target,*d]);n+=1;counter+=1
    if counter%10==0:volts.append(guard())
   print('cycle',cycle,'target',target,'PB0,PC0,diff',d[0],d[2],d[3],flush=True)
 summary['host_samples']=n;summary['min_reported_voltage_cV']=min(volts)
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
  (r/'servo-ground-events.json').write_text(json.dumps(events,indent=2)+'\n')
  (r/'servo-ground-run.json').write_text(json.dumps(summary,indent=2)+'\n')
  print(json.dumps(summary,indent=2),flush=True);m.close()
