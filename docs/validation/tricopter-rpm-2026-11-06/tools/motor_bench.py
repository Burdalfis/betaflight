from bench_io import *
import csv,signal
r=pathlib.Path(__file__).resolve().parent
m=MSP(); summary={'supply_limit_A':2,'supply_setting_V':7,'maximum_external_motor_command':1100,'motor_test_only':True}; events=[]; rows=[]; logging=False
signal.signal(signal.SIGTERM,lambda *a: (_ for _ in ()).throw(KeyboardInterrupt()))
def command_motor(index=None,value=1000):
 values=[1000]*3
 if index is not None: values[index]=value
 assert max(values)<=1100
 m.request(214,struct.pack('<3H',*values))
def telemetry():
 b=m.request(139); count=b[0]; assert count==3 and len(b)==40
 return [struct.unpack_from('<IHBHHH',b,1+13*i) for i in range(3)]
try:
 assert m.request(1)==bytes((0,1,48));m.request(99,b'\x01\x00')
 boxes=list(m.request(119)); arm_index=boxes.index(0)
 origservo=m.request(120); summary['original_servo5']=struct.unpack_from('<HHHbBI',origservo,60)
 assert summary['original_servo5'][:4]==(1100,1700,1420,100)
 command_motor()
 vbat=struct.unpack('<BHHhH',m.request(110))[-1]; assert vbat>=650,vbat
 summary['initial_voltage_cV']=vbat
 summary['flash_before']=struct.unpack('<BIII',m.request(70))
 m.request(0x3010,b'blackbox_mode = ALWAYS');logging=True
 t0=time.monotonic()
 plan=[(None,1000,3)]
 for i in range(3):plan += [(i,1050,1.5),(i,1075,1.5),(i,1100,1.5),(None,1000,1)]
 with open(r/'motor-mspsamples.csv','w',newline='') as f:
  w=csv.writer(f);w.writerow(['host_s','active_motor','command','voltage_cV','rpm0','rpm1','rpm2','invalid0','invalid1','invalid2']+[f'debug{i}' for i in range(8)])
  for index,value,duration in plan:
   command_motor(index,value)
   event={'host_s':time.monotonic()-t0,'motor':index,'external_command':value};events.append(event)
   end=time.monotonic()+duration; speeds=[]; volts=[]
   while time.monotonic()<end:
    status=m.request(150); assert not struct.unpack_from('<I',status,6)[0] & (1<<arm_index),'Aircraft armed'
    actual=struct.unpack('<8H',m.request(104));expected=tuple(value if k==index else 1000 for k in range(3)); assert all(abs(a-e)<=1 for a,e in zip(actual[:3],expected)),(actual,expected)
    t=telemetry(); vbat=struct.unpack('<BHHhH',m.request(110))[-1];assert vbat>=650,('Supply voltage dropped',vbat)
    d=struct.unpack('<8h',m.request(254)); assert 1100<=d[1]<=1700,('Servo beyond existing limits',d)
    if index is not None: assert 0<=d[4+index]<5000,('RPM telemetry stale',index,d)
    row=[time.monotonic()-t0,-1 if index is None else index,value,vbat,*[x[0] for x in t],*[x[1] for x in t],*d];rows.append(row);w.writerow(row)
    speeds.append([x[0] for x in t]);volts.append(vbat)
   if index is not None:assert max(x[index] for x in speeds)>0,('No measured RPM',index,value)
   print(event,'RPM last',speeds[-1],'Vmin',min(volts)/100,flush=True)
 summary['samples']=len(rows);summary['minimum_voltage_cV']=min(x[3] for x in rows)
 summary['maximum_mechanical_rpm']=[max(x[4+i] for x in rows) for i in range(3)]
 summary['servo_config_unchanged']=m.request(120)==origservo
finally:
 try:
  command_motor()
  if logging:
   m.request(0x3010,b'blackbox_mode = MOTOR_TEST');time.sleep(6)
   m.request(0x3010,b'blackbox_mode = NORMAL');time.sleep(.1)
  summary['final_motors']=struct.unpack('<8H',m.request(104))
  summary['final_rpm']=[x[0] for x in telemetry()]
  summary['flash_after']=struct.unpack('<BIII',m.request(70))
  summary['final_debug']=struct.unpack('<8h',m.request(254))
 finally:
  (r/'motor-events.json').write_text(json.dumps(events,indent=2)+'\n')
  (r/'motor-bench-summary.json').write_text(json.dumps(summary,indent=2)+'\n')
  print(json.dumps(summary,indent=2),flush=True);m.close()
