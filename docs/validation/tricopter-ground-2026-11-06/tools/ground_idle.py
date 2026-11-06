import sys,pathlib,struct,time,json,statistics
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[2]/'m1-bench-2026-11-06'))
from bench_io import MSP
r=pathlib.Path(__file__).resolve().parent
m=MSP();out={}
try:
 assert m.request(1)==bytes((0,1,48))
 m.request(99,b'\x01\x00')
 out['servo_config_hex']=m.request(120).hex()
 out['motors']=struct.unpack('<8H',m.request(104));assert out['motors'][:3]==(1000,)*3
 out['servos']=struct.unpack('<8H',m.request(103))
 out['flash']=struct.unpack('<BIII',m.request(70))
 out['analog_hex']=m.request(110).hex()
 out['status_hex']=m.request(150).hex()
 samples=[];t0=time.monotonic()
 while time.monotonic()-t0<3:
  d=struct.unpack('<8h',m.request(254));samples.append(d)
  assert d[5]&7==7,d
  assert d[3]==d[0]-d[2],d
  assert 0<d[0]<4095 and 0<=d[2]<4095,d
 out['sample_count']=len(samples)
 out['debug_min']=[min(d[i] for d in samples) for i in range(8)]
 out['debug_max']=[max(d[i] for d in samples) for i in range(8)]
 out['debug_median']=[statistics.median(d[i] for d in samples) for i in range(8)]
 out['fresh_fraction']=sum(bool(d[5]&8) for d in samples)/len(samples)
 assert out['fresh_fraction']>.95,out
 assert out['flash'][2]-out['flash'][3]>1500000,out['flash']
 print(json.dumps(out,indent=2))
finally:
 (r/'idle-check.json').write_text(json.dumps(out,indent=2)+'\n');m.close()
