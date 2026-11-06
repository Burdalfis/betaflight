import sys,pathlib
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[2]/'m1-bench-2026-11-06'))
from bench_io import *
import hashlib,sys
r=pathlib.Path(__file__).resolve().parent
m=MSP(); info={}
try:
 flags,sectors,total,used=struct.unpack('<BIII',m.request(70)); assert flags&3==3 and 0<used<total
 info.update(flags=flags,sectors=sectors,total=total,used=used)
 start,stop=0,used
 if len(sys.argv)>2:
  run=json.loads(pathlib.Path(sys.argv[2]).read_text());start=run['flash_before'][3];stop=run['flash_after'][3]
 assert 0<=start<stop<=used
 info.update(start_offset=start,end_offset=stop)
 prefix=sys.argv[1] if len(sys.argv)>1 else 'bench-servo'
 for name in [prefix+'.bbl',prefix+'-repeat.bbl']:
  digest=hashlib.sha256()
  with open(r/name,'wb') as f:
   address=start
   while address<stop:
    n=min(1984,stop-address)
    for attempt in range(3):
     try:
      data=m.request(71,struct.pack('<IHB',address,n,0),timeout=3,v2=True);break
     except TimeoutError:
      print('retry',address,attempt+1,flush=True);time.sleep(.1)
      if attempt==2:raise
    gotaddr,gotlen,compression=struct.unpack_from('<IHB',data)
    assert gotaddr==address and compression==0 and gotlen==len(data)-7 and 0<gotlen<=n,(address,n,len(data),gotaddr,gotlen,compression)
    chunk=data[7:]; f.write(chunk); digest.update(chunk); address+=gotlen
  info[name]=digest.hexdigest(); print(name,used,info[name],flush=True)
 assert info[prefix+'.bbl']==info[prefix+'-repeat.bbl']
 info['two_reads_identical']=True
 (r/(prefix+'-download.json')).write_text(json.dumps(info,indent=2)+'\n')
finally:m.close()
