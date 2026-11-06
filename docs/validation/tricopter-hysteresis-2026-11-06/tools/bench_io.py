import argparse, pathlib, serial, struct, time, functools, operator, json

class MSP:
    def __init__(self, port='/dev/ttyACM0'):
        self.s = serial.Serial(port, 115200, timeout=0.002, exclusive=True)
    def close(self): self.s.close()
    @staticmethod
    def crc8(data):
        crc=0
        for x in data:
            crc ^= x
            for _ in range(8): crc=((crc<<1)^0xd5)&255 if crc&128 else (crc<<1)&255
        return crc
    def request(self, cmd, payload=b'', timeout=2, v2=False):
        self.s.reset_input_buffer()
        if v2 or cmd>255:
            body=struct.pack('<BHH',0,cmd,len(payload))+payload
            packet=b'$X<'+body+bytes((self.crc8(body),))
        else:
            body=bytes((len(payload),cmd))+payload
            packet=b'$M<'+body+bytes((functools.reduce(operator.xor,body,0),))
        self.s.write(packet); self.s.flush()
        buf=bytearray(); end=time.monotonic()+timeout
        while time.monotonic()<end:
            buf.extend(self.s.read(max(1,self.s.in_waiting)))
            while len(buf)>=6:
                pos=buf.find(b'$')
                if pos<0: buf.clear(); break
                if pos: del buf[:pos]
                if len(buf)<6: break
                if buf[1]==ord('M'):
                    n=buf[3]; responseCmd=buf[4]; total=n+6; start=5
                    if len(buf)<total: break
                    packet=bytes(buf[:total]); valid=functools.reduce(operator.xor,packet[3:],0)==0
                elif buf[1]==ord('X'):
                    if len(buf)<9: break
                    _,responseCmd,n=struct.unpack_from('<BHH',buf,3)
                    total=n+9; start=8
                    if len(buf)<total: break
                    packet=bytes(buf[:total]); valid=self.crc8(packet[3:-1])==packet[-1]
                else:
                    del buf[0]; continue
                del buf[:total]
                if not valid or responseCmd!=cmd: continue
                if packet[2]==ord('!'): raise RuntimeError(f'MSP {cmd} rejected')
                return packet[start:-1]
        raise TimeoutError(f'MSP {cmd}, incomplete response {len(buf)} bytes')

def quiet_read(s, seconds=15):
    out=bytearray(); end=time.monotonic()+seconds; last=time.monotonic()
    while time.monotonic()<end:
        try: b=s.read(8192)
        except serial.SerialException: break
        if b: out.extend(b); last=time.monotonic()
        elif time.monotonic()-last>.3: break
    return bytes(out)

def cli(outpath, commands):
    s=serial.Serial('/dev/ttyACM0',115200,timeout=.05,exclusive=True)
    s.write(b'\rversion\r'); s.flush(); initial=quiet_read(s)
    if b'Betaflight /' not in initial:
        s.write(b'#'); s.flush(); initial+=quiet_read(s)
        s.write(b'\rversion\r'); s.flush(); initial+=quiet_read(s)
    if b'Betaflight /' not in initial: raise RuntimeError(f'No CLI response: {initial!r}')
    with open(outpath,'wb') as f:
        f.write(initial)
        for command in commands:
            s.write(command.encode()+b'\r'); s.flush()
            response=quiet_read(s)
            f.write(response); f.flush()
            print(command, '=>',len(response),'bytes',flush=True)
            if any(token in response for token in (b'###ERROR',b'Unknown command',b'Invalid name')):
                raise RuntimeError(response.decode('utf8','replace'))
    s.close()

if __name__=='__main__':
    p=argparse.ArgumentParser(); p.add_argument('action',choices=['cli']); p.add_argument('output'); p.add_argument('commands',nargs='+'); a=p.parse_args()
    cli(a.output,a.commands)
