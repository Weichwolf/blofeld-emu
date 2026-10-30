# SPDX-License-Identifier: GPL-3.0-only
"""Independent own-ROM input and complete NVM wire expectations; no project imports."""
import binascii,struct,stat
from pathlib import Path
SIZE=32768;TARGET=0x2000

def cleared(bits,n):bits[n//8]&=~(1<<(n%8))
def setbit(bits,n):bits[n//8]|=1<<(n%8)
def initial(command=0x20,divider=0x48,wait=False):
 data=bytearray([255])*SIZE;known=bytearray([255])*4096;fresh=bytearray([255])*2048
 def put(address,value):
  offset=address-0x8000;data[offset]=value;cleared(fresh,offset//2)
 def store(address,value):return bytes([0x86,value,0x7a,address>>8,address&255])
 algorithm=store(0x3c,0)+store(0x100,divider)
 if wait:algorithm+=store(0x103,0x40)
 algorithm+=bytes.fromhex('cc12347ca000')+store(0x106,command)+store(0x105,0x80)
 algorithm+=bytes.fromhex('10ef3e') if wait else bytes.fromhex('1f010540fb')
 algorithm+=store(0x1700,0x55)+bytes.fromhex('20fe')
 boot=bytes.fromhex('cf1ff0')+b''.join(store(0x1800+i,value) for i,value in enumerate(algorithm))+bytes.fromhex('061800')
 for i,b in enumerate(boot):put(0x8000+i,b)
 isr=store(0x103,0)+store(0x1701,0x55)+bytes([0x0b])
 for i,b in enumerate(isr):put(0x8700+i,b)
 for address,dest in [(0xffb8,0x8700),(0xfffe,0x8000)]:put(address,dest>>8);put(address+1,dest&255)
 put(0xff0d,255);put(0xff0f,254)
 if command==0x40:put(0xa000,0)
 for i,b in enumerate(b'EMULIVE00001\0'):put(0x81e0+i,b)
 return data,known,fresh

def transition(image,state):
 data,known,fresh=(bytearray(part) for part in image)
 if state=='initial':return data,known,fresh
 if state=='unknown':
  for i in range(TARGET,TARGET+2):data[i]=0;cleared(known,i);cleared(fresh,i//2)
 elif state=='program':data[TARGET:TARGET+2]=bytes.fromhex('1234');cleared(fresh,TARGET//2)
 elif state=='sector':
  for i in range(TARGET,TARGET+512):data[i]=255;setbit(known,i);setbit(fresh,i//2)
 elif state=='mass':data[:]=bytes([255])*SIZE;known[:]=bytes([255])*4096;fresh[:]=bytes([255])*2048
 else:raise AssertionError(state)
 return data,known,fresh

def worker_initial():
 data,known,fresh=initial()
 for address,value in [(0x8800+i,b) for i,b in enumerate(bytes.fromhex('b6a000068000'))]+[(0xfffe,0x88),(0xffff,0)]:
  offset=address-0x8000;data[offset]=value;cleared(fresh,offset//2)
 return data,known,fresh

def encoded(image):
 data,known,fresh=image;assert len(data)==32768 and len(known)==4096 and len(fresh)==2048
 for i,b in enumerate(data):
  assert known[i//8]&(1<<(i%8)) or b==0
  if fresh[(i//2)//8]&(1<<((i//2)%8)):assert b==255 and known[i//8]&(1<<(i%8))
 body=b'BLFNVM\r\n'+struct.pack('<II',1,38912)+data+known+fresh
 return body+struct.pack('<I',binascii.crc32(body))

def verify(path,image):
 path=Path(path);data=path.read_bytes();expected=encoded(image)
 assert len(data)==38932 and data==expected,(str(path),len(data),len(expected))
 assert stat.S_IMODE(path.stat().st_mode)==0o600
 assert not path.is_symlink()
 return {'path':str(path),'bytes':len(data),'crc32':struct.unpack('<I',data[-4:])[0]}

def normal_outputs(directory):
 result=[]
 for name,command,divider,wait,state in [('program',0x20,0x48,False,'program'),('fractional',0x20,0x3c,False,'program'),('sector',0x40,0x48,False,'sector'),('mass',0x41,0x48,False,'mass'),('verify',5,0x48,False,'initial'),('wai',0x20,0x48,True,'program'),('reset',0x40,0x48,False,'sector')]:
  image=initial(command,divider,wait);path=Path(directory)/(name+'.bfnvm');result.append(verify(str(path)+'.initial',image));result.append(verify(path,transition(image,state)))
 return result

def durable_outputs(directory):
 result=[]
 for spacing,count in [(2,32),(64,2)]:
  data=bytearray([255])*32768;known=bytearray([255])*4096;fresh=bytearray([255])*2048
  data[0x7f0f]=254;cleared(fresh,0x7f0f//2)
  for n in range(count):
   offset=TARGET+n*spacing;data[offset:offset+2]=bytes.fromhex('1234');cleared(fresh,offset//2)
  result.append(verify(Path(directory)/('row-'+str(spacing)+'.bfnvm'),(data,known,fresh)))
 result.append(verify(Path(directory)/'mass-verify.bfnvm',(bytearray([255])*32768,bytearray([255])*4096,bytearray([255])*2048)))
 return result
