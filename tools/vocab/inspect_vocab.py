#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Independent structural validation of our little-endian FBoW output."""
import struct,sys,json,math
from pathlib import Path
def inspect(path):
 b=Path(path).read_bytes();u32=lambda o:struct.unpack_from('<I',b,o)[0];u64=lambda o:struct.unpack_from('<Q',b,o)[0]
 assert u64(0)==55824124 and b[8:11]==b'ORB'
 blocks=u32(64);stride=u64(80);features=u64(88);children=u64(96);k=u32(120)
 assert len(b)==128+blocks*stride and u64(104)==blocks*stride
 assert u32(112)==0 and u32(116)==32 and u64(72)==32
 assert 2<=k<=16 and children==features+32*k and stride>=children+k*8
 seen=set();words=set();depths=[];pending=[(0,0,0)]
 while pending:
  block,level,parent=pending.pop();assert block<blocks and block not in seen;seen.add(block);o=128+block*stride
  n,leaf=struct.unpack_from('<HH',b,o);assert 1<=n<=k and u32(o+4)==parent
  all_leaf=True
  for i in range(n):
   child=u32(o+children+8*i)
   if child&0x80000000:
    word=child&0x7fffffff;assert word not in words;words.add(word);depths.append(level+1)
    weight=struct.unpack_from('<f',b,o+children+8*i+4)[0];assert math.isfinite(weight) and weight>0
   else:all_leaf=False;assert child!=0;pending.append((child,level+1,block))
  assert leaf==all_leaf
 assert len(seen)==blocks and words==set(range(len(words))) and max(depths)<=7
 return dict(blocks=blocks,words=len(words),min_depth=min(depths),max_depth=max(depths),bytes=len(b))
if __name__=='__main__':print(json.dumps(inspect(sys.argv[1]),indent=2))
