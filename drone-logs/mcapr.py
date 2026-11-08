"""Minimal streaming MCAP reader (uncompressed chunks), enough for ROS 2 bags from rosbag2."""
import struct
def _s(b,o):
    n=struct.unpack_from('<I',b,o)[0]; return b[o+4:o+4+n].decode(errors='replace'),o+4+n
def read(path, topics=None):
    """yield (topic, schema, log_time_ns, cdr_bytes)"""
    schemas={}; chans={}
    with open(path,'rb') as f:
        assert f.read(8)==b'\x89MCAP0\r\n'
        while True:
            h=f.read(9)
            if len(h)<9: return
            op=h[0]; L=struct.unpack_from('<Q',h,1)[0]
            if op in (6,3,4,5):
                body=f.read(L)
                if len(body)<L: return
                recs=[(op,body)]
                if op==6:
                    o=28; c,o=_s(body,o); assert c=='', 'compressed chunk: '+c
                    n=struct.unpack_from('<Q',body,o)[0]; o+=8; rec=body[o:o+n]; q=0; recs=[]
                    while q+9<=len(rec):
                        op2=rec[q]; L2=struct.unpack_from('<Q',rec,q+1)[0]; recs.append((op2,rec[q+9:q+9+L2])); q+=9+L2
                for op2,b in recs:
                    if op2==3:
                        sid=struct.unpack_from('<H',b,0)[0]; nm,_=_s(b,2); schemas[sid]=nm
                    elif op2==4:
                        cid,sid=struct.unpack_from('<HH',b,0); t,_=_s(b,4); chans[cid]=(t,sid)
                    elif op2==5:
                        cid,seq,lt,pt=struct.unpack_from('<HIQQ',b,0)
                        t,sid=chans[cid]
                        if topics is None or t in topics:
                            yield t, schemas.get(sid,''), lt, b[22:]
            else:
                f.seek(L,1)
