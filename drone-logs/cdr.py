import struct
class R:
    def __init__(s,b): s.b=b[4:]; s.o=0
    def al(s,n): s.o=(s.o+n-1)//n*n
    def u32(s): s.al(4); v=struct.unpack_from('<I',s.b,s.o)[0]; s.o+=4; return v
    def i32(s): s.al(4); v=struct.unpack_from('<i',s.b,s.o)[0]; s.o+=4; return v
    def u8(s): v=s.b[s.o]; s.o+=1; return v
    def f64(s,n=1): s.al(8); v=struct.unpack_from('<%dd'%n,s.b,s.o); s.o+=8*n; return v if n>1 else v[0]
    def string(s): n=s.u32(); v=s.b[s.o:s.o+n-1].decode(errors='replace'); s.o+=n; return v
    def header(s): sec=s.i32(); ns=s.u32(); fid=s.string(); return sec+ns*1e-9, fid
