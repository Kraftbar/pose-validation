import io, sys, zipfile, urllib.request
class HTTPFile(io.RawIOBase):
    def __init__(s,url,size=None):
        s.url=url; s.pos=0
        import time
        for t in range(60):
            try:
                r=urllib.request.Request(url,headers={'Range':'bytes=0-0','User-Agent':'Mozilla/5.0'})
                s.size=int(urllib.request.urlopen(r,timeout=60).headers['Content-Range'].split('/')[1]); break
            except Exception as ex:
                print('retry init',ex,file=sys.stderr); time.sleep(30)
    def seekable(s): return True
    def readable(s): return True
    def tell(s): return s.pos
    def seek(s,o,w=0):
        s.pos = o if w==0 else s.pos+o if w==1 else s.size+o
        return s.pos
    def readinto(s,b):
        import time
        if s.pos>=s.size: return 0
        c=getattr(s,'cache',None)
        if not c or not (c[0]<=s.pos<c[0]+len(c[1])):
            st=s.pos; e=min(st+(32<<20),s.size)-1
            for t in range(30):
                try:
                    r=urllib.request.Request(s.url,headers={'Range':f'bytes={st}-{e}','User-Agent':'Mozilla/5.0'})
                    d=urllib.request.urlopen(r,timeout=300).read(); break
                except Exception as ex:
                    print('retry',ex,file=sys.stderr); time.sleep(20)
            s.cache=c=(st,d)
        o=s.pos-c[0]; d=c[1][o:o+len(b)]
        b[:len(d)]=d; s.pos+=len(d); return len(d)
if __name__=='__main__':
    url=sys.argv[1]
    z=zipfile.ZipFile(io.BufferedReader(HTTPFile(url),buffer_size=1<<20))
    for i in z.infolist():
        if i.filename.endswith('.zip') or len(sys.argv)>2: print(i.filename,i.compress_type,i.file_size//1e6,i.compress_size//1e6)
