import sys, numpy as np
import os
R = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')) + '/'
def load(p): return np.loadtxt(p)
OFFS = {'outdoor1': 292.89, 'indoor2': 282.99, 'indoor1': 282.52}
SEQ = 'outdoor1'
def gt(seq=None):
    seq = seq or SEQ
    g = load(R + f'external/gnss/rob/{seq}/gt.tum'); g[:, 0] -= OFFS[seq]; return g  # GT clock offset of Outdoor-1 (phone_offsets.json / estimate_offset.py)
def umeyama_scale(A,B):  # B ~ s R A + t ; return s, rms
    ma=A.mean(0); mb=B.mean(0); Ac=A-ma; Bc=B-mb
    H=Ac.T@Bc/len(A); U,D,Vt=np.linalg.svd(H); S=np.eye(3)
    if np.linalg.det(U@Vt)<0: S[2,2]=-1
    R=(U@S@Vt).T  # maps A->B
    s=np.trace(np.diag(D)@S)/ (Ac**2).sum()*len(A)
    res=B-(s*(R@A.T).T+ (mb - s*R@ma))
    return s, np.sqrt((res**2).sum(1).mean())
def table(f, win=12, step=6, a0=0, a1=390, gravity_up=False):
    L=load(f); G=gt(); t0=L[0,0]; out=[]
    for a in np.arange(a0,a1-win,step):
        ts=np.arange(t0+a,t0+a+win+1e-6,1.0)
        if ts[0]<L[0,0] or ts[-1]>L[-1,0] or ts[-1]>G[-1,0]: continue
        P=np.stack([np.interp(ts,L[:,0],L[:,i]) for i in (1,2,3)],1)
        Q=np.stack([np.interp(ts,G[:,0],G[:,i]) for i in (1,2,3)],1)
        if np.linalg.norm(Q[-1]-Q[0])<3: out.append((a,np.nan,np.nan,np.linalg.norm(Q[-1]-Q[0]),np.linalg.norm(P[-1]-P[0]))); continue
        s,r=umeyama_scale(P,Q)
        out.append((a,s,r,np.linalg.norm(Q[-1]-Q[0]),np.linalg.norm(P[-1]-P[0])))
    return out
if __name__=='__main__':
    for a,s,r,qc,pc in table(sys.argv[1], float(sys.argv[2]) if len(sys.argv)>2 else 12, float(sys.argv[3]) if len(sys.argv)>3 else 6):
        print(f'{a:5.0f}-{a+float(sys.argv[2]) if len(sys.argv)>2 else a+12:<5.0f} sim3 scale {s:7.3f} (m/unit) rms {r:5.2f} m  gt-chord {qc:5.1f} m map-chord {pc:6.2f} u  chord ratio {qc/pc:6.3f}')
