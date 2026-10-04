#!/usr/bin/env python3
# SPDX-License-Identifier: MIT  (own code)
"""Python prototype of the REJECTED coloured-noise variant of section 14 (idea 2): generalised least squares for the geo-referencing similarity with AR(1) fix errors
(correlation time tauc, whitening of design and data by z_i - phi_i z_{i-1}, phi_i = exp(-dt_i / tauc)), causal, from the fix-free gait stream of the phone pipeline
(runs/phone_pipeline/<seq>/fuse_full_gait/causal.live). 'white' = same code with tauc = 0. The C implementation (gf_georef) uses white weights and only
lowers the weight of the scale information by the number of fixes per correlation time (corr_s).   usage: python georef_gls_proto.py   (numpy)"""
import sys; from pathlib import Path
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE)); sys.path.insert(0, str(HERE.parent.parent / 'phone_pipeline'))
import numpy as np, score as S, run as R, fuse_eval as F  # noqa
def fit(pf,z,t,tn,forget,tauc,rigid,huber=0,nit=1,minn=8):
    """GLS similarity (a,b,tx,ty): z = [a -b; b a] p + t, AR(1) noise (tauc) , exponential forgetting, optional Huber IRLS"""
    n=len(t)
    w=np.exp(-(tn-t)/forget) if forget>0 else np.ones(n)
    phi=np.zeros(n)
    if tauc>0: phi[1:]=np.exp(-np.diff(t)/tauc)
    # design rows (x then y) per fix: x: [px,-py,1,0], y: [py,px,0,1]
    A=np.zeros((n,2,4)); A[:,0,0]=pf[:,0]; A[:,0,1]=-pf[:,1]; A[:,0,2]=1; A[:,1,0]=pf[:,1]; A[:,1,1]=pf[:,0]; A[:,1,3]=1
    y=z.copy()
    # whitening: row_i' = (row_i - phi_i row_{i-1})/sqrt(1-phi_i^2)  (first row unchanged)
    sc=np.ones(n); sc[1:]=1/np.sqrt(1-phi[1:]**2)
    Aw=A.copy(); yw=y.copy()
    Aw[1:]=(A[1:]-phi[1:,None,None]*A[:-1])*sc[1:,None,None]; yw[1:]=(y[1:]-phi[1:,None]*y[:-1])*sc[1:,None]
    ww=w.copy(); 
    for it in range(nit+1):
        M=np.einsum('n,nrk,nrl->kl',ww,Aw,Aw); b=np.einsum('n,nrk,nr->k',ww,Aw,yw)
        th=np.linalg.solve(M+1e-9*np.eye(4),b)
        if huber<=0 or it==nit: break
        r=np.linalg.norm(yw-np.einsum('nrk,k->nr',Aw,th),axis=1); s=max(1.4826*np.median(r),0.5)
        ww=w*np.where(r<=huber*s,1,huber*s/np.maximum(r,1e-9))
    a,bq,tx,ty=th
    if rigid:
        s_=np.hypot(a,bq); a,bq=a/s_,bq/s_
        # translation by GLS given rotation
        yy=yw-np.einsum('nrk,k->nr',Aw[:,:,:2],[a,bq])
        wsum=np.zeros((2,2)); rhs=np.zeros(2)
        for k in (2,3):
            pass
        M2=np.einsum('n,nrk,nrl->kl',ww,Aw[:,:,2:],Aw[:,:,2:]); b2=np.einsum('n,nrk,nr->k',ww,Aw[:,:,2:],yy)
        tx,ty=np.linalg.solve(M2+1e-9*np.eye(2),b2)
    return a,bq,tx,ty
def georef(stream,fx,t_start,forget=0,tauc=0,rigid=False,huber=0,minext=30.0,minfix=8):
    t=stream[:,0]; P=stream[:,1:4]
    pf=np.c_[np.interp(fx[:,0],t,P[:,0]),np.interp(fx[:,0],t,P[:,1]),np.interp(fx[:,0],t,P[:,2])]
    res=np.full((len(t),3),np.nan); cur=None; nxt=0
    fi=0
    for i in range(len(t)):
        while fi<len(fx) and fx[fi,0]<=t[i]:
            fi+=1
            if fx[fi-1,0]>=t_start-1e-6 or True:
                m=fi
                if m>=minfix and fx[fi-1,0]>=t_start:
                    ext=np.ptp(pf[:m,0])+np.ptp(pf[:m,1])
                    if ext>=minext:
                        cur=fit(pf[:m,:2],fx[:m,1:3],fx[:m,0],fx[fi-1,0],forget,tauc,rigid,huber)
                        # z offset
                        a,b,_,_=cur; s=np.hypot(a,b) if not rigid else 1.0
                        w=np.exp(-(fx[fi-1,0]-fx[:m,0])/forget) if forget>0 else np.ones(m)
                        tz=(w*(fx[:m,3]-s*pf[:m,2])).sum()/w.sum()
                        cur=(a,b,cur[2],cur[3],s,tz)
        if cur is not None:
            a,b,tx,ty,s,tz=cur
            res[i,0]=a*P[i,0]-b*P[i,1]+tx; res[i,1]=b*P[i,0]+a*P[i,1]+ty; res[i,2]=s*P[i,2]+tz
    ok=~np.isnan(res[:,0])
    return np.c_[t[ok],res[ok],np.tile([0,0,0,1.0],(ok.sum(),1))]
if __name__=='__main__':
    opts=[('sim white',dict()),('sim tauc40',dict(tauc=40)),('sim tauc80',dict(tauc=80)),('rigid white',dict(rigid=True)),('rigid tauc40',dict(rigid=True,tauc=40)),
          ('sim tauc40 f600',dict(tauc=40,forget=600)),('sim tauc40 f300',dict(tauc=40,forget=300)),('sim tauc40 huber',dict(tauc=40,huber=2.0)),('sim tauc40 minext60',dict(tauc=40,minext=60))]
    rows={}
    for seq in ('outdoor1','outdoor2','advio20'):
        case,ft=F._case(seq); d=R.OUT/seq
        st=np.loadtxt(d/'fuse_full_gait/causal.live'); st=st[(st[:,8].astype(int)&1)==1]
        fx=case.gps[:,:4]
        for name,o in opts:
            g=georef(st,fx,ft[0]+30,**o); m=S.metrics(case,g,ft,tmin=ft[0]+30)
            rows.setdefault(name,[]).append(m['se3'])
    for k,v in rows.items(): print('%-22s'%k,' '.join('%6.2f'%x for x in v))
    print('GNSS 5.73 14.66 12.00 ; gait stream 4.73 13.71 11.76')
