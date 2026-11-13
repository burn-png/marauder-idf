import numpy as np
N=256
n=np.arange(N)
def cosw(a): return sum(((-1)**k)*c*np.cos(2*np.pi*k*n/N) for k,c in enumerate(a))
W={
 'RECT':np.ones(N),
 'HANN':cosw([0.5,0.5]),
 'HAMMING':cosw([0.54,0.46]),
 'BLACKMAN':cosw([0.42,0.5,0.08]),
 'BLK-HARRIS':cosw([0.35875,0.48829,0.14128,0.01168]),
 'FLATTOP':cosw([0.21557895,0.41663158,0.277263158,0.083578947,0.006947368]),
 'BARTLETT':1-np.abs(2*n/N-1),
 'TRIANGULAR':1-np.abs(n-N/2)/(N/2+1),
}
for k,w in W.items():
    pk=np.max(np.abs(w)); cg=w.sum()/N/pk
    enbw=N*np.sum(w**2)/np.sum(w)**2
    P=1<<16
    S=np.abs(np.fft.fft(w,P)); S/=S[0]
    d=P/N  # samples per bin
    # first null
    i=1
    while S[i]<=S[i-1]*1.0000001 or i<2: i+=1
    # find local min
    j=1
    while S[j+1]<S[j]: j+=1
    side=20*np.log10(S[j:int(P/2)].max())
    # 3dB width
    k3=np.where(S<10**(-3/20))[0][0]
    bw3=2*k3/d
    scal=-20*np.log10(S[int(d/2)])
    print(f"{k:11s} peak={pk:.4f} CG={cg:.4f} ENBW={enbw:.2f} side={side:.1f}dB bw3={bw3:.2f} scal={scal:.2f}dB")
