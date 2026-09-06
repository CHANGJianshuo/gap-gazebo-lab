"""Quintic interpolation used by ROS joint_trajectory_controller (p,v,a inputs).
Bounds are extrema within each polynomial segment; collision validation remains sampled.
"""
import math
import numpy as np

def segment(q0,v0,a0,q1,v1,a1,T):
 q0,v0,a0,q1,v1,a1=map(lambda x:np.asarray(x,dtype=float),(q0,v0,a0,q1,v1,a1))
 b=q1-q0-v0*T-a0*T*T/2;c=v1*T-v0*T-a0*T*T;d=(a1-a0)*T*T
 return np.array([q0,v0*T,a0*T*T/2,10*b-4*c+d/2,-15*b+7*c-d,6*b-3*c+d/2])

def derivative(c,k=1):
 for _ in range(k):c=np.array([i*c[i] for i in range(1,len(c))])
 return c

def extrema(c):
 values=[]
 for j in range(c.shape[1]):
  roots=np.polynomial.polynomial.polyroots(derivative(c)[:,j]) if len(c)>1 else []
  u=[0.,1.]+[float(r.real) for r in roots if abs(r.imag)<1e-8 and 0<r.real<1]
  values.append(max(abs(np.polynomial.polynomial.polyval(t,c[:,j])) for t in u))
 return max(values)

def required_scale(segments,velocity=.6,acceleration=1.2,jerk=8.):
 peaks=[0.,0.,0.]
 for c,T in segments:
  for k in (1,2,3):peaks[k-1]=max(peaks[k-1],extrema(derivative(c,k))/T**k)
 factor=max(1.,peaks[0]/velocity,math.sqrt(peaks[1]/acceleration),(peaks[2]/jerk)**(1/3))*1.01
 return factor,{'velocity':peaks[0]/factor,'acceleration':peaks[1]/factor**2,'jerk':peaks[2]/factor**3}
