# -*- coding: utf-8 -*-

# source paper: Least-Squares Rigid Motion Using SVD by Olga Sorkine-Hornung and Michael Rabinovich

import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from mpl_toolkits.mplot3d import Axes3D
from numpy import matlib
#%matplotlib qt

def rigidtrans_func(P_r_P,Q_r_Q,w=np.nan):
    
    if P_r_P.shape[1] != 3:
        P_r_P = P_r_P.T
    
    if Q_r_Q .shape[1] != 3:
        Q_r_Q  = Q_r_Q.T
    
    # check for NaN entries and delete
    Q_r_P = np.empty(P_r_P.shape)
    Q_r_P[:] = np.nan
    nan_idx = ~np.isnan(P_r_P[:,0])  #check only for x because if X = nan -> all are nan
    P_r_P = P_r_P[nan_idx,:]
    Q_r_Q = Q_r_Q[nan_idx,:]
    nn = len(P_r_P)
    mm = len(Q_r_Q)
    
    # check weights
    if w.any() == np.nan or w.shape != (nn,1) or w.shape != (1,nn):
        w = np.ones(nn)
        
    # find data centroid and deviations from centroid
    pmean = np.sum(P_r_P * np.matlib.repmat(w,3,1).T,axis=0)/np.sum(w)
    p = P_r_P - np.matlib.repmat(pmean, nn, 1)
    
    qmean = np.sum(Q_r_Q * np.matlib.repmat(w,3,1).T,axis=0)/np.sum(w)
    q = Q_r_Q - np.matlib.repmat(qmean, mm, 1)
    
    # covariance matrix
    C = p.T@np.diag(w)@q
    
    #singular value decomposition
    [U,sdiag,VH] = np.linalg.svd(C)
    V = VH.T.conj() #https://stackoverflow.com/questions/50930899/svd-command-in-python-v-s-matlab
    
    #% handle the reflection case
    R_QP = V@np.diag([1,1,np.sign(np.linalg.det(U@V.T))])@U.T
    
    
    #compute the translation
    Q_t_PQ = qmean.T - R_QP@pmean.T;
    
    # calculate transformed points
    Q_r_P[nan_idx,:] = (R_QP@P_r_P.T + np.matlib.repmat(Q_t_PQ,nn,1).T).T
    
    return (R_QP,Q_t_PQ,Q_r_P)
