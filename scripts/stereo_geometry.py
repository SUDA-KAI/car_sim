"""Metric stereo visual odometry geometry. No simulator pose input."""
import cv2
import numpy as np


def estimate_delta(objects,pixels,k,base_camera,dt):
    if dt<=0 or len(objects)<20:raise ValueError('Insufficient temporal stereo tracks')
    ok,rvec,tvec,inliers=cv2.solvePnPRansac(np.asarray(objects,np.float32),np.asarray(pixels,np.float32),k,None,
        iterationsCount=150,reprojectionError=2.,confidence=.999,flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok or inliers is None or len(inliers)<20 or len(inliers)<.5*len(objects):raise ValueError('Visual PnP inlier check failed')
    current_previous=np.eye(4);current_previous[:3,:3]=cv2.Rodrigues(rvec)[0];current_previous[:3,3]=tvec[:,0]
    delta=base_camera@np.linalg.inv(current_previous)@np.linalg.inv(base_camera)
    angle=float(np.linalg.norm(cv2.Rodrigues(delta[:3,:3])[0]))
    if np.linalg.norm(delta[:3,3])>1.0*dt+.05 or angle>.8*dt+.05:raise ValueError('Implausible visual jump')
    projected=cv2.projectPoints(objects[inliers[:,0]],rvec,tvec,k,None)[0].reshape(-1,2)
    error=float(np.sqrt(np.mean(np.sum((projected-pixels[inliers[:,0]])**2,axis=1))))
    return delta,len(inliers),error


def stereo_features(left,right,k,baseline):
    stereo=cv2.StereoSGBM_create(minDisparity=0,numDisparities=128,blockSize=5,
        P1=8*25,P2=32*25,disp12MaxDiff=1,uniquenessRatio=10,
        speckleWindowSize=50,speckleRange=2,mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY)
    disparity=stereo.compute(left,right).astype(np.float32)/16.
    mask=np.full(left.shape,255,np.uint8);mask[:16]=0;mask[-16:]=0;mask[:,:140]=0;mask[:,-16:]=0
    corners=cv2.goodFeaturesToTrack(left,maxCorners=650,qualityLevel=.015,minDistance=7,mask=mask,blockSize=5)
    if corners is None:return np.empty((0,2),np.float32),np.empty((0,3),np.float32)
    points=[];objects=[]
    for point in corners[:,0]:
        u,v=map(float,point);x,y=round(u),round(v)
        patch=disparity[y-1:y+2,x-1:x+2];valid=patch[patch>1.]
        if len(valid)<5:continue
        d=float(np.median(valid));z=k[0,0]*baseline/d
        if not .4<z<20.:continue
        points.append([u,v]);objects.append([(u-k[0,2])*z/k[0,0],(v-k[1,2])*z/k[1,1],z])
    return np.asarray(points,np.float32).reshape(-1,2),np.asarray(objects,np.float32).reshape(-1,3)
