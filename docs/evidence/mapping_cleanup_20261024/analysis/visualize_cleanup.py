from pathlib import Path
import json,sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import LogNorm
from scipy.spatial import cKDTree
sys.path.insert(0,'src/agt_field_test_workbench')
from workbench.maps import read_pgm
root=Path('runs/map_cleanup_20261024T1320'); a=root/'indexed_full/artifact'
def load(path):
    with Path(path).open('rb') as f:
        while not f.readline().startswith(b'DATA binary'):pass
        return np.frombuffer(f.read(),dtype='<f4').reshape(-1,3)
raw=np.memmap(a/'points.raw',dtype='<f4',mode='r').reshape(-1,3)
clean=load(a/'temporal_cleanup/clouds/static_map.pcd')
removed=load(a/'temporal_cleanup/clouds/removed_candidates.pcd')
poses=np.genfromtxt(a/'trajectory.csv',delimiter=',',names=True)
xy=np.column_stack([poses['x'],poses['y']]); tree=cKDTree(xy)
limits=np.percentile(raw[:,:2],[.05,99.95],axis=0)
lo=np.floor(limits[0]);hi=np.ceil(limits[1]); bins=[np.arange(lo[i],hi[i]+.1,.1) for i in range(2)]
# Restrict the comparison to the collision-height band, fixed in this gravity-aligned map.
ground=np.percentile(raw[:,2],5); band=(ground+.2,ground+1.8)
fig,axes=plt.subplots(1,3,figsize=(20,8),constrained_layout=True)
for ax,points,title in zip(axes,[raw,clean,removed],['Before cleanup','Static candidate','Removed candidates']):
    p=points[(points[:,2]>band[0])&(points[:,2]<band[1])]
    hist,*_=np.histogram2d(p[:,0],p[:,1],bins=bins)
    ax.imshow(hist.T,origin='lower',extent=[lo[0],hi[0],lo[1],hi[1]],norm=LogNorm(1,100),cmap='magma')
    ax.plot(xy[:,0],xy[:,1],color='#30bad4',lw=.5,alpha=.8)
    ax.set_title(title);ax.set_xlabel('World X (m)');ax.set_ylabel('World Y (m)');ax.set_aspect('equal');ax.set_facecolor('#101824')
fig.suptitle('Same 4,912 scans | fixed height band | cyan: recorded LIO trajectory',fontsize=14)
fig.savefig(root/'cleanup_comparison.png',dpi=150);plt.close(fig)
# Inspect the strongest removed elevated cluster close to the driven path.
dist,_=tree.query(removed[:,:2],workers=2)
p=removed[(dist<1.5)&(removed[:,2]>band[0])&(removed[:,2]<band[1])]
hist,xe,ye=np.histogram2d(p[:,0],p[:,1],bins=[np.arange(lo[0],hi[0]+1,1),np.arange(lo[1],hi[1]+1,1)])
i,j=np.unravel_index(hist.argmax(),hist.shape);center=[xe[i]+.5,ye[j]+.5]
fig,axes=plt.subplots(1,3,figsize=(15,5),constrained_layout=True)
for ax,points,title in zip(axes,[raw,clean,removed],['Before cleanup','Static candidate','Removed candidates']):
    mask=(np.abs(points[:,0]-center[0])<5)&(np.abs(points[:,1]-center[1])<5)&(points[:,2]>band[0])&(points[:,2]<band[1]);p=points[mask]
    ax.scatter(p[::max(1,len(p)//120000),0],p[::max(1,len(p)//120000),1],s=.1,c='#212e42' if title!='Removed candidates' else '#d24e23',rasterized=True)
    ax.plot(xy[:,0],xy[:,1],color='#15a5b1',lw=.7);ax.set_xlim(center[0]-5,center[0]+5);ax.set_ylim(center[1]-5,center[1]+5);ax.set_aspect('equal');ax.set_title(title);ax.set_xlabel('World X (m)')
fig.suptitle('10 m crop selected by removed-point density near the trajectory (not a person label)')
fig.savefig(root/'cleanup_detail.png',dpi=170);plt.close(fig)
report={'fixed_height_band_world_z':list(map(float,band)), 'detail_center_xy':list(map(float,center)),
        'removed_candidate_near_trajectory_1_5m':int((dist<1.5).sum()),'removed_candidate_count':len(removed)}
(root/'visual_metrics.json').write_text(json.dumps(report,indent=2)+'\n')
