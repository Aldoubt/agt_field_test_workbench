import sys,json,yaml
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.ndimage import map_coordinates
sys.path.insert(0,'src/agt_field_test_workbench')
from workbench.maps import read_pgm
exp=Path('runs/map_cleanup_20261024T1320'); r=json.loads((exp/'registered_candidate.json').read_text()); v=json.loads((exp/'visual_metrics.json').read_text())
def load(path):
    meta=yaml.safe_load(Path(path).read_text()); p=Path(path).parent/meta['image'];w,h,pixels=read_pgm(p)
    a=np.frombuffer(pixels,dtype=np.uint8).reshape(h,w); ox,oy,_=meta['origin'];res=meta['resolution']
    return meta,a,(ox,ox+w*res,oy,oy+h*res)
bm,b,be=load(exp/'before_navigation/map.yaml'); cm,c,ce=load(r['grid']['yaml'])
fig,axes=plt.subplots(2,2,figsize=(15,13),constrained_layout=True)
center=v['detail_center_xy']
for row in range(2):
    for ax,arr,extent,title in zip(axes[row],[b,c],[be,ce],['Before cleanup','After cleanup: same projection parameters']):
        ax.imshow(arr,cmap='gray',vmin=0,vmax=255,origin='upper',extent=extent,interpolation='nearest')
        ax.set_title(title);ax.set_aspect('equal');ax.set_xlabel('World X (m)');ax.set_ylabel('World Y (m)')
        if row==1:ax.set_xlim(center[0]-6,center[0]+6);ax.set_ylim(center[1]-6,center[1]+6)
fig.suptitle('2D occupancy | black: occupied | white: free | gray: unknown')
fig.savefig(exp/'grid_cleanup_comparison.png',dpi=140);plt.close(fig)
# Align after into before's raster; compare only shared cell centers.
yy,xx=np.indices(b.shape); res=bm['resolution']; wx=bm['origin'][0]+(xx+.5)*res; wy=bm['origin'][1]+(b.shape[0]-yy-.5)*res
cx=np.floor((wx-cm['origin'][0])/cm['resolution']).astype(int); cy=c.shape[0]-1-np.floor((wy-cm['origin'][1])/cm['resolution']).astype(int)
valid=(cx>=0)&(cx<c.shape[1])&(cy>=0)&(cy<c.shape[0]); after=np.full_like(b,205);after[valid]=c[cy[valid],cx[valid]]
report={'before':{'occupied':int((b==0).sum()),'free':int((b==254).sum()),'unknown':int((b==205).sum())},
        'after':{'occupied':int((c==0).sum()),'free':int((c==254).sum()),'unknown':int((c==205).sum())},
        'shared_grid_before_occupied_cleared':int(((b==0)&(after!=0)&valid).sum()),
        'shared_grid_before_occupied_retained':int(((b==0)&(after==0)&valid).sum())}
(exp/'grid_comparison_metrics.json').write_text(json.dumps(report,indent=2)+'\n');print(report)
