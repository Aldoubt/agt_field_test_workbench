"""Physical accuracy and AP50 calculations; missing records remain NOT_RUN."""
import math
from pathlib import Path
from .maps import hash_file
FORMAL={5:.12,7:.25,10:.35}
ENGINEERING={5:.12,7:.15,10:.20}

def navigation(trials):
    if any(r.get('distance_m') not in FORMAL for r in trials):raise ValueError('unsupported test distance')
    rows=[]
    for distance,limit in FORMAL.items():
        group=[r for r in trials if r.get('distance_m')==distance]
        errors=[];trial_results=[]
        for r in group:
            if r.get('truth_source') not in ('physical_ruler','laser_rangefinder','survey') or not r.get('evidence'):
                raise ValueError('independent physical truth and evidence required')
            evidence=Path(r['evidence']);expected=r.get('evidence_sha256')
            if not evidence.is_file() or hash_file(evidence)!=expected:raise ValueError('measurement evidence SHA mismatch')
            if len(r['goal_xy'])!=2 or len(r['measured_xy'])!=2:raise ValueError('XY requires two coordinates')
            e=math.dist(r['goal_xy'],r['measured_xy'])
            if not math.isfinite(e):raise ValueError('nonfinite arrival error')
            errors.append(e);trial_results.append(dict(trial_index=len(errors),goal_xy=r['goal_xy'],measured_xy=r['measured_xy'],truth_source=r['truth_source'],evidence=str(evidence),evidence_sha256=expected,error_m=e))
        mean=sum(errors)/len(errors) if errors else None
        complete=len(errors)==5
        rows.append(dict(distance_m=distance,trials=len(errors),errors_m=errors,trial_results=trial_results,mean_error_m=mean,
            formal_limit_m=limit,formal_state=('PASS' if mean<=limit else 'BLOCKED') if complete else 'NOT_RUN',
            engineering_limit_m=ENGINEERING[distance],engineering_state=('PASS' if mean<=ENGINEERING[distance] else 'WARN') if complete else 'NOT_RUN'))
    return dict(source='user thresholds; uploaded formal document comparison pending',engineering_source='agt_navigation_v3/docs/acceptance/FIELD_ACCEPTANCE_V1.md @ b3a6048',results=rows)

def ap50(ground_truth,predictions,images,model):
    if not ground_truth or not predictions:return dict(state='NOT_RUN',map50=None)
    for p,sha in images.items():
        if hash_file(p)!=sha:raise ValueError('test image changed')
    if not model.get('version') or not model.get('sha256'):raise ValueError('model version/SHA required')
    if not images:raise ValueError('image evidence required')
    for row in ground_truth+predictions:
        if row['image'] not in images:raise ValueError('annotation image has no SHA evidence')
        b=row['box']
        if len(b)!=4 or not all(math.isfinite(v) for v in b) or b[0]>=b[2] or b[1]>=b[3]:raise ValueError('invalid box')
    if any(not math.isfinite(p['score']) or not 0<=p['score']<=1 for p in predictions):raise ValueError('invalid prediction score')
    def iou(a,b):
        x=max(0,min(a[2],b[2])-max(a[0],b[0]));y=max(0,min(a[3],b[3])-max(a[1],b[1]));inter=x*y
        return inter/max((a[2]-a[0])*(a[3]-a[1])+(b[2]-b[0])*(b[3]-b[1])-inter,1e-12)
    scores={}
    for cls in sorted({r['class'] for r in ground_truth}):
        gt=[r for r in ground_truth if r['class']==cls];used=set();tp=0;fp=0;curve=[]
        for p in sorted((r for r in predictions if r['class']==cls),key=lambda r:r['score'],reverse=True):
            matches=[(iou(p['box'],g['box']),i) for i,g in enumerate(gt) if g['image']==p['image'] and i not in used]
            best=max(matches,default=(0,-1))
            if best[0]>=.5:tp+=1;used.add(best[1])
            else:fp+=1
            curve.append((tp/len(gt),tp/(tp+fp)))
        scores[cls]=sum(max((p for r,p in curve if r>=i/100),default=0) for i in range(101))/101
    value=sum(scores.values())/len(scores)
    return dict(state='PASS' if value>=.8 else 'BLOCKED',map50=value,per_class=scores,threshold=.8,model=model)
