/* Fixed field operations; no command console. */
let fieldProfiles=[], fieldBundles=[], fieldClouds=[], fieldRoutes=[], fieldLive={}, routeMap=null, routeImage=null;
let routePoints=[], routeIndex=-1, routeParent=null, routeDrag=false, routeView={scale:1,x:0,y:0};
const routeCanvas=$('route-canvas'),routeCtx=routeCanvas.getContext('2d');
const pretty=v=>JSON.stringify(v,null,2);
async function fieldRefresh(){
  const [ps,bs,cs,rs,ms]=await Promise.all([api('profiles'),api('bundles/'+robot),api('clouds'),api('routes'),api('maps')]);
  fieldProfiles=ps;fieldBundles=bs;fieldClouds=cs;fieldRoutes=rs;maps=ms;
  const currentProfile=ps.find(p=>p.robot===robot);$('robot-config').textContent=pretty(fieldLive.session?.robot===robot?fieldLive.session.profile:currentProfile);
  $('profile-editor').value=pretty(currentProfile?.migration_candidate||currentProfile);
  const previous=$('bundle-select').value;$('bundle-select').innerHTML=bs.map(b=>`<option value="${safe(b.path)}">${safe(b.version)} · ${safe(b.status)}</option>`).join('');
  if(bs.some(b=>b.path===previous))$('bundle-select').value=previous;
  $('bundle-select').disabled=robot!=='bunker';showBundle();
  const cloud=$('cloud-version').value;$('cloud-version').innerHTML=cs.map(c=>`<option value="${safe(c.id)}">${safe(c.id)} · ${c.processing?.output_points??'?'} 点</option>`).join('');
  if(cs.some(c=>c.id===cloud))$('cloud-version').value=cloud;
  const map=$('route-map').value;$('route-map').innerHTML=ms.map(m=>`<option value="${safe(m.id)}">${safe(m.id)} · ${safe(m.status)}</option>`).join('');
  if(ms.some(m=>m.id===map))$('route-map').value=map;
  $('route-select').innerHTML='<option value="">新路线</option>'+rs.map(r=>`<option value="${safe(r.id)}">${safe(r.name||r.id)} · ${safe(r.map_id)}</option>`).join('');
  if(!routeMap&&ms.length)await openRouteMap($('route-map').value);
}
function showBundle(){let b=fieldBundles.find(b=>b.path===$('bundle-select').value);$('bundle-editor').value=b?pretty(b):'';}
$('bundle-select').onchange=showBundle;
$('bundle-save').onclick=()=>run(async()=>{let b=await api('bundles/'+robot,{bundle:JSON.parse($('bundle-editor').value)});await fieldRefresh();$('bundle-select').value=b.path;showBundle();toast('已保存 CANDIDATE 新版本')});
$('bundle-verify').onclick=()=>run(async()=>{let b=await api('bundles/verify',{source:$('bundle-select').value,report:$('bundle-report').value});await fieldRefresh();$('bundle-select').value=b.path;showBundle();toast('已绑定物理报告保存 VERIFIED 新版本')});
$('time-diagnose').onclick=()=>run(async()=>{let x=await api('diagnostics/time',{interface:$('time-interface').value});toast('时间诊断 '+x.id);await refreshJobs()});
$('robot-select').addEventListener('change',()=>run(fieldRefresh));
$('field-start').onclick=()=>run(async()=>{
  let checks=Object.fromEntries([...document.querySelectorAll('[data-check]')].map(e=>[e.dataset.check,e.checked]));
  let payload={robot,task:$('field-task').value,checks,calibration_bundle:$('bundle-select').value||null,
    mount_profile:robot==='fwmini'?selectedMount:null,geometry_file:$('field-geometry').value||null,
    gps_alignment_file:$('field-gps').value||null,safety_report:$('field-safety').value||null,enable_sensor:$('field-sensor').checked,
    livox_config:$('field-livox').value||null,map_id:$('route-map').value||null,global_backend:$('field-global').value};
  let x=await api('session/start',payload);$('field-state').textContent=pretty(x);toast('会话已启动，检查 Runs 和实时门控');
});
$('field-stop').onclick=()=>run(async()=>{$('field-state').textContent=pretty(await api('session/stop',{}));await refreshJobs()});
$('field-livox').onchange=()=>{$('mapping-livox').value=$('field-livox').value};
$('mapping-finish').onclick=()=>run(async()=>{let jobs=await api('jobs'),j=jobs.find(j=>j.kind==='mapping_live'&&j.state==='RUNNING');if(!j)throw Error('没有运行中的实机建图');await api('jobs/'+j.id+'/finish',{});toast('正在关闭 raw bag 并导出地图，请在 Runs 查看结果')});
async function showCloudVersion(id){let c=fieldClouds.find(c=>c.id===id);if(!c)return;let x=await api('clouds/'+id+'/preview');cloudPoints=x.points;drawCloud();$('cloud-note').textContent=`完整文件 ${x.total_points} 点，预览 ${x.sampled_points} 点`;$('cloud-state').textContent=pretty(c);}
$('cloud-version').onchange=()=>run(()=>showCloudVersion($('cloud-version').value));
$('cloud-import').onclick=()=>run(async()=>{toast('正在导入完整 PCD…');let x=await api('clouds/import',{path:$('cloud-import-path').value,calibration:$('bundle-select').value||selectedMount});await fieldRefresh();$('cloud-version').value=x.id;await showCloudVersion(x.id)});
async function cleanCloud(op){let id=$('cloud-version').value;if(!id)throw Error('先导入或选择 PCD');toast('正在对完整文件执行操作…');let x=await api('clouds/'+id+'/clean',{operations:[op]});await fieldRefresh();$('cloud-version').value=x.id;await showCloudVersion(x.id);toast('已保存完整 PCD 新版本')}
$('cloud-crop').onclick=()=>run(()=>cleanCloud({type:'crop',min:nums($('roi-min').value,3),max:nums($('roi-max').value,3)}));
$('cloud-delete').onclick=()=>run(()=>cleanCloud({type:'delete',min:nums($('roi-min').value,3),max:nums($('roi-max').value,3)}));
$('cloud-voxel').onclick=()=>run(()=>cleanCloud({type:'voxel',size:Number($('voxel-size').value)}));
$('cloud-project').onclick=()=>run(async()=>{let x=await api('clouds/'+$('cloud-version').value+'/project',{projection:JSON.parse($('projection-input').value)});await refreshMaps();await openMap(x.id);await fieldRefresh();toast('已生成 DRAFT 2D 地图 '+x.id)});
function mapToPixel(p,m){const dx=p.x-m.origin[0],dy=p.y-m.origin[1],c=Math.cos(m.origin[2]),s=Math.sin(m.origin[2]);return{x:(c*dx+s*dy)/m.resolution-.5,y:m.height-.5-(-s*dx+c*dy)/m.resolution}}
function pixelToMap(p,m){const x=(p.x+.5)*m.resolution,y=(m.height-p.y-.5)*m.resolution,c=Math.cos(m.origin[2]),s=Math.sin(m.origin[2]);return{x:m.origin[0]+c*x-s*y,y:m.origin[1]+s*x+c*y,yaw:0,photo:false}}
$('map-center-robot').onclick=()=>run(()=>{if(!selected||!fieldLive.pose)throw Error('需要地图和实时机器人地图位姿');let p=mapToPixel(fieldLive.pose,selected);ox=canvas.width/2-p.x*scale;oy=canvas.height/2-p.y*scale;draw()});
async function openRouteMap(id){routeMap=maps.find(m=>m.id===id);if(!routeMap)return;let img=new Image();img.src='/api/maps/'+id+'/preview';await new Promise((ok,bad)=>{img.onload=ok;img.onerror=bad});routeImage=img;routeView.scale=Math.min(routeCanvas.width/routeMap.width,routeCanvas.height/routeMap.height)*.94;routeView.x=(routeCanvas.width-routeMap.width*routeView.scale)/2;routeView.y=(routeCanvas.height-routeMap.height*routeView.scale)/2;routePoints=[];routeIndex=-1;routeParent=null;drawRoute()}
$('route-map').onchange=()=>run(()=>openRouteMap($('route-map').value));
function routeScreen(p){let a=mapToPixel(p,routeMap);return{x:routeView.x+a.x*routeView.scale,y:routeView.y+a.y*routeView.scale}}
function routeEvent(e){let r=routeCanvas.getBoundingClientRect(),x=(e.clientX-r.left)*routeCanvas.width/r.width,y=(e.clientY-r.top)*routeCanvas.height/r.height;return{x,y,pixel:{x:(x-routeView.x)/routeView.scale,y:(y-routeView.y)/routeView.scale}}}
function drawRoute(){routeCtx.fillStyle='#243735';routeCtx.fillRect(0,0,routeCanvas.width,routeCanvas.height);if(!routeImage||!routeMap)return;routeCtx.imageSmoothingEnabled=false;routeCtx.drawImage(routeImage,routeView.x,routeView.y,routeMap.width*routeView.scale,routeMap.height*routeView.scale);
  function path(points,color,width=2){if(!points?.length)return;routeCtx.beginPath();points.forEach((p,i)=>{let s=routeScreen(p);i?routeCtx.lineTo(s.x,s.y):routeCtx.moveTo(s.x,s.y)});routeCtx.strokeStyle=color;routeCtx.lineWidth=width;routeCtx.stroke()}
  let cost=fieldLive.costmap;if(cost?.map_origin){let step=Math.max(1,Math.ceil(Math.sqrt(cost.width*cost.height/16000))),c=Math.cos(cost.map_origin[2]),s=Math.sin(cost.map_origin[2]);routeCtx.fillStyle='#ec8a6060';for(let y=0;y<cost.height;y+=step)for(let x=0;x<cost.width;x+=step){let v=cost.data[y*cost.width+x];if(v<65)continue;let wx=(x+.5)*cost.resolution,wy=(y+.5)*cost.resolution,p=routeScreen({x:cost.map_origin[0]+c*wx-s*wy,y:cost.map_origin[1]+s*wx+c*wy});routeCtx.fillRect(p.x,p.y,Math.max(2,step*cost.resolution/routeMap.resolution*routeView.scale),Math.max(2,step*cost.resolution/routeMap.resolution*routeView.scale))}}
  path(fieldLive.plan,'#bb6dff',3);path(routePoints,'#13adcb');routePoints.forEach((p,i)=>{let s=routeScreen(p);routeCtx.fillStyle=i===routeIndex?'#f5a742':'#137d9c';routeCtx.beginPath();routeCtx.arc(s.x,s.y,9,0,Math.PI*2);routeCtx.fill();routeCtx.fillStyle='white';routeCtx.fillText(String(i+1),s.x+10,s.y-8)});if(fieldLive.pose){let p=routeScreen(fieldLive.pose);routeCtx.fillStyle='#e12929';routeCtx.beginPath();routeCtx.arc(p.x,p.y,7,0,Math.PI*2);routeCtx.fill();let yaw=fieldLive.pose.yaw-routeMap.origin[2];routeCtx.strokeStyle='#e12929';routeCtx.beginPath();routeCtx.moveTo(p.x,p.y);routeCtx.lineTo(p.x+20*Math.cos(yaw),p.y-20*Math.sin(yaw));routeCtx.stroke()}
  $('route-points').textContent=pretty({map_id:routeMap.id,map_sha256:routeMap.sha256,selected:routeIndex,points:routePoints});
}
routeCanvas.onpointerdown=e=>{if(!routeMap)return;let p=routeEvent(e);routeIndex=routePoints.findIndex(w=>{let q=routeScreen(w);return Math.hypot(q.x-p.x,q.y-p.y)<14});if(routeIndex<0){if(p.pixel.x<0||p.pixel.x>=routeMap.width||p.pixel.y<0||p.pixel.y>=routeMap.height)return;routePoints.push(pixelToMap(p.pixel,routeMap));routeIndex=routePoints.length-1}routeDrag=true;routeCanvas.setPointerCapture(e.pointerId);$('route-yaw').value=routePoints[routeIndex].yaw;$('route-photo').checked=routePoints[routeIndex].photo;drawRoute()};
routeCanvas.onpointermove=e=>{if(!routeDrag||routeIndex<0)return;let p=routeEvent(e);if(p.pixel.x<0||p.pixel.x>=routeMap.width||p.pixel.y<0||p.pixel.y>=routeMap.height)return;routePoints[routeIndex]={...routePoints[routeIndex],...pixelToMap(p.pixel,routeMap),yaw:Number($('route-yaw').value),photo:$('route-photo').checked};drawRoute()};
routeCanvas.onpointerup=()=>routeDrag=false;
$('route-delete').onclick=()=>{if(routeIndex>=0)routePoints.splice(routeIndex,1);routeIndex=-1;drawRoute()};
function reorder(delta){let i=routeIndex,j=i+delta;if(i<0||j<0||j>=routePoints.length)return;[routePoints[i],routePoints[j]]=[routePoints[j],routePoints[i]];routeIndex=j;drawRoute()}
$('route-up').onclick=()=>reorder(-1);$('route-down').onclick=()=>reorder(1);
$('route-apply').onclick=()=>{if(routeIndex<0)return;routePoints[routeIndex].yaw=Number($('route-yaw').value);routePoints[routeIndex].photo=$('route-photo').checked;drawRoute()};
$('route-save').onclick=()=>run(async()=>{if(!routeMap)throw Error('先选择地图');let x=await api('routes',{map_id:routeMap.id,points:routePoints,name:$('route-name').value,parent:routeParent});routeParent=x.id;await fieldRefresh();$('route-select').value=x.id;toast('已保存绑定地图的路线新版本')});
$('route-select').onchange=()=>run(async()=>{let r=fieldRoutes.find(r=>r.id===$('route-select').value);if(!r)return;$('route-map').value=r.map_id;await openRouteMap(r.map_id);routePoints=structuredClone(r.points);routeParent=r.id;$('route-name').value=r.name;drawRoute()});
const control=(op,p={})=>api('control/'+op,{robot,map_id:$('route-map').value,...p});
$('route-run').onclick=()=>run(async()=>{if(!routeParent)throw Error('先保存路线');toast(pretty(await control('route',{route_id:routeParent})))});
$('nav-cancel').onclick=()=>run(async()=>{releaseJog();toast(pretty(await control('stop')))});
$('distance-forward').onclick=()=>run(async()=>toast(pretty(await control('distance',{distance_m:Math.abs(Number($('distance-value').value))}))));
$('distance-back').onclick=()=>run(async()=>toast(pretty(await control('distance',{distance_m:-Math.abs(Number($('distance-value').value))}))));
$('distance-five').onclick=()=>run(async()=>toast(pretty(await control('distance',{distance_m:5}))));
$('loc-request').onclick=()=>run(async()=>toast(pretty(await control('relocalize'))));
$('mode-four').onclick=()=>run(async()=>toast(pretty(await control('mode',{mode:'four_wheel_steer'}))));
$('mode-lateral').onclick=()=>run(async()=>toast(pretty(await control('mode',{mode:'lateral'}))));
let jogLease=null,jogTimer=null,jogSequence=0,jogBusy=false;
function releaseJog(){let lease=jogLease;jogLease=null;if(jogTimer)clearInterval(jogTimer);jogTimer=null;if(lease)run(()=>control('stop',{lease}));}
for(let button of document.querySelectorAll('[data-jog]')){button.onpointerdown=e=>{e.preventDefault();releaseJog();jogLease=crypto.randomUUID();jogSequence=0;button.setPointerCapture(e.pointerId);let [vx,vy,wz]=button.dataset.jog.split(',').map(Number),lease=jogLease;
  const pulse=async()=>{if(jogBusy||jogLease!==lease)return;jogBusy=true;try{await control('jog',{lease,sequence:++jogSequence,vx,vy,wz})}catch(e){toast(e.message);releaseJog()}finally{jogBusy=false}};run(pulse);jogTimer=setInterval(()=>run(pulse),80)};button.onpointerup=releaseJog;button.onpointercancel=releaseJog;button.onlostpointercapture=releaseJog;}
window.addEventListener('blur',releaseJog);document.addEventListener('visibilitychange',()=>{if(document.hidden)releaseJog()});
async function liveRefresh(){fieldLive=await api('live');$('live-state').textContent=pretty({localized:fieldLive.localized,nav2:fieldLive.nav2_states,feedback:fieldLive.feedback_fresh,fwmini:fieldLive.fwmini,gps:fieldLive.gps,result:fieldLive.goal_result,unavailable:fieldLive.reason});$('field-state').textContent=pretty(fieldLive.session||fieldLive);drawRoute();}
$('diag-run').onclick=()=>run(async()=>{let x=await api('diagnostics/analyze',{bag:$('diag-bag').value});$('diag-result').textContent=pretty(x);toast('诊断任务已启动，可在 Runs 查看')});
let diagnosis=null;
$('diag-open').onclick=()=>run(async()=>{diagnosis=await api('reports/read',{path:$('diag-path').value});$('diag-result').textContent=pretty(diagnosis.metrics||diagnosis);drawDiagnostic()});
$('diag-signal').onchange=drawDiagnostic;
function drawDiagnostic(){let c=$('diag-chart'),ctx=c.getContext('2d');ctx.fillStyle='#102325';ctx.fillRect(0,0,c.width,c.height);if(!diagnosis)return;let k=$('diag-signal').value,series=Object.entries(diagnosis.plot_series||diagnosis.series||{}).filter(([_,r])=>r.some(p=>Number.isFinite(p[k])));if(!series.length)return;let all=series.flatMap(([_,r])=>r.filter(p=>Number.isFinite(p[k]))),t0=Math.min(...all.map(p=>p.t)),t1=Math.max(...all.map(p=>p.t)),ys=all.map(p=>p[k]),lo=Math.min(...ys),hi=Math.max(...ys);ctx.fillStyle='white';ctx.fillText(`${k} · ${t0.toFixed(2)} — ${t1.toFixed(2)} s`,10,18);series.forEach(([name,rows],i)=>{ctx.strokeStyle=['#e5b35a','#58dcca','#e47ab5','#788fff'][i%4];ctx.beginPath();rows.filter(p=>Number.isFinite(p[k])).forEach((p,j)=>{let x=40+(p.t-t0)/(t1-t0||1)*(c.width-60),y=c.height-30-(p[k]-lo)/(hi-lo||1)*(c.height-80);j?ctx.lineTo(x,y):ctx.moveTo(x,y)});ctx.stroke();ctx.fillStyle=ctx.strokeStyle;ctx.fillText(name,10,35+i*14)})}
$('acceptance-run').onclick=()=>run(async()=>{$('acceptance-result').textContent=pretty(await api('reports/acceptance',JSON.parse($('acceptance-input').value)))});
run(fieldRefresh);run(liveRefresh);setInterval(()=>run(liveRefresh),1200);

$('benchmark-run').onclick=()=>run(async()=>toast(pretty(await api('experiments/benchmark',{spec:JSON.parse($('benchmark-spec').value)}))));
$('ab-run').onclick=()=>run(async()=>toast(pretty(await api('experiments/tracker_ab',{robot,map_id:$('route-map').value,bag:$('ab-bag').value,calibration:$('ab-calibration').value}))));

$('ins-start').onclick=()=>run(async()=>toast(pretty(await api('ins/start',{port:$('ins-port').value,baudrate:Number($('ins-baud').value)}))));

$('profile-save').onclick=()=>run(async()=>{await api('profiles/'+robot,{profile:JSON.parse($('profile-editor').value)});await fieldRefresh();toast('新配置已选用，原文件保留；运动须重新检查')});
