"""Standalone skeleton comparison; no external fonts, media or JavaScript libraries."""
import json
import numpy as np
from scipy.spatial.transform import Rotation

def write_preview(path,b,before,after,before_g,after_g):
    frames=np.arange(0,b.frame_count,4)
    if frames[-1]!=b.frame_count-1:
        frames=np.r_[frames,b.frame_count-1]
    data={'frames':frames.tolist(),'dt':b.dt,'parents':[j['parent'] for j in b.joints],
          'names':[j['name'] for j in b.joints],
          'poses':[np.round(before[frames],3).tolist(),np.round(after[frames],3).tolist()]}
    joints=[i for i,j in enumerate(b.joints) if j['name'].endswith(('ForeArm','Hand'))]
    data['markers']=[]
    for p,g in [(before,before_g),(after,after_g)]:
        markers=[]
        for i in joints:
            end=p[frames,i]+Rotation.from_quat(g[frames,i]).apply(np.tile([0,7,0],(len(frames),1)))
            markers.append({'joint':i,'ends':np.round(end,3).tolist()})
        data['markers'].append(markers)
    payload=json.dumps(data,allow_nan=False).replace('<','\\u003c').replace('>','\\u003e').replace('&','\\u0026')
    html='''<!doctype html><html lang="ja"><meta charset="utf-8"><title>モーションの修正前後比較</title>
<style>body{background:#121923;color:#eee;font:16px sans-serif;margin:24px}canvas{width:100%;max-width:1200px;background:#192331}input{width:60%}button,select{padding:8px}p{max-width:1100px}</style>
<h1>修正前 / 修正後</h1><p>青：体の左側 ／ オレンジ：右側 ／ 灰色：中央。前腕と手の横線・点は回転方向の目印です。比較表示は約30fpsです。出力BVHは元のフレーム数を維持しています。</p>
<button id="play">再生</button> <select id="speed"><option value="1">1×</option><option value="0.25">0.25×</option></select>
<select id="view"><option value="0">正面</option><option value="2">側面</option></select>
<input id="seek" type="range" min="0" value="0"><span id="time"></span><br><canvas id="canvas" width="1200" height="650"></canvas>
<script>const D=PAYLOAD;const c=document.getElementById('canvas'),ctx=c.getContext('2d'),seek=document.getElementById('seek'),play=document.getElementById('play'),view=document.getElementById('view'),speed=document.getElementById('speed'),label=document.getElementById('time');
seek.max=D.frames.length-1;let active=false,previous=null,elapsed=0;
let lo=[Infinity,Infinity,Infinity],hi=[-Infinity,-Infinity,-Infinity];for(const poses of D.poses)for(const pose of poses)for(const p of pose)for(let a=0;a<3;a++){lo[a]=Math.min(lo[a],p[a]);hi[a]=Math.max(hi[a],p[a]);}
function draw(){let f=+seek.value,axis=+view.value;ctx.clearRect(0,0,c.width,c.height);let scale=Math.min(500/Math.max(1,hi[axis]-lo[axis]),540/Math.max(1,hi[1]-lo[1]));
for(let panel=0;panel<2;panel++){ctx.fillStyle='#eee';ctx.font='22px sans-serif';ctx.fillText(panel?'修正後':'修正前',panel*600+24,35);let pose=D.poses[panel][f];function pt(p){return [panel*600+300+(p[axis]-(lo[axis]+hi[axis])/2)*scale,610-(p[1]-lo[1])*scale];}
for(let j=0;j<pose.length;j++){let p=D.parents[j];if(p<0)continue;ctx.strokeStyle=D.names[j].startsWith('Left')?'#59b8ff':D.names[j].startsWith('Right')?'#ffa45e':'#ddd';ctx.lineWidth=3;let a=pt(pose[p]),b=pt(pose[j]);ctx.beginPath();ctx.moveTo(...a);ctx.lineTo(...b);ctx.stroke();}
for(const m of D.markers[panel]){let a=pt(pose[m.joint]),b=pt(m.ends[f]);ctx.strokeStyle=D.names[m.joint].startsWith('Left')?'#59b8ff':'#ffa45e';ctx.fillStyle=ctx.strokeStyle;ctx.lineWidth=1;ctx.beginPath();ctx.moveTo(...a);ctx.lineTo(...b);ctx.stroke();ctx.beginPath();ctx.arc(...b,3,0,Math.PI*2);ctx.fill();}}
label.textContent=(D.frames[f]*D.dt).toFixed(3)+' s / F'+D.frames[f];}
play.onclick=()=>{if(!active&&+seek.value===D.frames.length-1){seek.value=0;draw();}active=!active;play.textContent=active?'一時停止':'再生';previous=null;elapsed=D.frames[+seek.value]*D.dt;};seek.oninput=()=>{elapsed=D.frames[+seek.value]*D.dt;previous=null;draw();};view.onchange=draw;
function tick(t){if(active){if(previous!==null)elapsed+=(t-previous)/1000*(+speed.value);previous=t;let f=+seek.value;while(f<D.frames.length-1&&D.frames[f+1]*D.dt<=elapsed)f++;seek.value=f;if(f===D.frames.length-1){active=false;play.textContent='再生';}draw();}requestAnimationFrame(tick);}draw();requestAnimationFrame(tick);</script></html>'''
    path.write_text(html.replace('PAYLOAD',payload),encoding='utf-8')
