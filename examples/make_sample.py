"""Generate an artificial motion and corrupted copy. No captured motion or models."""
import argparse
from pathlib import Path
import numpy as np

def generate(output):
    output=Path(output)
    output.mkdir(parents=True,exist_ok=False)
    nodes=[('Hips',-1,(0,0,0)),('Spine',0,(0,18,0)),('Spine1',1,(0,18,0)),
        ('Neck',2,(0,12,0)),('Head',3,(0,10,0))]
    for side,sign in [('Left',1),('Right',-1)]:
        start=len(nodes)
        nodes.extend([(side+'Shoulder',2,(sign*8,8,0)),(side+'Arm',start,(sign*10,0,0)),
            (side+'ForeArm',start+1,(sign*28,0,0)),(side+'Hand',start+2,(sign*24,0,0))])
        start=len(nodes)
        nodes.extend([(side+'UpLeg',0,(sign*9,-3,0)),(side+'Leg',start,(0,-40,0)),
            (side+'Foot',start+1,(0,-40,0)),(side+'ToeBase',start+2,(0,-5,12))])
    lines=[]; order=[]
    def emit(i,depth):
        name,parent,offset=nodes[i];pad='  '*depth
        lines.extend([pad+('ROOT ' if parent<0 else 'JOINT ')+name,pad+'{',pad+'  OFFSET '+' '.join(map(str,offset)),
            pad+'  CHANNELS '+('6 Xposition Yposition Zposition ' if parent<0 else '3 ')+'Xrotation Yrotation Zrotation'])
        order.append(i)
        children=[k for k,node in enumerate(nodes) if node[1]==i]
        for k in children:emit(k,depth+1)
        if not children:lines.extend([pad+'  End Site',pad+'  {',pad+'    OFFSET 0 3 0',pad+'  }'])
        lines.append(pad+'}')
    emit(0,0)
    n=480;t=np.arange(n)/120
    data=np.zeros((n,3+3*len(nodes)));data[:,1]=90+.3*np.sin(t*2)
    channel={nodes[i][0]:3+3*k for k,i in enumerate(order)}
    for name,idx in channel.items():
        data[:,idx]=2*np.sin(t*1.7+.13*idx)
        data[:,idx+2]=1*np.sin(t*1.2+.1*idx)
        if name.endswith('Leg') and not name.endswith('UpLeg'):data[:,idx]+=15
    damaged=data.copy()
    damaged[180:210,channel['LeftForeArm']]+=180
    damaged[180:210,channel['LeftHand']]-=180
    damaged[300,channel['Head']+2]+=100
    i=channel['RightArm'];damaged[350:375,i:i+3]=damaged[349,i:i+3]
    header='\n'.join(lines)+'\nMOTION\nFrames: 480\nFrame Time: 0.008333333333333333\n'
    for name,values in [('reference.bvh',data),('damaged.bvh',damaged)]:
        with (output/name).open('x',encoding='ascii') as f:
            f.write(header);np.savetxt(f,values,fmt='%.9f')
    return output

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True)
    generate(parser.parse_args().output)
