"""Validated, non-overwriting command-line entry point."""
import argparse
import contextlib
import hashlib
import json
import sys
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from . import engine
from .preview import write_preview

PROFILE = 'humanoid-xarm-yup-cm-120fps'

def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def write_json(path, value):
    Path(path).write_text(json.dumps(value, indent=2, allow_nan=False), encoding='utf-8')

def validate(b):
    if not np.isfinite(b.dt) or abs(1 / b.dt - 120) > .02:
        raise ValueError('Only approximately 120 fps is supported; no automatic resampling.')
    if b.frame_count < 120:
        raise ValueError('At least 120 frames are required.')
    if len(b.names) != len(b.joints):
        raise ValueError('Duplicate joint names are unsupported.')
    if b.joints[0]['name'] != 'Hips' or b.joints[0]['parent'] != -1:
        raise ValueError('Root must be Hips.')
    if b.joints[0]['channels'][:3] != ['Xposition', 'Yposition', 'Zposition']:
        raise ValueError('Root position channels must start with Xposition Yposition Zposition.')
    for i,j in enumerate(b.joints):
        if not np.isfinite(j['offset']).all():
            raise ValueError('Nonfinite joint offset.')
        if i and not 0 <= j['parent'] < i:
            raise ValueError('Unsupported hierarchy.')
        channels = j['channels'][3:] if i == 0 else j['channels']
        if channels and (len(channels) != 3 or set(channels) != {'Xrotation','Yrotation','Zrotation'}):
            raise ValueError('Only three distinct Euler rotation channels per joint are supported.')
    required = ['Head'] + [side+s for side in ['Left','Right'] for s in ['Arm','ForeArm','Hand','UpLeg','Leg','Foot','ToeBase']]
    if any(n not in b.names for n in required):
        raise ValueError('Missing required bones: ' + ', '.join(n for n in required if n not in b.names))
    for name in required:
        if len(b.joints[b.names[name]]['channels']) != 3:
            raise ValueError('Required bones must have three rotation channels: '+name)
    for side in ['Left','Right']:
        for parent, child in [('Arm','ForeArm'),('ForeArm','Hand'),('UpLeg','Leg'),('Leg','Foot'),('Foot','ToeBase')]:
            c = b.joints[b.names[side+child]]
            if c['parent'] != b.names[side+parent]:
                raise ValueError('Unsupported limb hierarchy: '+side+child)
            if np.linalg.norm(c['offset']) < .1:
                raise ValueError('Degenerate limb length.')
        for child in ['ForeArm','Hand']:
            offset = np.array(b.joints[b.names[side+child]]['offset'])
            sign = 1 if side == 'Left' else -1
            if sign*offset[0] <= 0 or np.linalg.norm(offset[1:]) > abs(offset[0])*.001:
                raise ValueError('Arms must extend along local +X (Left) / -X (Right).')
        leg = np.linalg.norm(b.joints[b.names[side+'Leg']]['offset'])
        if not 10 <= leg <= 100:
            raise ValueError('Leg scale outside centimeter humanoid profile (10–100 cm).')

def load_config(path, n):
    default = json.loads(Path(__file__).with_name('profile.json').read_text())
    if path:
        user = json.loads(Path(path).read_text(encoding='utf-8-sig'))
        if not isinstance(user,dict) or set(user)-set(default):
            raise ValueError('Unknown configuration keys.')
        default.update(user)
    if default['profile'] != PROFILE:
        raise ValueError('Unsupported profile.')
    for key in ['pelvis_snap_frames','glitch_frames']:
        values = default[key]
        low = 20 if key == 'pelvis_snap_frames' else 1
        if not isinstance(values,list) or any(type(f) is not int or not low <= f < n-1 for f in values):
            raise ValueError('Invalid zero-based frame in '+key)
        if len(set(values)) != len(values):
            raise ValueError('Duplicate frames in '+key)
    for key in ['manual_bridges','arm_chain_bridges']:
        if not isinstance(default[key],list):
            raise ValueError('Expected list: '+key)
        for row in default[key]:
            if not isinstance(row,list) or len(row)!=3 or row[0] not in ['Left','Right'] or any(type(f) is not int for f in row[1:]) or not 1 <= row[1] < row[2] < n:
                raise ValueError('Expected [side, first_bad, first_good] in '+key)
    for key,lo,hi in [('pole_sigma_samples',.1,30),('knee_straighten_degrees',0,20)]:
        v=default[key]
        if type(v) not in [int,float] or not np.isfinite(v) or not lo<=v<=hi:
            raise ValueError('Invalid setting: '+key)
    return default

def diagnostics(b,p,g,sha):
    steps = np.rad2deg((Rotation.from_quat(g[:-1,0]).inv()*Rotation.from_quat(g[1:,0])).magnitude())
    distance = np.linalg.norm(np.diff(b.data[:,:3],axis=0),axis=1)
    return {'sha256':sha, 'frames':b.frame_count,'frame_time':b.dt_text,
        'pelvis_snap_candidates':[int(f) for f in np.flatnonzero((steps>12)&(distance>3*np.median(distance)))],
        'pelvis_frame_convention':'f denotes transition f -> f+1; review before adding to config',
        'frozen_candidates':{j['name']:[[int(f) for f in v] for v in engine.frozen_runs(b,i)] for i,j in enumerate(b.joints) if j['channels']},
        'status':'candidates_only_not_proof_of_tracking_error'}

def run(source, output, profile, config_path=None, analyze_only=False, progress=None):
    progress = progress or (lambda message: None)
    progress('ファイルを読み込んでいます')
    source = Path(source).resolve(strict=True)
    output = Path(output).resolve()
    if profile != PROFILE:
        raise ValueError('Explicit supported profile selection is required.')
    if output.exists():
        raise ValueError('Output directory already exists. Choose a new directory.')
    original_sha = digest(source)
    b=engine.BVH(source)
    validate(b)
    config=load_config(config_path,b.frame_count)
    progress('動きと異常の候補を調べています')
    p,g=b.fk()
    if not np.isfinite(p).all():
        raise ValueError('Nonfinite forward kinematics.')
    output.mkdir(parents=True,exist_ok=False)
    try:
        write_json(output/'diagnostics.json',diagnostics(b,p,g,original_sha))
        write_json(output/'settings.json',config)
        if analyze_only:
            write_preview(output/'review.html',b,p,p,g,g)
        else:
            np.savez_compressed(output/'source_cache.npz',positions=p,global_quat=g)
            progress('飛び・ねじれ・接地を補修しています')
            with (output/'processing.log').open('w',encoding='utf-8') as log, contextlib.redirect_stdout(log):
                engine.clean(source,output,config)
            progress('保存したモーションを検証しています')
            after=engine.BVH(output/'repaired.bvh')
            ap,ag=after.fk()
            if after.header!=b.header or after.dt_text!=b.dt_text or after.data.shape!=b.data.shape or not np.isfinite(ap).all():
                raise ValueError('Written BVH failed structure/time/finite verification.')
            progress('比較画面を作っています')
            write_preview(output/'review.html',b,p,ap,g,ag)
        if digest(source)!=original_sha:
            raise ValueError('Source changed during processing.')
        write_json(output/'verification.json',{'status':'passed','source_unchanged':True,
            'source_sha256':original_sha,'mode':'analyze' if analyze_only else 'repair',
            'output_sha256':None if analyze_only else digest(output/'repaired.bvh'),
            'checks':['structure','frame_time','finite_positions','source_sha256'],
            'visual_acceptance':'required; numeric checks do not establish choreography correctness'})
    except Exception as error:
        write_json(output/'FAILED.json',{'status':'failed','error':str(error),'instruction':'Do not use partial output.'})
        raise
    return output

def main(argv=None):
    parser=argparse.ArgumentParser(description='Experimental 120 fps BVH repair. Always writes a new directory.')
    parser.add_argument('source',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--profile',choices=[PROFILE],required=True,help='Affirms Y-up, centimeter units and supported local arm axes.')
    parser.add_argument('--config',type=Path)
    parser.add_argument('--analyze-only',action='store_true')
    args=parser.parse_args(argv)
    try:
        out=run(args.source,args.output,args.profile,args.config,args.analyze_only)
    except Exception as error:
        print(f'Error: {error}',file=sys.stderr)
        return 1
    print(f'Completed: {out}\nOpen review.html and inspect before using the result.')
    return 0
