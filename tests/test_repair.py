import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np
from scipy.spatial.transform import Rotation as R
from motion_repair.cli import run, PROFILE, load_config
from motion_repair.engine import BVH, prerepair

spec=importlib.util.spec_from_file_location('sample',Path(__file__).parents[1]/'examples/make_sample.py')
sample=importlib.util.module_from_spec(spec);spec.loader.exec_module(sample)

class RepairTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.root=Path(self.temp.name);sample.generate(self.root/'input')
        self.source=self.root/'input/damaged.bvh'

    def test_end_to_end(self):
        digest=hashlib.sha256(self.source.read_bytes()).hexdigest()
        out=run(self.source,self.root/'result',PROFILE)
        a=BVH(self.source);b=BVH(out/'repaired.bvh');ref=BVH(self.root/'input/reference.bvh')
        self.assertEqual(digest,hashlib.sha256(self.source.read_bytes()).hexdigest())
        self.assertEqual(a.header,b.header);self.assertEqual(a.dt_text,b.dt_text)
        self.assertEqual(a.data.shape,b.data.shape)
        _,ag=a.fk();_,bg=b.fk();_,rg=ref.fk()
        head=a.names['Head'];fore=a.names['LeftForeArm']
        before=np.rad2deg((R.from_quat(ag[299,head]).inv()*R.from_quat(ag[300,head])).magnitude())
        after=np.rad2deg((R.from_quat(bg[299,head]).inv()*R.from_quat(bg[300,head])).magnitude())
        self.assertLess(after,before*.2)
        old=(R.from_quat(rg[185:205,fore]).inv()*R.from_quat(ag[185:205,fore])).magnitude().mean()
        new=(R.from_quat(rg[185:205,fore]).inv()*R.from_quat(bg[185:205,fore])).magnitude().mean()
        self.assertLess(new,old*.2)
        self.assertTrue(json.loads((out/'verification.json').read_text())['source_unchanged'])
        self.assertTrue((out/'review.html').is_file())
        with self.assertRaises(ValueError):run(self.source,out,PROFILE)
        with self.assertRaises(ValueError):run(self.source,self.source,PROFILE)

    def test_analyze_has_no_repaired_motion(self):
        out=run(self.source,self.root/'analysis',PROFILE,analyze_only=True)
        self.assertFalse((out/'repaired.bvh').exists())
        self.assertIn('RightArm',json.loads((out/'diagnostics.json').read_text())['frozen_candidates'])

    def test_reject_unsupported_fps(self):
        bad=self.root/'30fps.bvh'
        bad.write_text(self.source.read_text().replace('0.008333333333333333','0.03333333333333333'))
        with self.assertRaises(ValueError):run(bad,self.root/'result',PROFILE)
        self.assertFalse((self.root/'result').exists())

    def test_reject_bad_axis_and_nonfinite(self):
        for k,(old,new) in enumerate([('OFFSET 28 0 0','OFFSET 0 28 0'),('OFFSET 28 0 0','OFFSET nan 0 0')]):
            path=self.root/f'bad{k}.bvh';path.write_text(self.source.read_text().replace(old,new))
            with self.assertRaises(ValueError):run(path,self.root/f'result{k}',PROFILE)

    def test_terminal_freeze_does_not_index_past_end(self):
        b=BVH(self.source);b.data[-30:]=b.data[-31]
        b.local=b.rotations(b.data);_,g=b.fk()
        prerepair(b,g.copy(),g,load_config(None,b.frame_count))

    def test_bad_config(self):
        for value in [{'glitch_frames':[-1]},{'pelvis_snap_frames':[2]},{'manual_bridges':[['Left',40,20]]},{'unknown':1}]:
            p=self.root/'bad.json';p.write_text(json.dumps(value))
            with self.assertRaises(ValueError):load_config(p,480)

if __name__=='__main__':unittest.main()
