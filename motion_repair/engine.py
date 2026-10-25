"""Production-derived experimental BVH repair engine. See docs/ALGORITHM.md."""
from pathlib import Path
import sys, re, json, hashlib, warnings
import numpy as np
from scipy.spatial.transform import Rotation as R, Slerp
from scipy.ndimage import gaussian_filter1d, median_filter, binary_closing, binary_opening, maximum_filter1d
from scipy.signal import savgol_filter, find_peaks

def reach_cap(l1, l2, distance, tol=None):
    tol = 8.0 if tol is None else tol
    cos = np.clip((l1 * l1 + l2 * l2 - distance ** 2) / (2 * l1 * l2), -1, 1)
    angle = np.minimum(np.arccos(cos) + np.deg2rad(tol), np.pi)
    return np.minimum(np.sqrt(np.maximum(l1 * l1 + l2 * l2 - 2 * l1 * l2 * np.cos(angle), 0)), l1 + l2 - 0.01)

class BVH:

    def __init__(self, path):
        text = Path(path).read_text(encoding='utf-8-sig')
        self.header, motion = text.split('MOTION', 1)
        mt = re.fullmatch(r'\s*Frames:\s*(\d+)\s*Frame Time:\s*([^\s]+)\s*(.*)', motion, re.S)
        if not mt:
            raise ValueError('Invalid BVH motion header.')
        self.frame_count = int(mt[1])
        self.dt = float(mt[2])
        self.dt_text = mt[2]
        if self.frame_count < 1 or not np.isfinite(self.dt) or self.dt <= 0:
            raise ValueError('Invalid BVH frame count or frame time.')
        self.data = np.array(mt[3].split(), dtype=float).reshape(self.frame_count, -1)
        self.joints = []
        stack = []
        pending = None
        channel = 0
        for line in self.header.splitlines():
            tok = line.strip().split()
            if not tok:
                continue
            if tok[0] in ('ROOT', 'JOINT') or tok[:2] == ['End', 'Site']:
                name = tok[1] if tok[0] != 'End' else self.joints[stack[-1]]['name'] + '_End'
                pending = len(self.joints)
                self.joints.append(dict(name=name, parent=stack[-1] if stack else -1, offset=[0.0, 0.0, 0.0], channels=[], indices=[]))
            elif tok[0] == '{':
                stack.append(pending)
            elif tok[0] == '}':
                stack.pop()
            elif tok[0] == 'OFFSET':
                self.joints[stack[-1]]['offset'] = list(map(float, tok[1:]))
            elif tok[0] == 'CHANNELS':
                n = int(tok[1])
                j = self.joints[stack[-1]]
                j['channels'] = tok[2:]
                j['indices'] = list(range(channel, channel + n))
                channel += n
        if not (channel == self.data.shape[1] and np.isfinite(self.data).all()):
            raise ValueError('Internal verification failed')
        self.names = {j['name']: i for i, j in enumerate(self.joints)}
        self.local = self.rotations(self.data)

    def rotations(self, data):
        result = []
        for j in self.joints:
            idx = [i for i, c in zip(j['indices'], j['channels']) if c.endswith('rotation')]
            order = ''.join((c[0] for c in j['channels'] if c.endswith('rotation')))
            result.append(R.from_euler(order, data[:, idx], degrees=True).as_quat() if idx else np.tile([0, 0, 0, 1.0], (len(data), 1)))
        return np.stack(result, axis=1)

    def fk(self, local=None, root=None):
        local = self.local if local is None else local
        root = self.data[:, :3] if root is None else root
        n = len(root)
        p = np.zeros((n, len(self.joints), 3))
        g = np.zeros((n, len(self.joints), 4))
        for i, j in enumerate(self.joints):
            if j['parent'] < 0:
                p[:, i] = root + np.array(j['offset'])
                g[:, i] = local[:, i]
            else:
                parent = j['parent']
                pr = R.from_quat(g[:, parent])
                p[:, i] = p[:, parent] + pr.apply(np.broadcast_to(j['offset'], (n, 3)))
                g[:, i] = (pr * R.from_quat(local[:, i])).as_quat()
        return (p, g)

    def write(self, path, data):
        with Path(path).open('w', encoding='ascii', newline='\n') as f:
            f.write(self.header + 'MOTION\nFrames:    ' + str(len(data)) + '\nFrame Time:    ' + self.dt_text + '\n')
            np.savetxt(f, data, fmt='%.6f', delimiter='    ')

def hold_pole(vec, valid, seed):
    out = vec.copy()
    idx = np.where(valid, np.arange(len(vec)), -1)
    idx = np.maximum.accumulate(idx)
    first = np.flatnonzero(valid)
    out[idx < 0] = seed[idx < 0]
    fill = ~valid & (idx >= 0)
    out[fill] = vec[idx[fill]]
    if not (len(first) or not valid.any()):
        raise ValueError('Internal verification failed')
    return out

def runs(mask):
    edges = np.diff(np.r_[False, mask, False].astype(int))
    return list(zip(np.where(edges == 1)[0], np.where(edges == -1)[0]))

def stats(v):
    return dict(zip(['min', 'p01', 'p50', 'p95', 'p99', 'max'], np.percentile(v, [0, 1, 50, 95, 99, 100]).round(4).tolist()))

def continuous(q):
    q = q.copy()
    flips = np.r_[1, np.cumprod(np.where(np.sum(q[1:] * q[:-1], axis=1) < 0, -1, 1))]
    return q * flips[:, None]

def qsmooth(q, sigma):
    q = continuous(q)
    sm = gaussian_filter1d(q, sigma, axis=0, mode='nearest', truncate=3)
    return sm / np.linalg.norm(sm, axis=1, keepdims=True)

def strong_smooth(q, limit, iterations=6):
    q = q.copy()
    for _ in range(iterations):
        r = R.from_quat(q)
        step = (r[:-1].inv() * r[1:]).magnitude() * 180 / np.pi
        if step.max() <= limit:
            break
        bad = np.r_[step > limit, False] | np.r_[False, step > limit]
        weight = gaussian_filter1d(maximum_filter1d(bad.astype(float), size=37), 5)
        delta = (r.inv() * R.from_quat(qsmooth(q, 5))).as_rotvec()
        q = (r * R.from_rotvec(delta * weight[:, None])).as_quat()
    return q

def between(a, b):
    a = a / np.linalg.norm(a, axis=1, keepdims=True)
    b = b / np.linalg.norm(b, axis=1, keepdims=True)
    q = np.c_[np.cross(a, b), 1 + np.sum(a * b, axis=1)]
    bad = np.linalg.norm(q, axis=1) < 1e-08
    if np.any(bad):
        ort = np.cross(a[bad], np.array([1.0, 0, 0]))
        zero = np.linalg.norm(ort, axis=1) < 1e-05
        ort[zero] = np.cross(a[bad][zero], np.array([0.0, 0, 1.0]))
        q[bad, :3] = ort
        q[bad, 3] = 0
    return R.from_quat(q)

def frozen_runs(b, i, seconds=0.1):
    j = b.joints[i]
    idx = [k for k, c in zip(j['indices'], j['channels']) if c.endswith('rotation')]
    if not idx:
        return []
    n = b.frame_count
    same = np.abs(np.diff(b.data[:, idx], axis=0)).sum(1) == 0
    return [(a + 1, z + 1) for a, z in runs(same) if (z - a) * b.dt >= seconds and a + 1 >= 1 and (z + 1 <= n - 1) and (z - a <= 0.5 * n)]

def bridge_joint(g, i, lo, hi, parent=None):
    if parent is None or parent < 0:
        g[lo:hi, i] = Slerp([lo - 1, hi], R.from_quat(g[[lo - 1, hi], i]))(np.arange(lo, hi)).as_quat()
        return
    P = R.from_quat(g[lo - 1:hi + 1, parent])
    st = np.rad2deg((P[:-1].inv() * P[1:]).magnitude())
    k = np.r_[0, np.cumsum(st > 120)] % 2
    flip = R.from_euler('X', 180, degrees=True)
    P = R.from_quat(np.where(k[:, None] == 1, (P * flip).as_quat(), P.as_quat()))
    rel = P[[0, -1]].inv() * R.from_quat(g[[lo - 1, hi], i])
    g[lo:hi, i] = (P[1:-1] * Slerp([0, 1], rel)((np.arange(lo, hi) - (lo - 1)) / (hi - (lo - 1)))).as_quat()

def frozen_record(b, name, lo, hi):
    return {'type': 'joint_frozen_hold', 'joint': name, 'frames': [int(lo), int(hi)], 'seconds': [round(lo * b.dt, 4), round(hi * b.dt, 4)], 'joints': name + ' rotation in its parent bone frame (roll-free)'}

def prerepair(b, g, rawg, config):
    n = len(g)
    root = b.data[:, :3]
    repairs = []

    def bridge(lo, hi, joints):
        t = (np.arange(lo, hi) - (lo - 1)) / (hi - (lo - 1))
        root[lo:hi] = root[lo - 1] + t[:, None] * (root[hi] - root[lo - 1])
        for i in joints:
            g[lo:hi, i] = Slerp([lo - 1, hi], R.from_quat(g[[lo - 1, hi], i]))(np.arange(lo, hi)).as_quat()
    same = np.abs(np.diff(b.data, axis=0)).sum(1) == 0
    for a, z in runs(same):
        if z - a < 3 or z + 1 >= n:
            continue
        lo, hi = (a + 1, z + 1)
        bridge(lo, hi, range(g.shape[1]))
        repairs.append({'type': 'frozen_hold', 'frames': [int(lo), int(hi)], 'seconds': [round(lo * b.dt, 4), round(hi * b.dt, 4)], 'joints': 'all'})
    for f in config['glitch_frames']:
        bridge(f, f + 1, range(g.shape[1]))
        repairs.append({'type': 'single_frame_glitch', 'frames': [int(f), int(f) + 1], 'seconds': [round(f * b.dt, 4), round((f + 1) * b.dt, 4)], 'joints': 'all'})
    hs = (R.from_quat(rawg[:-1, 0]).inv() * R.from_quat(rawg[1:, 0])).magnitude() * 180 / np.pi
    rs = np.linalg.norm(np.diff(b.data[:, :3], axis=0), axis=1)
    snaps = [int(f) for f in np.flatnonzero((hs > 12) & (rs > 3 * np.median(rs))) if not any((lo <= f < hi for r in repairs for lo, hi in [r['frames']])) and (not any((abs(f - gf) <= 1 for gf in config['glitch_frames'])))]
    detected_snaps = snaps
    snaps = config['pelvis_snap_frames']
    if any((f not in detected_snaps for f in snaps)):
        raise ValueError('Configured pelvis snap does not match detector')
    flip = R.from_euler('X', 180, degrees=True)
    for side in ['Left', 'Right']:
        i = b.names[side + 'Arm']
        Q = R.from_quat(g[:, i])
        st = np.rad2deg((Q[:-1].inv() * Q[1:]).magnitude())
        ev = [int(f) + 1 for f in np.flatnonzero(st > 120)]
        k = 0
        while k + 1 < len(ev):
            lo, hi = (ev[k], ev[k + 1])
            if (hi - lo) * b.dt <= 0.6:
                inner = R.from_quat(g[lo:hi, i]) * flip
                a = np.rad2deg((R.from_quat(g[lo - 1, i]).inv() * inner[0]).magnitude())
                z = np.rad2deg((inner[-1].inv() * R.from_quat(g[hi, i])).magnitude())
                if a < 60 and z < 60:
                    g[lo:hi, i] = inner.as_quat()
                    repairs.append({'type': 'upper_arm_fold', 'joint': side + 'Arm', 'frames': [lo, hi], 'seconds': [round(lo * b.dt, 4), round(hi * b.dt, 4)], 'boundary_steps_deg': [round(float(st[lo - 1]), 1), round(float(st[hi - 1]), 1)], 'after_flip_deg': [round(float(a), 1), round(float(z), 1)]})
                    k += 2
                    continue
            k += 1
    for i, j in enumerate(b.joints):
        if 'ForeArm' in j['name'] or 'Hand' in j['name']:
            continue
        for lo, hi in frozen_runs(b, i):
            if any((r['type'] == 'frozen_hold' and r['frames'][0] <= lo < r['frames'][1] for r in repairs)):
                continue
            bridge_joint(g, i, lo, hi, j['parent'])
            repairs.append(frozen_record(b, j['name'], lo, hi))
    for f in snaps:
        lo, hi = (f - 19, f + 1)
        bridge(lo, hi, [0])
        repairs.append({'type': 'pelvis_snap', 'snap_frame': f, 'frames': [int(lo), int(hi)], 'seconds': [round(lo * b.dt, 4), round(hi * b.dt, 4)], 'joints': 'Hips world rotation and root position; descendants keep world rotation'})
    return repairs

def clean(source, output_dir, config):
    SOURCE = Path(source)
    BASE = Path(output_dir)
    if not not (BASE / 'repaired.bvh').exists():
        raise ValueError('Preserve previous candidate')
    provenance = json.loads((BASE / 'diagnostics.json').read_text(encoding='utf-8'))
    if not provenance['sha256'] == hashlib.sha256(SOURCE.read_bytes()).hexdigest():
        raise ValueError('Internal verification failed')
    b = BVH(SOURCE)
    cache = np.load(BASE / 'source_cache.npz')
    orig = cache['positions']
    rawg = cache['global_quat']
    g = rawg.copy()
    n = len(g)
    report = {'source_sha256': hashlib.sha256(SOURCE.read_bytes()).hexdigest(), 'frames': n, 'frame_time': b.dt_text, 'units': 'centimeters; explicitly selected profile', 'twist_repairs': {}, 'spike_repairs': {}, 'contacts': {}}
    report['dropout_repairs'] = prerepair(b, g, rawg, config)
    print('DROPOUT', json.dumps(report['dropout_repairs']))
    flip = R.from_euler('X', 180, degrees=True)

    def qang(u, v):
        return 2 * np.arccos(np.clip(np.abs(np.sum(u * v, axis=-1)), 0, 1))

    def wrap(a):
        return (a + 180) % 360 - 180

    def twist_of(rel):
        q = continuous(rel.as_quat())
        return wrap(np.rad2deg(np.arctan2(q[:, 0], q[:, 3]) * 2))

    def dilate(mask, pad):
        return maximum_filter1d(mask.astype(int), size=2 * pad + 1) > 0
    report['glitch_bridges'] = {}
    for side in ['Left', 'Right']:
        i = b.names[side + 'ForeArm']
        hj = b.names[side + 'Hand']
        ai = b.names[side + 'Arm']
        q = g[:, i].copy()
        alt = (R.from_quat(q) * flip).as_quat()
        cand = np.stack([q, alt])
        tw = twist_of(R.from_quat(g[:, ai]).inv() * R.from_quat(q))
        centre = float(np.median(tw))
        dev = wrap(np.stack([tw, tw + 180]) - centre)
        penalty = np.deg2rad(np.maximum(0, np.abs(dev) - 80)) * 0.02
        hrel = np.stack([(R.from_quat(cand[s]).inv() * R.from_quat(g[:, hj])).as_quat() for s in range(2)])
        htw = twist_of(R.from_quat(q).inv() * R.from_quat(g[:, hj]))
        hpen = np.deg2rad(np.maximum(0, np.abs(wrap(np.stack([htw, htw + 180]))) - 100)) * 0.02
        hmag = np.rad2deg(2 * np.arccos(np.clip(np.abs(hrel[:, :, 3]), 0, 1)))
        hpen = hpen + np.deg2rad(np.maximum(0, hmag - 110)) * 0.02
        penalty = penalty + hpen
        cost = np.zeros((2, n))
        back = np.zeros((2, n), int)
        cost[:, 0] = penalty[:, 0]
        cost[1, 0] += np.deg2rad(5)
        for f in range(1, n):
            step = np.array([[qang(cand[p, f - 1], cand[s, f]) + qang(hrel[p, f - 1], hrel[s, f]) for s in range(2)] for p in range(2)])
            total = cost[:, f - 1][:, None] + step + penalty[:, f][None, :]
            back[:, f] = np.argmin(total, axis=0)
            cost[:, f] = np.min(total, axis=0)
        state = np.zeros(n, int)
        state[-1] = int(np.argmin(cost[:, -1]))
        for f in range(n - 1, 0, -1):
            state[f - 1] = back[state[f], f]
        events = [int(f) for f in np.flatnonzero(np.diff(state) != 0) + 1]
        g[:, i] = np.where(state[:, None] == 1, alt, q)
        for jname in [side + 'ForeArm', side + 'Hand']:
            ji = b.names[jname]
            for lo, hi in frozen_runs(b, ji):
                bridge_joint(g, ji, lo, hi, b.joints[ji]['parent'])
                report['dropout_repairs'].append(frozen_record(b, jname, lo, hi))
        for bside, lo, hi in config['arm_chain_bridges']:
            if bside != side:
                continue
            for ji in [ai, i, hj]:
                bridge_joint(g, ji, lo, hi)
            report['dropout_repairs'].append({'type': 'arm_chain_resolve', 'joint': side + 'Arm+ForeArm+Hand', 'frames': [lo, hi], 'seconds': [round(lo * b.dt, 4), round(hi * b.dt, 4)]})
        report['twist_repairs'][side + 'ForeArm'] = {'events': len(events), 'frames': events, 'alt_branch_frames': int(state.sum()), 'ends_on_alt_branch': bool(state[-1] == 1), 'twist_centre_deg': centre}
        bridges = []
        arm = R.from_quat(g[:, ai])
        rel = arm.inv() * R.from_quat(g[:, i])
        tw = twist_of(rel)
        bad = dilate(np.abs(wrap(tw - centre)) > 100, 4)
        for a, z in runs(bad):
            lo, hi = (max(1, a), min(n - 1, z))
            t = np.deg2rad(tw)
            t0, t1 = (t[lo - 1], t[hi])
            t1 = t0 + np.angle(np.exp(1j * (t1 - t0)))
            relq = rel.as_quat()
            twq = np.c_[np.sin(t / 2), np.zeros(n), np.zeros(n), np.cos(t / 2)]
            swing = rel * R.from_quat(twq).inv()
            tt = t0 + (t1 - t0) * (np.arange(lo, hi) - (lo - 1)) / (hi - (lo - 1))
            newrel = swing[lo:hi] * R.from_quat(np.c_[np.sin(tt / 2), np.zeros(hi - lo), np.zeros(hi - lo), np.cos(tt / 2)])
            wrist = R.from_quat(g[lo:hi, i]).inv() * R.from_quat(g[lo:hi, hj])
            g[lo:hi, i] = (arm[lo:hi] * newrel).as_quat()
            g[lo:hi, hj] = (R.from_quat(g[lo:hi, i]) * wrist).as_quat()
            bridges.append({'joint': side + 'ForeArm', 'kind': 'twist_interp', 'frames': [int(lo), int(hi)], 'seconds': [round(lo * b.dt, 3), round(hi * b.dt, 3)], 'twist_deg_from_to': [round(float(np.rad2deg(t0)), 1), round(float(np.rad2deg(t1)), 1)], 'hand_follows_forearm': True})
        fore = R.from_quat(g[:, i])
        hrel = fore.inv() * R.from_quat(g[:, hj])
        htw = twist_of(hrel)
        bad = dilate(np.abs(htw) > 125, 4)
        for a, z in runs(bad):
            lo, hi = (max(1, a), min(n - 1, z))
            hq = hrel.as_quat()
            sl = Slerp([lo - 1, hi], R.from_quat(hq[[lo - 1, hi]]))(np.arange(lo, hi))
            g[lo:hi, hj] = (fore[lo:hi] * sl).as_quat()
            bridges.append({'joint': side + 'Hand', 'kind': 'relative_slerp', 'frames': [int(lo), int(hi)], 'seconds': [round(lo * b.dt, 3), round(hi * b.dt, 3)]})
        for mside, lo, hi in config['manual_bridges']:
            if mside != side:
                continue
            arm = R.from_quat(g[:, ai])
            rel = arm.inv() * R.from_quat(g[:, i])
            tw = twist_of(rel)
            t = np.deg2rad(tw)
            t0, t1 = (t[lo - 1], t[hi])
            t1 = t0 + np.angle(np.exp(1j * (t1 - t0)))
            swing = rel * R.from_quat(np.c_[np.sin(t / 2), np.zeros(n), np.zeros(n), np.cos(t / 2)]).inv()
            tt = t0 + (t1 - t0) * (np.arange(lo, hi) - (lo - 1)) / (hi - (lo - 1))
            g[lo:hi, i] = (arm[lo:hi] * swing[lo:hi] * R.from_quat(np.c_[np.sin(tt / 2), np.zeros(hi - lo), np.zeros(hi - lo), np.cos(tt / 2)])).as_quat()
            fore = R.from_quat(g[:, i])
            hrel = fore.inv() * R.from_quat(g[:, hj])
            hq = hrel.as_quat()
            g[lo:hi, hj] = (fore[lo:hi] * Slerp([lo - 1, hi], R.from_quat(hq[[lo - 1, hi]]))(np.arange(lo, hi))).as_quat()
            bridges.append({'joint': side + 'ForeArm+Hand', 'kind': 'manual_twist_interp+relative_slerp', 'frames': [int(lo), int(hi)], 'seconds': [round(lo * b.dt, 3), round(hi * b.dt, 3)], 'twist_deg_from_to': [round(float(np.rad2deg(t0)), 1), round(float(np.rad2deg(t1)), 1)]})
        report['glitch_bridges'][side] = bridges
    corrected_local = g.copy()
    for i, j in enumerate(b.joints):
        if j['parent'] >= 0:
            corrected_local[:, i] = (R.from_quat(g[:, j['parent']]).inv() * R.from_quat(g[:, i])).as_quat()
    twistpos, _ = b.fk(corrected_local)
    report['twist_only_joint_position_error_max'] = float(np.max(np.linalg.norm(twistpos - orig, axis=2)))
    pre = g.copy()
    report['wrist_repairs'] = {}
    for i, j in enumerate(b.joints):
        if not j['channels']:
            continue
        wrist_mode = 'Hand' in j['name'] and b.joints[j['parent']]['name'].endswith('ForeArm')
        q = (R.from_quat(pre[:, j['parent']]).inv() * R.from_quat(pre[:, i])).as_quat() if wrist_mode else g[:, i].copy()
        if wrist_mode:
            events = []
            mag = R.from_quat(q).magnitude() * 180 / np.pi
            gross = np.zeros(n, bool)
            over = mag > 110
            over = over | binary_closing(over, structure=np.ones(13))
            for a, z in runs(over):
                if (z - a) * b.dt >= 0.1:
                    gross[a:z] = True
            for a, z in runs(dilate(gross, 4)):
                lo, hi = (max(1, a), min(n - 1, z))
                if lo >= hi:
                    continue
                path = Slerp([lo - 1, hi], R.from_quat(q[[lo - 1, hi]]))(np.arange(lo, hi))
                ends = R.from_quat(q[[lo - 1, hi]]).magnitude()
                if path.magnitude().max() > ends.max() + np.deg2rad(10):
                    mid = lo - 1 + (hi - lo + 1) * ends[0] / max(ends.sum(), 1e-09)
                    k = np.arange(lo, hi)
                    ident = np.array([0, 0, 0, 1.0])
                    first = Slerp([lo - 1, mid], R.from_quat([q[lo - 1], ident]))(np.clip(k, lo - 1, mid)).as_quat()
                    second = Slerp([mid, hi], R.from_quat([ident, q[hi]]))(np.clip(k, mid, hi)).as_quat()
                    path = R.from_quat(np.where((k <= mid)[:, None], first, second))
                q[lo:hi] = path.as_quat()
                events.append({'frames': [int(lo), int(hi)], 'seconds': [round(lo * b.dt, 3), round(hi * b.dt, 3)], 'peak_relation_deg': round(float(mag[a:z].max()), 1)})
            st = (R.from_quat(q[:-1]).inv() * R.from_quat(q[1:])).magnitude() * 180 / np.pi
            for a, z in runs(dilate(np.r_[st > 40, False] | np.r_[False, st > 40], 4)):
                lo, hi = (max(1, a), min(n - 1, z))
                if lo >= hi:
                    continue
                q[lo:hi] = Slerp([lo - 1, hi], R.from_quat(q[[lo - 1, hi]]))(np.arange(lo, hi)).as_quat()
                events.append({'frames': [int(lo), int(hi)], 'seconds': [round(lo * b.dt, 3), round(hi * b.dt, 3)], 'peak_step_deg': round(float(st[max(0, a - 1):z].max()), 1)})
            tw = twist_of(R.from_quat(q))
            for a, z in runs(dilate(np.abs(wrap(tw)) > 100, 4)):
                lo, hi = (max(1, a), min(n - 1, z))
                if lo >= hi:
                    continue
                q[lo:hi] = Slerp([lo - 1, hi], R.from_quat(q[[lo - 1, hi]]))(np.arange(lo, hi)).as_quat()
                events.append({'frames': [int(lo), int(hi)], 'seconds': [round(lo * b.dt, 3), round(hi * b.dt, 3)], 'peak_twist_deg': round(float(np.abs(wrap(tw[a:z])).max()), 1)})
            report['wrist_repairs'][j['name']] = events
        step = (R.from_quat(q[:-1]).inv() * R.from_quat(q[1:])).magnitude() * 180 / np.pi
        jumps = np.flatnonzero(step > 23)
        events = []
        last = -1
        for k, a in enumerate(jumps):
            if a <= last:
                continue
            for z in jumps[k + 1:]:
                if z - a > int((0.35 if 'ForeArm' in j['name'] or 'Hand' in j['name'] else 0.24) / b.dt):
                    break
                across = (R.from_quat(q[a]).inv() * R.from_quat(q[z + 1])).magnitude() * 180 / np.pi
                if across < max(18, (z - a) * 1.5):
                    lo = max(0, a - 2)
                    hi = min(n - 1, z + 3)
                    q[lo:hi + 1] = Slerp([lo, hi], R.from_quat(q[[lo, hi]]))(np.arange(lo, hi + 1)).as_quat()
                    events.append([int(lo), int(hi)])
                    last = hi
                    break
        report['spike_repairs'][j['name']] = events
        sigma = 1.4 if j['name'] in ['Hips', 'Spine', 'Spine1', 'Neck', 'Head'] else 1.8
        if 'Hand' in j['name']:
            sigma = 2.2
        smoothed = qsmooth(q, sigma)
        if wrist_mode:
            smoothed = strong_smooth(smoothed, 12)
        g[:, i] = (R.from_quat(g[:, j['parent']]) * R.from_quat(smoothed)).as_quat() if wrist_mode else smoothed
        if not wrist_mode and any((part in j['name'] for part in ['Shoulder', 'Arm', 'Hand', 'Foot', 'ToeBase'])):
            limit = 12 if 'Hand' in j['name'] or 'Foot' in j['name'] or 'ToeBase' in j['name'] else 10
            g[:, i] = strong_smooth(g[:, i], limit)
    report['backbend_repairs'] = []
    for side in ['Left', 'Right']:
        F, H = (b.names[side + 'ForeArm'], b.names[side + 'Hand'])
        sgn = 1.0 if side == 'Left' else -1.0

        def bend_az(G):
            d = (R.from_quat(G[:, F]).inv() * R.from_quat(G[:, H])).apply(np.tile([sgn, 0, 0], (n, 1)))
            return (np.rad2deg(np.arccos(np.clip(d[:, 0] * sgn, -1, 1))), np.rad2deg(np.arctan2(d[:, 2], d[:, 1])))
        bs, azs = bend_az(rawg)
        if not np.any(bs > 40):
            continue
        palm = float(np.rad2deg(np.angle(np.exp(1j * np.deg2rad(azs[bs > 40])).mean())))
        bend, az = bend_az(g)
        off = np.abs((az - palm + 180) % 360 - 180)
        core = (off > 120) & (bend > 70)
        soft = (off > 90) & (bend > 40)
        wins = []
        for a, z in runs(core):
            if (z - a) * b.dt < 0.05:
                continue
            lo, hi = (a, z)
            while lo > 1 and soft[lo - 1]:
                lo -= 1
            while hi < n - 1 and soft[hi]:
                hi += 1
            lo, hi = (max(1, lo - 3), min(n - 1, hi + 3))
            if wins and lo <= wins[-1][1] + 6:
                wins[-1][1] = max(wins[-1][1], hi)
            else:
                wins.append([lo, hi])
        fore = R.from_quat(g[:, F])
        rel = fore.inv() * R.from_quat(g[:, H])
        for lo, hi in wins:
            u = (np.arange(lo, hi) - (lo - 1)) / (hi - (lo - 1))
            u = u * u * (3 - 2 * u)
            g[lo:hi, H] = (fore[lo:hi] * Slerp([0, 1], rel[[lo - 1, hi]])(u)).as_quat()
            report['backbend_repairs'].append({'joint': side + 'Hand', 'frames': [int(lo), int(hi)], 'seconds': [round(lo * b.dt, 3), round(hi * b.dt, 3)], 'bend_max_deg': round(float(bend[lo:hi].max()), 1), 'palm_azimuth_deg': round(palm, 1)})
    local = g.copy()
    for i, j in enumerate(b.joints):
        if j['parent'] >= 0:
            local[:, i] = (R.from_quat(g[:, j['parent']]).inv() * R.from_quat(g[:, i])).as_quat() if j['channels'] else b.local[:, i]
    root = gaussian_filter1d(b.data[:, :3], 1.2, axis=0, mode='nearest', truncate=3)
    fade = np.minimum(np.arange(n), np.arange(n)[::-1]) * b.dt / 0.12
    fade = np.minimum(fade, 1)
    root = b.data[:, :3] + fade[:, None] * (root - b.data[:, :3])
    for i, j in enumerate(b.joints):
        if not j['channels']:
            continue
        diff = R.from_quat(corrected_local[:, i]).inv() * R.from_quat(local[:, i])
        local[:, i] = (R.from_quat(corrected_local[:, i]) * R.from_rotvec(diff.as_rotvec() * fade[:, None])).as_quat()
    smoothpos, smoothg = b.fk(local, root)
    foot_before = {}
    foot_targets = {}
    masks = {}
    contactrows = []
    for side in ['Left', 'Right']:
        ti = b.names[side + 'ToeBase']
        ai = b.names[side + 'Foot']
        hi = b.names[side + 'UpLeg']
        ki = b.names[side + 'Leg']
        toe = smoothpos[:, ti]
        smtoe = gaussian_filter1d(toe, 2, axis=0)
        speed = np.linalg.norm(np.gradient(smtoe, b.dt, axis=0), axis=1)
        floor = float(np.percentile(toe[:, 1], 5))
        candidate = (toe[:, 1] < floor + 2.2) & (speed < 18)
        candidate = binary_closing(candidate, structure=np.ones(7))
        candidate = binary_opening(candidate, structure=np.ones(9))
        mask = np.zeros(n)
        target = toe.copy()
        accepted = []
        for a, z in runs(candidate):
            if z - a < int(0.18 / b.dt):
                continue
            pts = toe[a:z]
            anchor = np.median(pts, axis=0)
            extent = np.max(np.linalg.norm(pts[:, [0, 2]] - anchor[[0, 2]], axis=1))
            if extent > 5:
                continue
            edge = max(1, min(int(0.1 / b.dt), (z - a) // 4))
            weight = np.ones(z - a)
            ramp = (1 - np.cos(np.linspace(0, np.pi, edge))) / 2
            weight[:edge] = ramp
            weight[-edge:] = ramp[::-1]
            anchor[1] = np.median(pts[:, 1])
            mask[a:z] = weight
            target[a:z] = pts + weight[:, None] * (anchor - pts)
            accepted.append([int(a), int(z)])
            contactrows.append({'side': side, 'start_frame': int(a), 'end_frame_exclusive': int(z), 'start_seconds': round(a * b.dt, 4), 'end_seconds': round(z * b.dt, 4)})
        target_ankle = smoothpos[:, ai] + target - toe
        hp = smoothpos[:, hi]
        kp = smoothpos[:, ki]
        ap = smoothpos[:, ai]
        direction = target_ankle - hp
        distance = np.linalg.norm(direction, axis=1)
        u = direction / np.maximum(distance[:, None], 1e-08)
        l1 = float(np.linalg.norm(b.joints[ki]['offset']))
        l2 = float(np.linalg.norm(b.joints[ai]['offset']))
        source_reach = np.linalg.norm(orig[:, ai] - orig[:, hi], axis=1)
        cap = np.maximum(reach_cap(l1, l2, source_reach, config['knee_straighten_degrees']), abs(l1 - l2) + 0.02)
        reach = np.minimum(np.clip(distance, abs(l1 - l2) + 0.01, l1 + l2 - 0.01), cap)
        knee_capped = reach < distance - 1e-09
        target_ankle = hp + u * reach[:, None]
        pole = kp - hp - u * np.sum((kp - hp) * u, axis=1, keepdims=True)
        pole = gaussian_filter1d(pole, config['pole_sigma_samples'], axis=0, mode='nearest')
        pole = hold_pole(pole, np.linalg.norm(pole, axis=1) >= 1e-05, R.from_quat(smoothg[:, hi]).apply(np.tile([0, 0, 1.0], (n, 1))))
        pole -= u * np.sum(pole * u, axis=1, keepdims=True)
        pole /= np.linalg.norm(pole, axis=1, keepdims=True)
        along = (l1 * l1 - l2 * l2 + reach * reach) / (2 * reach)
        height = np.sqrt(np.maximum(l1 * l1 - along * along, 0))
        newk = hp + u * along[:, None] + pole * height[:, None]
        newhg = between(kp - hp, newk - hp) * R.from_quat(smoothg[:, hi])
        newkg = between(ap - kp, target_ankle - newk) * R.from_quat(smoothg[:, ki])
        parentg = R.from_quat(smoothg[:, b.joints[hi]['parent']])
        active = mask > 0
        local[active, hi] = (parentg.inv() * newhg).as_quat()[active]
        local[active, ki] = (newhg.inv() * newkg).as_quat()[active]
        local[active, ai] = (newkg.inv() * R.from_quat(smoothg[:, ai])).as_quat()[active]
        masks[side] = mask
        foot_before[side] = orig[:, ti]
        foot_targets[side] = target
        report['contacts'][side] = {'intervals': len(accepted), 'full_weight_frames': int(np.sum(mask > 0.999)), 'estimated_floor_native': floor, 'interval_frames': accepted}
        report['contacts'][side]['reach_capped_frames'] = int((knee_capped & active).sum())
        report['contacts'][side]['reach_cap_shortfall_max'] = float((distance - reach)[knee_capped & active].max()) if (knee_capped & active).any() else 0.0
    finalp, finalg = b.fk(local, root)
    data = b.data.copy()
    data[:, :3] = root
    for i, j in enumerate(b.joints):
        if not j['channels']:
            continue
        idx = [k for k, c in zip(j['indices'], j['channels']) if c.endswith('rotation')]
        order = ''.join((c[0] for c in j['channels'] if c.endswith('rotation')))
        with warnings.catch_warnings():
            warnings.simplefilter('ignore')
            e = R.from_quat(local[:, i]).as_euler(order, degrees=True)
        e[0] = b.data[0, idx]
        for f in range(1, n):
            a = e[f]
            alt = np.array([a[0] + 180, 180 - a[1], a[2] + 180])
            a = a + 360 * np.round((e[f - 1] - a) / 360)
            alt = alt + 360 * np.round((e[f - 1] - alt) / 360)
            e[f] = a if np.linalg.norm(a - e[f - 1]) <= np.linalg.norm(alt - e[f - 1]) else alt
        data[:, idx] = e
    out = BASE / 'repaired.bvh'
    b.write(out, data)
    check = BVH(out)
    cp, cg = check.fk()
    if not (check.header == b.header and check.frame_count == b.frame_count and (check.dt == b.dt)):
        raise ValueError('Internal verification failed')
    err = np.max(np.linalg.norm(cp - finalp, axis=2))
    if not err < 0.001:
        raise ValueError(err)
    report['export_fk_error_max'] = float(err)
    report['root_correction_max'] = float(np.max(np.linalg.norm(root - b.data[:, :3], axis=1)))
    report['position_delta'] = stats(np.linalg.norm(cp - orig, axis=2))
    report['joint_step_degrees'] = {}
    for name in ['LeftForeArm', 'RightForeArm', 'LeftHand', 'RightHand', 'LeftArm', 'RightArm', 'Head', 'LeftFoot', 'RightFoot']:
        i = b.names[name]
        before = (R.from_quat(rawg[:-1, i]).inv() * R.from_quat(rawg[1:, i])).magnitude() * 180 / np.pi
        after = (R.from_quat(cg[:-1, i]).inv() * R.from_quat(cg[1:, i])).magnitude() * 180 / np.pi
        report['joint_step_degrees'][name] = {'before_max': float(before.max()), 'after_max': float(after.max()), 'before_over60': int(sum(before > 60)), 'after_over60': int(sum(after > 60))}
    for side in masks:
        ti = b.names[side + 'ToeBase']
        m = (masks[side][:-1] > 0.999) & (masks[side][1:] > 0.999)
        before = np.linalg.norm(np.diff(orig[:, ti][:, [0, 2]], axis=0), axis=1) / b.dt
        after = np.linalg.norm(np.diff(cp[:, ti][:, [0, 2]], axis=0), axis=1) / b.dt
        report['contacts'][side]['horizontal_speed_before_mean'] = float(before[m].mean()) if m.any() else None
        report['contacts'][side]['horizontal_speed_after_mean'] = float(after[m].mean()) if m.any() else None
        report['contacts'][side]['toe_height_before_min'] = float(orig[:, ti, 1].min())
        report['contacts'][side]['toe_height_after_min'] = float(cp[:, ti, 1].min())
        report['contacts'][side]['toe_correction_max'] = float(np.linalg.norm(cp[:, ti] - orig[:, ti], axis=1).max())
    (BASE / 'repair_report.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    (BASE / 'contacts.json').write_text(json.dumps(contactrows, indent=2), encoding='utf-8')
    print(json.dumps({k: v for k, v in report.items() if k not in ['spike_repairs', 'twist_repairs', 'contacts', 'dropout_repairs']}, indent=2))
    print('TWIST', [(k, v['events']) for k, v in report['twist_repairs'].items()])
    print('SPIKES', [(k, len(v)) for k, v in report['spike_repairs'].items() if v])
    print('CONTACTS', json.dumps({k: {a: z for a, z in v.items() if a != 'interval_frames'} for k, v in report['contacts'].items()}, indent=2))
    np.savez(BASE / 'repaired_cache.npz', data=data, positions=cp, global_quat=cg, local=check.local)
