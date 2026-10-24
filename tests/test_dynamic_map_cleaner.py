"""Pure-Python regressions for stationary operator cleanup."""
import importlib
import tempfile
from pathlib import Path

import numpy as np
import pytest

from agt_dynamic_map_cleaner.core import Settings, classify
from agt_dynamic_map_cleaner.pcd import read_xyz, write_xyz
from agt_dynamic_map_cleaner.cli import keyframes, run


def synthetic(persist=False, clear=True):
    person = np.array([1.125, .125, 1.125])
    pillar = np.array([2.125, 1.125, 1.125])
    ground = np.array([[x,y,0.] for x in np.arange(-.5, 3., .25)
                       for y in np.arange(-.5, 2., .25)])
    static = np.vstack((ground,person,pillar)).astype(np.float32)
    frames = []
    for i in range(5):
        origin = np.array([0.,0.,0.])
        world = np.vstack((person,pillar,ground[::8]))
        frames.append((float(i),origin,(0,0,0,1),world-origin))
    if clear:
        for i,ox in enumerate((-.6,-.1,.45,1.0,1.5), start=5):
            origin = np.array([ox,0.,0.])
            behind = origin + 3. * (person-origin)
            world = np.vstack((pillar, behind, ground[::8]))
            if persist:
                world = np.vstack((world, person))
            frames.append((float(i),origin,(0,0,0,1),world-origin))
    config = Settings(initial_window_sec=3.,later_frame_stride=1,
                      max_rays_per_frame=100,min_early_frames=3,
                      min_distinct_viewpoints=3,ray_tolerance_m=.18)
    return static, frames, config


def test_stationary_start_person_cleared_but_ground_pillar_preserved():
    static, frames, config = synthetic()
    keep, report = classify(static,frames,config)
    assert report['state'] == 'CANDIDATE'
    assert report['removed_points'] == 1
    assert not keep[-2]  # person
    assert keep[-1]  # pillar
    assert keep[:-2].all()  # all ground


def test_still_occupied_person_or_no_later_views_must_be_preserved():
    for persist, clear in ((True,True),(False,False)):
        static, frames, config = synthetic(persist,clear)
        keep, report = classify(static,frames,config)
        assert keep.all()
        assert report['removed_points'] == 0


def test_invalid_quaternion_and_nonmonotonic_timestamp_fail_closed():
    static, frames, config = synthetic()
    bad = list(frames)
    t,p,q,xyz = bad[1]
    bad[1] = (t,p,(0,0,0,0),xyz)
    with pytest.raises(ValueError,match='quaternion'):
        classify(static,bad,config)
    bad = list(frames)
    t,p,q,xyz=bad[-1]
    bad[-1]=(1.,p,q,xyz)
    with pytest.raises(ValueError,match='timestamp'):
        classify(static,bad,config)


def test_pcd_roundtrip_and_empty_output():
    with tempfile.TemporaryDirectory() as tmp:
        p = Path(tmp)
        for xyz in (np.array([[1,2,3]], dtype=np.float32),np.empty((0,3), dtype=np.float32)):
            write_xyz(p/'test.pcd',xyz)
            np.testing.assert_allclose(read_xyz(p/'test.pcd'),xyz)


def test_cli_immutable_output_with_real_pcd_keyframe_contract():
    static, frames, config = synthetic()
    with tempfile.TemporaryDirectory() as tmp:
        root=Path(tmp)
        write_xyz(root/'static.pcd',static)
        keydir=root/'keyframes'
        (keydir/'patches').mkdir(parents=True)
        pose_lines=[]
        for i,(stamp,origin,q,body) in enumerate(frames):
            name=f'{i:06d}.pcd'
            write_xyz(keydir/'patches'/name,body)
            pose_lines.append(f'{name} {stamp} {origin[0]} {origin[1]} {origin[2]} 1 0 0 0')
        (keydir/'poses_timed.txt').write_text('\n'.join(pose_lines)+'\n')
        import yaml
        params=root/'config.yaml'
        params.write_text(yaml.safe_dump(config.__dict__))
        result=run(root/'static.pcd',keydir,root/'out',params)
        assert result['state']=='CANDIDATE'
        assert result['removed_points']==1
        assert (root/'out/manifest.json').is_file()
        assert len(read_xyz(root/'out/static_candidate.pcd'))==len(static)-1
        assert len(read_xyz(root/'out/removed_candidates.pcd'))==1
        with pytest.raises(ValueError,match='already exists'):
            run(root/'static.pcd',keydir,root/'out',params)
