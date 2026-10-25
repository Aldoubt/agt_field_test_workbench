"""Conservative map version diff and immutable update candidate tests."""
from pathlib import Path
import base64
import json
import pytest

from workbench.maps import Maps, hash_file, read_pgm


def make_map(maps, root, name, pixels, status, origin=(0.,0.,0.), resolution=1.0):
    pcd=root/'maps'/'clouds'/(name+'.pcd')
    pcd.parent.mkdir(parents=True,exist_ok=True);pcd.write_bytes((name+' test source PCD').encode())
    return maps._save(3,2,bytes(pixels),
        {'resolution':resolution,'origin':list(origin),'negate':0,'occupied_thresh':.65,'free_thresh':.196},
        {'name':name,'source_pcd':str(pcd),'source_pcd_sha256':hash_file(pcd),
         'coordinate_reference_id':'synthetic-test-survey-frame',
         'source_yaml':None,'source_pgm_sha256':None,'source_bag':None,'calibration':None,
         'cloud_id':name,'projection':{'resolution':resolution}},status=status)


def test_map_diff_creates_draft_with_same_candidate_pcd_and_rgb_overlay(tmp_path):
    maps=Maps(tmp_path)
    base=make_map(maps,tmp_path,'base',[0,254,205,0,254,205],'ACCEPTED')
    candidate=make_map(maps,tmp_path,'candidate',[0,254,254,254,254,205],'DRAFT',origin=(1.,0.,0.))
    result=maps.compare_and_propose(base['id'],candidate['id'])
    report=result['report'];update=result['update_map']
    assert report['state']=='REVIEW_REQUIRED'
    assert report['automatic_activation'] is False
    assert report['base_map_id']==base['id']
    assert report['candidate_map_id']==candidate['id']
    assert update['status']=='DRAFT'
    assert update['parent']==base['id']
    assert update['source_pcd']==candidate['source_pcd']
    assert update['source_pcd_sha256']==candidate['source_pcd_sha256']
    assert report['counts']['new_obstacle_cells']>=0
    overlay=Path(report['overlay_png']).read_bytes()
    assert overlay.startswith(b'\x89PNG\r\n\x1a\n')
    assert overlay[25]==2
    assert Path(report['report_path']).is_file()
    manifest=json.loads(Path(tmp_path/'maps'/'versions'/update['id']/'manifest.json').read_text())
    assert manifest['status']=='DRAFT'
    assert manifest['base_map_sha256']==base['sha256']


def test_map_diff_rejects_nonaccepted_base_and_unaligned_grid(tmp_path):
    maps=Maps(tmp_path)
    base=make_map(maps,tmp_path,'base',[0,254,205,0,254,205],'DRAFT')
    candidate=make_map(maps,tmp_path,'candidate',[0,254,205,0,254,205],'DRAFT')
    with pytest.raises(ValueError,match='ACCEPTED'):
        maps.compare_and_propose(base['id'],candidate['id'])
    base['status']='ACCEPTED'
    (tmp_path/'maps'/'versions'/base['id']/'manifest.json').write_text(json.dumps(base))
    unaligned=make_map(maps,tmp_path,'unaligned',[0,254,205,0,254,205],'DRAFT',origin=(.4,0.,0.))
    with pytest.raises(ValueError,match='cell-aligned'):
        maps.compare_and_propose(base['id'],unaligned['id'])


def test_map_diff_requires_source_pcd_hash_unchanged(tmp_path):
    maps=Maps(tmp_path)
    base=make_map(maps,tmp_path,'base',[0,254,205,0,254,205],'ACCEPTED')
    candidate=make_map(maps,tmp_path,'candidate',[0,254,205,0,254,205],'DRAFT',origin=(1.,0.,0.))
    Path(candidate['source_pcd']).write_text('tampered')
    with pytest.raises(ValueError,match='SHA-256 changed'):
        maps.compare_and_propose(base['id'],candidate['id'])


def test_manual_grid_edit_keeps_same_source_pcd_and_relocalization_assets(tmp_path):
    maps=Maps(tmp_path)
    original=make_map(maps,tmp_path,'editable',[0,254,205,0,254,205],'ACCEPTED')
    assets=Path(original['source_pcd']).parent/'relocalization'
    (assets/'voxelmaps_coords').mkdir(parents=True)
    (assets/'global_map_downsampled.pcd').write_bytes(b'valid same-source BBS asset')
    (assets/'asset_status.json').write_text(json.dumps({
        'state':'READY','bbs':'READY','source_sha256':original['source_pcd_sha256']}))
    _,_,old_pixels=read_pgm(Path(original['pgm']))
    pixels=bytearray(old_pixels);pixels[1]=0
    revised=maps.revise(original['id'],base64.b64encode(bytes(pixels)).decode('ascii'),status='ACCEPTED')
    assert revised['id']!=original['id']
    assert revised['sha256']!=original['sha256']
    assert revised['source_pcd']==original['source_pcd']
    assert revised['source_pcd_sha256']==original['source_pcd_sha256']
    assert revised['localization_geometry_valid'] is True
    assert revised['navigation_grid_edited'] is True
    from workbench.maps import relocalization_assets
    assert relocalization_assets(revised)==str(assets)
    assert maps.get(original['id'])['sha256']==original['sha256']


def test_legacy_grid_only_invalidation_can_restore_unchanged_pcd(tmp_path):
    maps=Maps(tmp_path)
    original=make_map(maps,tmp_path,'legacy-edit',[0,254,205,0,254,205],'DRAFT')
    manifest_path=Path(tmp_path/'maps'/'versions'/original['id']/'manifest.json')
    legacy=dict(original,source_pcd=None,source_pcd_sha256=None,
        localization_geometry_valid=False,
        invalidated_source_pcd=original['source_pcd'],
        invalidated_source_pcd_sha256=original['source_pcd_sha256'],
        asset_invalidation_reason='Grid edited independently of PCD; rebuild from a matching PCD before localization')
    manifest_path.write_text(json.dumps(legacy))
    _,_,old_pixels=read_pgm(Path(original['pgm']))
    pixels=bytearray(old_pixels);pixels[1]=0
    revised=maps.revise(original['id'],base64.b64encode(bytes(pixels)).decode('ascii'),status='ACCEPTED')
    assert revised['source_pcd']==original['source_pcd']
    assert revised['source_pcd_sha256']==original['source_pcd_sha256']
    assert revised['localization_geometry_valid'] is True
