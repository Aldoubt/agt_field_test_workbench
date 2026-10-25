"""Immutable full-resolution clouds and reproducible PCD-to-grid operations."""
from pathlib import Path
import json
import math
import shutil
import os
import yaml
from .runtime import ROOT, REPO, ros_call, write_json
from .maps import Maps, _new_id, hash_file, read_pgm, scan_context_database


def finite(value):
    return isinstance(value,(int,float)) and not isinstance(value,bool) and math.isfinite(value)


def operations(items):
    if not isinstance(items,list) or len(items)>100: raise ValueError('invalid operations')
    for op in items:
        if op.get('type') in ('crop','delete'):
            if any(not isinstance(op.get(k),list) or len(op[k])!=3 or not all(map(finite,op[k])) for k in ('min','max')):
                raise ValueError('ROI needs finite XYZ bounds')
            if any(a>=b for a,b in zip(op['min'],op['max'])): raise ValueError('ROI min must be smaller than max')
        elif op.get('type')=='voxel':
            if not finite(op.get('size')) or op['size']<=0: raise ValueError('positive voxel size required')
        else: raise ValueError('unknown cloud operation')
    return items


def projection(p):
    defaults=dict(resolution=.05,origin=[0.,0.,0.],range=[20.,20.],z_min=.15,z_max=2.,ground_min=-.3,
                  min_points=2,occupied_thresh=.65,free_thresh=.2)
    if not isinstance(p,dict) or set(p)-set(defaults): raise ValueError('unknown projection parameter')
    defaults.update(p);p=defaults
    if any(not finite(p[k]) for k in ('resolution','z_min','z_max','ground_min','occupied_thresh','free_thresh')):
        raise ValueError('nonfinite projection parameter')
    if p['resolution']<=0 or p['z_min']>=p['z_max'] or p['ground_min']>p['z_min']: raise ValueError('invalid height or resolution')
    for k,n in (('origin',3),('range',2)):
        if not isinstance(p[k],list) or len(p[k])!=n or not all(map(finite,p[k])): raise ValueError('invalid '+k)
    if min(p['range'])<=0 or math.ceil(p['range'][0]/p['resolution'])*math.ceil(p['range'][1]/p['resolution'])>16_000_000:
        raise ValueError('output grid too large')
    if not isinstance(p['min_points'],int) or p['min_points']<1 or not 0<=p['free_thresh']<p['occupied_thresh']<=1:
        raise ValueError('invalid occupancy threshold')
    return p


class Clouds:
    def __init__(self,root=ROOT,runner=ros_call):
        self.root=Path(root); self.directory=self.root/'maps/clouds';self.runner=runner
        self.directory.mkdir(parents=True,exist_ok=True)
    def list(self):
        return [json.loads(p.read_text()) for p in sorted(self.directory.glob('*/manifest.json'),reverse=True)]
    def get(self,version):
        import re
        if not re.fullmatch(r'[0-9TZ-]{16,24}-[0-9a-f]{8}',version): raise ValueError('invalid cloud ID')
        record=json.loads((self.directory/version/'manifest.json').read_text())
        if hash_file(record['pcd'])!=record['sha256']: raise ValueError('PCD SHA changed')
        return record
    def _process(self,source,spec):
        ident=_new_id(); config=self.root/'runs'/ident/'operations.yaml';config.parent.mkdir(parents=True)
        config.write_text(yaml.safe_dump(spec,sort_keys=False));output=self.directory/ident
        self.runner(['ros2','run','agt_field_test_bringup','pcd_ops',str(source),str(config),str(output)],timeout=3600)
        return ident,output,config
    @staticmethod
    def _artifact_root(path):
        if not path:return None
        source=Path(path).expanduser().resolve()
        for candidate in (source.parent,*source.parents):
            if all((candidate/name).is_file() for name in ('frames.csv','trajectory.csv','points.raw')):
                return candidate
        return None

    @staticmethod
    def _keyframes_ready(path):
        if not path:return False
        root=Path(path).expanduser()
        return root.is_dir() and (root/'patches').is_dir() and any(
            (root/name).is_file() for name in ('poses_timed.txt','poses.txt'))

    def import_pcd(self,path,calibration=None,bag=None,name='',cleanup=None):
        source=Path(path).expanduser().resolve()
        if not source.is_file() or source.suffix.lower()!='.pcd': raise ValueError('existing PCD required')
        sha=hash_file(source)
        calibration=dict(calibration or {})
        artifact=calibration.get('source_artifact') or self._artifact_root(source)
        if artifact and not Path(artifact).expanduser().is_dir():artifact=None
        if artifact:artifact=str(Path(artifact).expanduser().resolve())
        keyframes=calibration.get('sc_keyframes')
        if not self._keyframes_ready(keyframes) and artifact:
            for candidate in (Path(artifact)/'temporal_cleanup'/'keyframes',Path(artifact)/'sc_keyframes'):
                if self._keyframes_ready(candidate):
                    keyframes=str(candidate.resolve());break
        lio_config=calibration.get('internal_config') or calibration.get('lio_config')
        if lio_config and not Path(lio_config).expanduser().is_file():lio_config=None
        ident,out,config=self._process(source,{'operations':[]})
        shutil.copyfile(source,out/'original.pcd')
        if hash_file(source)!=sha or hash_file(out/'original.pcd')!=sha: raise ValueError('source changed during import')
        bag_sha={str(p.relative_to(Path(bag))):hash_file(p) for p in Path(bag).rglob('*') if p.is_file()} if bag and Path(bag).is_dir() else None
        return self._save(ident,out,dict(source=str(source),source_sha256=sha,calibration=calibration,source_bag=bag,source_bag_sha256=bag_sha,
            source_artifact=artifact,sc_keyframes=str(Path(keyframes).resolve()) if self._keyframes_ready(keyframes) else None,
            lio_config=str(Path(lio_config).expanduser().resolve()) if lio_config else None,
            name=str(name).strip()[:120],parent=None,operations=[],temporal_cleanup=cleanup))

    def _save(self,ident,out,lineage):
        record=dict(id=ident,pcd=str(out/'cloud.pcd'),sha256=hash_file(out/'cloud.pcd'),status='DRAFT',
                    frame_id='map',processing=yaml.safe_load((out/'result.yaml').read_text()),**lineage)
        write_json(out/'manifest.json',record);return record

    def clean(self,parent,items):
        original=self.get(parent);items=operations(items)
        ident,out,config=self._process(original['pcd'],{'operations':items})
        return self._save(ident,out,dict(parent=parent,parent_sha256=original['sha256'],operations=items,
            calibration=original.get('calibration'),source_bag=original.get('source_bag'),source_bag_sha256=original.get('source_bag_sha256'),
            source_artifact=original.get('source_artifact'),sc_keyframes=original.get('sc_keyframes'),
            lio_config=original.get('lio_config'),name=original.get('name',''),source=original['pcd'],source_sha256=original['sha256']))

    def _scan_context_inputs(self,cloud):
        calibration=cloud.get('calibration') or {}
        artifact_value=cloud.get('source_artifact') or calibration.get('source_artifact')
        if not artifact_value and cloud.get('source'):
            artifact_value=self._artifact_root(cloud['source'])
        artifact=Path(artifact_value).expanduser().resolve() if artifact_value else None
        if artifact and not artifact.is_dir():artifact=None
        keyframes_value=cloud.get('sc_keyframes') or calibration.get('sc_keyframes')
        if not self._keyframes_ready(keyframes_value) and artifact:
            for candidate in (artifact/'temporal_cleanup'/'keyframes',artifact/'sc_keyframes',
                              Path(cloud['pcd']).parent/'sc_keyframes'):
                if self._keyframes_ready(candidate):
                    keyframes_value=str(candidate.resolve());break
        keyframe_error=''
        if not self._keyframes_ready(keyframes_value) and artifact and all(
                (artifact/name).is_file() for name in ('frames.csv','trajectory.csv','points.raw')):
            target=Path(cloud['pcd']).parent/'sc_keyframes'
            if target.exists() and not self._keyframes_ready(target):
                target=Path(cloud['pcd']).parent/('sc_keyframes_'+cloud['sha256'][:8])
            if not target.exists():
                try:
                    from .map_cleanup import prepare_keyframes
                    prepare_keyframes(artifact,target)
                except Exception as exc:
                    keyframe_error=str(exc)[:500]
            if self._keyframes_ready(target):keyframes_value=str(target.resolve())
        lio_value=cloud.get('lio_config') or calibration.get('internal_config') or calibration.get('lio_config')
        lio_config=Path(lio_value).expanduser().resolve() if lio_value else None
        lio_error=''
        if lio_config and not lio_config.is_file():
            lio_error='recorded LIO configuration is missing: '+str(lio_config);lio_config=None
        elif lio_config and calibration.get('internal_sha256') and hash_file(lio_config)!=calibration['internal_sha256']:
            lio_error='recorded LIO configuration SHA-256 changed';lio_config=None
        keyframes=Path(keyframes_value).expanduser().resolve() if self._keyframes_ready(keyframes_value) else None
        return artifact,keyframes,lio_config,keyframe_error,lio_error

    def build_assets(self,cloud_id,map_id=None):
        """Build same-source BBS voxels, Polar Context candidates and SC descriptors."""
        cloud=self.get(cloud_id);source=Path(cloud['pcd'])
        directory=source.parent/'relocalization';directory.mkdir(exist_ok=True)
        status_path=directory/'asset_status.json'
        old={}
        if status_path.is_file():
            try:old=json.loads(status_path.read_text())
            except (OSError,ValueError):old={}
        state=dict(old) if old.get('source_sha256')==cloud['sha256'] else {}
        state.update(source_sha256=cloud['sha256'],source_pcd=str(source))
        artifact,keyframes,lio_config,keyframe_error,lio_error=self._scan_context_inputs(cloud)

        map_record=None
        if map_id:
            map_record=Maps(self.root).get(map_id)
        else:
            for candidate in Maps(self.root).list():
                if candidate.get('cloud_id')==cloud_id and candidate.get('source_pcd_sha256')==cloud['sha256']:
                    map_record=candidate;break

        bbs_ready=(state.get('state')=='READY' and state.get('bbs')=='READY'
            and (directory/'voxelmaps_coords').is_dir()
            and (directory/'global_map_downsampled.pcd').is_file())
        if not bbs_ready:
            state.update(state='BUILDING',bbs='BUILDING')
            write_json(status_path,state)
            temporary=source.parent/('relocalization_build_'+_new_id())
            try:
                self.runner(['ros2','run','agt_global_relocalization_native',
                             'build_relocalization_assets','--map',str(source),
                             '--output',str(temporary)],timeout=1200)
                if not (temporary/'voxelmaps_coords').is_dir() or not (temporary/'global_map_downsampled.pcd').is_file():
                    raise RuntimeError('BBS builder returned without mandatory voxel assets')
                for child in temporary.iterdir():
                    target=directory/child.name
                    if target.is_dir():
                        if target.exists():shutil.rmtree(target)
                        os.replace(child,target)
                    else:os.replace(child,target)
                state.update(state='READY',bbs='READY',error='')
            except Exception as error:
                state.update(state='FAILED',bbs='FAILED',error=str(error)[:800])
            finally:
                if temporary.exists():shutil.rmtree(temporary)

        if (directory/'polar_context.db').is_file():
            state['polar_context']='READY'
        elif keyframes and state.get('bbs')=='READY':
            try:
                from .polar_context import prepare_builder_input
                compatible = directory/('polar_context_source_'+_new_id())
                provenance = prepare_builder_input(keyframes, compatible)
                temporary = directory/('.polar_context_build_'+_new_id())
                self.runner(['ros2','run','agt_global_relocalization_native',
                             'build_relocalization_candidates','--map-dir',str(compatible),
                             '--output',str(temporary)],timeout=1200)
                if not (temporary/'polar_context.db').is_file() or not (temporary/'polar_context.db').stat().st_size:
                    raise RuntimeError('candidate builder returned without polar_context.db')
                metadata = yaml.safe_load((temporary/'polar_context.yaml').read_text())
                if metadata.get('entries', 0) <= 0:
                    raise RuntimeError('candidate builder returned an empty index')
                # Publish the complete DB last. A running matcher must never read a partial file.
                os.replace(temporary/'polar_context.yaml', directory/'polar_context.yaml')
                os.replace(temporary/'polar_context.db', directory/'polar_context.db')
                temporary.rmdir()
                state.update(polar_context_entries=metadata['entries'],
                    polar_context_db_sha256=hash_file(directory/'polar_context.db'),
                    polar_context_source=provenance)
                state['polar_context']='READY';state.pop('polar_context_error',None)
            except Exception as error:
                state['polar_context']='FAILED';state['polar_context_error']=str(error)[:600]
        else:
            state.setdefault('polar_context','NOT_GENERATED_NO_KEYFRAME_PATCHES')

        sc_database='';sc_error=''
        if map_record:
            from .relocalization import validate_sc_database
            existing=state.get('sc_database') or scan_context_database(map_record)
            if existing:
                try:sc_database=validate_sc_database(existing,map_record)
                except Exception:sc_database=''
            if not sc_database and keyframes and lio_config:
                base_name='scancontext_db_'+cloud['sha256'][:12]+'_'+hash_file(lio_config)[:8]
                output=directory/base_name
                if output.exists():output=directory/(base_name+'_'+_new_id())
                command=['python3',str(REPO/'scripts/build_sc_database.py'),
                         '--map',str(source),'--map-yaml',str(map_record['yaml']),
                         '--lio-config',str(lio_config),'--output',str(output),'--keyframes',str(keyframes)]
                try:
                    self.runner(command,timeout=1200)
                    sc_database=validate_sc_database(str(output),map_record)
                except Exception as error:
                    sc_error=str(error)[:800]
            if sc_database:
                state.update(scan_context='READY',sc_database=sc_database)
                state.pop('scan_context_error',None)
            else:
                state['scan_context']='FAILED' if sc_error else 'NOT_GENERATED'
                state['scan_context_error']=sc_error or lio_error or keyframe_error or (
                    'mapping artifact keyframes are unavailable' if not keyframes else
                    'matching LIO configuration is unavailable' if not lio_config else
                    'no SC database matches this PCD and map-frame geometry')
        else:
            state['scan_context']='NOT_GENERATED_NO_MAP_VERSION'
            state['scan_context_error']='generate the navigation grid before building the SC database'
        write_json(status_path,state)
        return state

    def grid(self,parent,params,name=None,build_assets=True):
        """Reuse the pinned, terrain-relative AGT exporter for a standalone PCD.

        The keyframe-dependent traversability mode is NOT available for a single
        fused FAST-LIO2 map. Keep unknown cells unknown and never mutate the
        localization PCD. Every projection creates a new draft map version.
        """
        original=self.get(parent)
        requested=dict(params or {})
        supported={'resolution','ground_cell_size','ground_percentile','ground_tolerance',
                   'obstacle_min_height','obstacle_max_height','occupied_threshold',
                   'free_space_radius_cells','closing_radius_cells','min_component_cells',
                   'min_points','z_min','z_max','ground_min'}
        invalid=set(requested)-supported
        if invalid:raise ValueError('unknown local_ground parameter(s): '+', '.join(sorted(invalid)))
        legacy_height=sorted(set(requested)&{'z_min','z_max','ground_min'})
        # Older clients sent absolute world-Z slices. They are deliberately
        # ignored, rather than silently reinterpreted as heights above ground.
        if 'min_points' in requested and 'occupied_threshold' not in requested:
            requested['occupied_threshold']=requested['min_points']
        defaults=Path(self.root)/'src/agt-lio-pgo-mapping/exporters/agt_pcd2grid_exporter/config/projection.yaml'
        if not defaults.is_file():
            raise ValueError('missing locked agt_pcd2grid_exporter local_ground config: '+str(defaults))
        config=yaml.safe_load(defaults.read_text())
        if config.get('projection_mode')!='local_ground':
            raise ValueError('locked exporter config is not local_ground')
        config['temporal_filter']['enabled']=False  # no per-keyframe patches in a fused PCD
        fields={
            'resolution':('resolution',None),
            'ground_cell_size':('ground','cell_size'),
            'ground_percentile':('ground','percentile'),
            'ground_tolerance':('ground','free_tolerance'),
            'obstacle_min_height':('obstacle_height','min'),
            'obstacle_max_height':('obstacle_height','max'),
            'occupied_threshold':('occupied_threshold',None),
            'free_space_radius_cells':('free_space_radius_cells',None),
            'closing_radius_cells':('postprocess','closing_radius_cells'),
            'min_component_cells':('postprocess','min_component_cells')}
        for key,(group,item) in fields.items():
            if key not in requested:continue
            value=requested[key]
            if not finite(value):raise ValueError('non-finite local_ground parameter: '+key)
            if key in ('occupied_threshold','free_space_radius_cells','closing_radius_cells','min_component_cells'):
                if int(value)!=value or value<0 or (key=='occupied_threshold' and value<1):
                    raise ValueError('invalid integer local_ground parameter: '+key)
                value=int(value)
            if item is None:config[group]=value
            else:config[group][item]=value
        if not .01<=float(config['resolution'])<=1.:
            raise ValueError('resolution outside supported [0.01,1.0] m')
        if not 0.<float(config['ground']['cell_size'])<=10.:
            raise ValueError('invalid ground_cell_size')
        if not 0.<=float(config['ground']['percentile'])<=1.:
            raise ValueError('ground_percentile outside [0,1]')
        if float(config['ground']['free_tolerance'])<0.:
            raise ValueError('ground_tolerance must be nonnegative')
        if not 0.<=float(config['obstacle_height']['min'])<float(config['obstacle_height']['max']):
            raise ValueError('relative obstacle height min must be below max')
        ident=_new_id()
        directory=self.root/'runs'/ident
        directory.mkdir(parents=True,exist_ok=False)
        snapshot=directory/'projection_local_ground.yaml'
        snapshot.write_text(yaml.safe_dump(config,sort_keys=False))
        out=self.directory/ident
        # The backend C++ executable already belongs to the locked mapping repo.
        self.runner(['ros2','run','agt_pcd2grid_exporter','pcd2grid_exporter',
                     '--pcd',original['pcd'],'--config',str(snapshot),'--output',str(out)],timeout=3600)
        meta=yaml.safe_load((out/'map.yaml').read_text())
        w,h,pixels=read_pgm(out/'map.pgm')
        stats=yaml.safe_load((out/'metadata.yaml').read_text())
        result = Maps(self.root)._save(w,h,pixels,meta,dict(
            parent=None,cloud_id=parent,
            projection_mode='local_ground',projection_backend='agt_pcd2grid_exporter',
            projection_config=str(snapshot),projection_config_sha256=hash_file(snapshot),
            projection_stats=stats,ignored_legacy_z_parameters=legacy_height,
            source_pcd=original['pcd'],source_pcd_sha256=original['sha256'],
            source_bag=original.get('source_bag'),
            source_bag_sha256=original.get('source_bag_sha256'),
            calibration=original.get('calibration'),
            pre_projection_cleanup=original.get('temporal_cleanup'),
            name=original.get('name','') if name is None else str(name).strip()[:120],
            source_yaml=str(out/'map.yaml'),source_pgm_sha256=hash_file(out/'map.pgm')))
        if build_assets:
            self.build_assets(parent,map_id=result['id'])
        from .maps import asset_summary, relocalization_assets, scan_context_database
        return dict(result, asset_summary=asset_summary(result),
                    relocalization_assets=relocalization_assets(result),
                    sc_database=scan_context_database(result))
