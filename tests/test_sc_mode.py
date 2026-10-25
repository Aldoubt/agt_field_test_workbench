"""Optional experiment routing/provenance tests; do not need SC or KISS installed."""
import json
from pathlib import Path
import subprocess
import pytest
from workbench.maps import Maps,hash_file
from workbench.relocalization import validate_sc_database
from test_map_diff import make_map

REPO=Path(__file__).resolve().parents[1]


def test_sc_database_bound_to_exact_map_and_grid(tmp_path):
    m=make_map(Maps(tmp_path),tmp_path,'source',[254]*6,'DRAFT')
    directory=tmp_path/'sc';directory.mkdir();data=directory/'descriptors.npz';data.write_bytes(b'opaque descriptor fixture')
    manifest=dict(algorithm='scancontext',map_sha256=m['source_pcd_sha256'],navigation_yaml_sha256=hash_file(Path(m['yaml'])),
        navigation_pgm_sha256=m['sha256'],descriptors_sha256=hash_file(data))
    (directory/'manifest.json').write_text(json.dumps(manifest))
    assert validate_sc_database(str(directory),m)==str(directory)
    changed=dict(m,source_pcd_sha256='wrong')
    with pytest.raises(ValueError,match='identity'):validate_sc_database(str(directory),changed)
    data.write_bytes(b'changed')
    with pytest.raises(ValueError,match='SHA'):validate_sc_database(str(directory),m)


def test_sc_patch_composes_with_existing_control_patch_and_preserves_edits(tmp_path):
    for name,sha in [('agt_navigation_v3','b3a60487dff973c6326aca2ca2e9db1e78daeaaa'),
                     ('fwmini_navigation_lab','34ac35650452027f6f8efde0e3c4ace6ca526a9f')]:
        dest=tmp_path/'src'/name;dest.parent.mkdir(parents=True,exist_ok=True)
        subprocess.run(['git','clone','-q','--shared','--no-checkout',str(REPO.parent/name),str(dest)],check=True)
        subprocess.run(['git','-C',str(dest),'checkout','-q','--detach',sha],check=True)
    def invoke(script,*extra):return subprocess.run(['python3',str(REPO/'scripts'/script),'--workspace',str(tmp_path),*extra],capture_output=True,text=True)
    assert invoke('apply_control_integration.py').returncode==0
    assert invoke('apply_sc_kiss_integration.py').returncode==0
    assert invoke('apply_sc_kiss_integration.py').returncode==0
    assert invoke('apply_control_integration.py','--check').returncode==0
    path=tmp_path/'src/agt_navigation_v3/navigation/localization/agt_global_relocalization/agt_global_relocalization/global_relocalization.py'
    path.write_text(path.read_text()+'\n# operator local edit\n');before=path.read_bytes()
    result=invoke('apply_sc_kiss_integration.py')
    assert result.returncode!=0 and 'preserved' in result.stderr and path.read_bytes()==before
