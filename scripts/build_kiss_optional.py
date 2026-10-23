#!/usr/bin/env python3
"""Optional isolated Python backend build; main workspace build never depends on it."""
import hashlib,json,os,shutil,subprocess,sys,tarfile,urllib.request
from pathlib import Path
REPO=Path(__file__).resolve().parents[1];ROOT=REPO.parent.parent
lock=json.loads((REPO/'dependencies/kiss_python_build.json').read_text());work=ROOT/'work/kiss_locked';work.mkdir(parents=True,exist_ok=True)
def run(cmd,**kw):subprocess.run(list(map(str,cmd)),check=True,**kw)
source=ROOT/'src/external/kiss_matcher'
if subprocess.check_output(['git','-c','safe.directory='+str(source),'-C',str(source),'rev-parse','HEAD'],text=True).strip()!=lock['kiss_matcher_sha']:raise SystemExit('unexpected KISS source revision')
pybind=work/'pybind11'
if not pybind.exists():
    run(['git','clone','--no-checkout','https://github.com/pybind/pybind11.git',pybind]);run(['git','-C',pybind,'checkout','--detach',lock['pybind11_sha']])
if subprocess.check_output(['git','-C',str(pybind),'rev-parse','HEAD'],text=True).strip()!=lock['pybind11_sha']:raise SystemExit('pybind checkout changed')
archive=work/'robin.tar.gz'
if not archive.exists():archive.write_bytes(urllib.request.urlopen(lock['robin_url']).read())
if hashlib.sha256(archive.read_bytes()).hexdigest()!=lock['robin_archive_sha256']:raise SystemExit('ROBIN archive checksum mismatch')
robin=work/'robin'
if not robin.exists():
    temporary=work/'unpack';temporary.mkdir(exist_ok=False)
    with tarfile.open(archive) as tar:
        for member in tar.getmembers():
            if member.issym() or member.islnk() or not (temporary/member.name).resolve().is_relative_to(temporary.resolve()):raise ValueError('unsafe archive path')
        tar.extractall(temporary)
    shutil.move(str(next(temporary.iterdir())),robin);temporary.rmdir()
    cmake=robin/'cmake/DownloadExternal.cmake';text=cmake.read_text()
    if text.count('GIT_TAG        main')!=1:raise ValueError('unexpected ROBIN dependency script')
    cmake.write_text(text.replace('GIT_TAG        main','GIT_TAG        '+lock['xenium_sha']))
text=(robin/'cmake/DownloadExternal.cmake').read_text()
if any(sha not in text for sha in [lock['pmc_sha'],lock['atomic_queue_sha'],lock['xenium_sha']]):raise SystemExit('ROBIN dependency pins changed')
tools=work/'build_tools'
run([sys.executable,'-m','pip','install','--target',tools,'scikit-build-core==1.1.1','packaging==26.3','pybind11==3.1.0','pathspec==1.1.1','tomli==2.5.0','ninja==1.13.2','typing-extensions==4.16.0','exceptiongroup==1.3.1'])
output=ROOT/'work/kiss_python_locked'
if output.exists():raise SystemExit('output exists; preserve it or select a new build version')
env=dict(os.environ,PYTHONPATH=str(tools),SKBUILD_BUILD_TARGETS='_kiss_matcher',CMAKE_BUILD_PARALLEL_LEVEL='1',
    CMAKE_ARGS=f'-DFETCHCONTENT_SOURCE_DIR_PYBIND11={pybind} -DFETCHCONTENT_SOURCE_DIR_ROBIN={robin} -DUSE_SYSTEM_ROBIN=OFF -DUSE_SYSTEM_TBB=ON -DUSE_SYSTEM_EIGEN3=ON')
run([sys.executable,'-m','pip','install','--no-build-isolation','--no-deps','--target',output,source/'python'],env=env)
(output/'build_metadata.json').write_text(json.dumps(dict(lock=lock,scope='EXPERIMENT_ONLY',physical_benchmark='NOT_RUN'),indent=2))
