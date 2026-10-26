"""Run immutable V1/V2 comparators and V3 with identical input/selection events."""
import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
import json
from pathlib import Path
import subprocess
import os
import re
from replay_inputs import basename, resolve_input

ROOT = Path(__file__).resolve().parents[1]

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--versions',nargs='+',choices=['v1','v2','v3'],default=['v1','v2','v3'])
    p.add_argument('--jobs',type=int,default=3)
    p.add_argument('--cached',action='store_true',help='Tracking-only ablation, NOT final ADC validation')
    p.add_argument('--profile',default='default')
    p.add_argument('--tag',default='')
    p.add_argument('--gate-extra',type=float,default=-1.0)
    p.add_argument('--manifest', type=Path, default=ROOT/'reports/audit/recordings.json')
    p.add_argument('--data-root', type=Path, help='Root containing bin/ and tracking/ or tracking_diagnostics/')
    p.add_argument('--cached-dir', type=Path, default=ROOT/'reports/replay/v2')
    p.add_argument('--output-root', type=Path, default=ROOT/'reports/replay')
    p.add_argument('--dry-run', action='store_true', help='Validate and print commands without writing results')
    p.add_argument('--overwrite', action='store_true', help='Explicitly allow replacing result CSV/log files')
    a=p.parse_args()
    if a.cached and 'v1' in a.versions: p.error('V1 legacy Doppler comparison must replay raw ADC, not V2 cached points')
    if a.jobs < 1: p.error('--jobs must be positive')
    if a.tag and len(a.versions)!=1:p.error('--tag is for one-version experiments; do not mix version outputs')
    if a.tag and (not re.fullmatch(r'[A-Za-z0-9_-][A-Za-z0-9_.-]*',a.tag) or '..' in a.tag):
        p.error('--tag must be a simple experiment name, not a path')
    audit=json.loads(a.manifest.read_text(encoding='utf-8'))
    prepared = []
    for entry in audit['recordings']:
        stem = Path(basename(entry['bin'])).stem
        if a.cached:
            # Cached ablations need the explicit original frame count (including zero-point frames), not an ADC file.
            binary = Path(entry['bin_path'])
            if int(entry.get('bin_frames', 0)) <= 0: raise ValueError('Cached replay requires bin_frames in manifest')
        else:
            binary = resolve_input(entry['bin_path'], a.manifest, a.data_root, 'bin')
        diagnostic = resolve_input(entry['diagnostic_csv'], a.manifest, a.data_root, 'diagnostics')
        cfg = resolve_input(entry.get('cfg', audit['cfg']), a.manifest, a.data_root, 'cfg')
        selection = a.manifest.resolve().parent / (stem + '_selections.csv')
        if not selection.is_file(): raise FileNotFoundError(selection)
        prepared.append(dict(entry, bin_path=str(binary), diagnostic_csv=str(diagnostic),
                             cfg=str(cfg), selections=str(selection)))
        for version in a.versions:
            executable = ROOT/'build'/('replay_'+version+('.exe' if os.name=='nt' else ''))
            if not executable.is_file(): raise FileNotFoundError(executable)
            directory = a.output_root/(a.tag or version)
            if not a.overwrite and not a.dry_run and any((directory/(stem+suffix)).exists() for suffix in
                ('_frames.csv','_points.csv','_candidates.csv','_clusters.csv','_stdout.log')):
                raise FileExistsError(f'Preserve existing results: {directory}; choose --tag/--output-root')
        if a.cached and not (a.cached_dir/(stem+'_points.csv')).is_file():
            raise FileNotFoundError(a.cached_dir/(stem+'_points.csv'))
    def run(entry,version):
        stem=Path(basename(entry['bin'])).stem
        directory=a.output_root/(a.tag or version)
        if not a.dry_run: directory.mkdir(parents=True,exist_ok=True)
        exe=ROOT/'build'/('replay_'+version+('.exe' if os.name=='nt' else ''))
        command=[str(exe),entry['bin_path'],entry['cfg'],entry['diagnostic_csv'],
                 entry['selections'],str(directory/(stem+'_frames.csv')),
                 '--candidates-output',str(directory/(stem+'_candidates.csv'))]
        command+=['--clusters-output',str(directory/(stem+'_clusters.csv'))]
        if version=='v1': command+=['--legacy-doppler']
        if version=='v3': command+=['--profile',a.profile]
        if version=='v3' and a.gate_extra>=0: command+=['--gate-extra',str(a.gate_extra)]
        if a.cached:
            command+=['--cached-points',str(a.cached_dir/(stem+'_points.csv')), '--cached-frame-count', str(entry['bin_frames'])]
        else: command+=['--points-output',str(directory/(stem+'_points.csv'))]
        if a.dry_run:
            print(json.dumps(command, ensure_ascii=False))
            return dict(version=version, bin=entry['bin'], command=command)
        with (directory/(stem+'_stdout.log')).open('w',encoding='utf-8') as f:
            r=subprocess.run(command,cwd=ROOT,stdout=f,stderr=subprocess.STDOUT,
                             creationflags=subprocess.CREATE_NO_WINDOW if os.name=='nt' else 0)
        if r.returncode: raise RuntimeError(f'Failed {version} {stem}: see {directory}')
        print(f'PASS {version} {stem}',flush=True)
        return dict(version=version,bin=entry['bin'],command=command,exit_code=r.returncode)
    with ThreadPoolExecutor(max_workers=a.jobs) as pool:
        tasks=[pool.submit(run,e,v) for e in prepared for v in a.versions]
        results=[t.result() for t in as_completed(tasks)]
    if a.dry_run: return
    (ROOT/'reports'/('run_'+(a.tag or '_'.join(a.versions))+'.json')).write_text(
        json.dumps(dict(cached=a.cached,profile=a.profile,runs=results),ensure_ascii=False,indent=2),encoding='utf-8')

if __name__=='__main__': main()
