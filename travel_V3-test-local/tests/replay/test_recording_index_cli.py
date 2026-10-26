"""Exercise actual replay/converter executables against an indexed synthetic BIN."""
import csv
from pathlib import Path
import subprocess
import sys
import tempfile


def main():
    replay, converter, cfg = map(lambda value: Path(value).resolve(), sys.argv[1:])
    with tempfile.TemporaryDirectory() as temporary:
        root = Path(temporary)
        binary = root/'indexed.bin'
        binary.write_bytes(bytes(524288*2))
        index = binary.with_name(binary.name+'.frames.csv')
        index.write_text('source_frame,session_id,received_time_sec,sample_time_sec,time_quality\n'
                         '3,99,1791985414.125,1791985414.125,receive_estimate\n'
                         '5,99,1791985414.325,1791985414.325,receive_estimate\n')
        diagnostic=root/'motion.csv'
        diagnostic.write_text('frame,input_path,radar_timestamp\n3,C:\\old\\indexed.bin,2026-10-14 21:43:34.125\n'
                              '5,C:\\old\\indexed.bin,2026-10-14 21:43:34.325\n')
        selections=root/'selection.csv'; selections.write_text('after_frame,x,y\n')
        output=root/'frames.csv'
        command=[str(replay),str(binary),str(cfg),str(diagnostic),str(selections),str(output)]
        subprocess.run(command,cwd=root,check=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
        rows=list(csv.DictReader(output.open()))
        assert [row['frame'] for row in rows]==['3','5'], 'Packed BIN slots replaced original source frames'
        subprocess.run([str(converter),str(binary),str(cfg),'--diagnostic-csv',str(diagnostic),
                        '--output',str(root/'points.csv')],cwd=root,check=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
        index.write_text(index.read_text().splitlines()[0]+'\n')
        failure=subprocess.run(command,cwd=root,stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
        assert failure.returncode!=0 and b'length mismatch' in failure.stdout, 'Corrupt companion index accepted'
    print('PASS: real replay/converter, indexed gaps, Windows provenance paths and corrupt-index rejection')


if __name__=='__main__':main()
