#!/usr/bin/env python3
"""Record startup acceptance without publishing any drive or lift commands."""
import http.cookiejar
import json
import subprocess
import sys
import time
import urllib.request
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'robot_ws/src/logistics_robot'))
from logistics_robot.durability import atomic_write


def main():
    report = {'boot_id':Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
              'checked_at_utc':datetime.now(timezone.utc).isoformat(), 'verified':False}
    deadline = time.monotonic()+90
    base = 'http://10.42.50.1:8080'
    while time.monotonic()<deadline:
        try:
            client = urllib.request.build_opener(urllib.request.ProxyHandler({}),
                urllib.request.HTTPCookieProcessor(http.cookiejar.CookieJar()))
            password = (Path.home()/'.config/dase7503/remote-password').read_text().strip()
            with client.open(urllib.request.Request(base+'/api/login',
                data=json.dumps({'password':password}).encode(),
                headers={'Content-Type':'application/json'}),timeout=2) as response:
                response.read()
            with client.open(base+'/api/status',timeout=2) as response:
                status = json.load(response)
            assert not status['armed'] and status.get('board_armed') is False, 'startup must stay locked'
            assert status.get('wheel_pwm') == [0]*4, 'startup must be neutral'
            assert status['feedback_age'] is not None and status['feedback_age']<.5, 'board not ready'
            assert status['camera_age'] is not None and status['camera_age']<1, 'camera not ready'
            sequences=[]
            with client.open(base+'/api/video',timeout=3) as response:
                assert response.headers.get_content_type()=='application/octet-stream'
                start=time.monotonic()
                while len(sequences)<15 and time.monotonic()-start<5:
                    assert response.readline()==b'--frame\r\n'
                    headers={}
                    while True:
                        line=response.readline()
                        if line==b'\r\n':break
                        if not line:raise OSError('video ended')
                        key,value=line.decode().split(':',1);headers[key.lower()]=value.strip()
                    length=int(headers['content-length'])
                    assert 2<=length<=1024*1024
                    jpeg=response.read(length)
                    assert jpeg.startswith(b'\xff\xd8') and jpeg.endswith(b'\xff\xd9')
                    assert response.read(2)==b'\r\n'
                    sequences.append(int(headers['x-frame-sequence']))
                assert len(sequences)>=5 and len(set(sequences))==len(sequences), 'video not advancing'
            units={kind:subprocess.check_output(['systemctl','--user','show',
                f'dase7503-{kind}.service','-p','ActiveState','-p','MainPID'],text=True).strip()
                for kind in ('agent','camera','vision','remote','health')}
            assert all('ActiveState=active' in value for value in units.values()), 'service not active'
            report.update(verified=True, uptime_seconds=float(Path('/proc/uptime').read_text().split()[0]),
                units=units, video_frames=len(sequences),
                video_fps=round(len(sequences)/(time.monotonic()-start),2),
                status={key:status.get(key) for key in ('armed','board_armed','wheel_pwm',
                    'feedback_age','camera_age','lift_pulses_us','lift_command_us')})
            break
        except (OSError,ValueError,AssertionError,subprocess.SubprocessError) as error:
            report['last_error']=str(error)
            time.sleep(1)
    if report['verified']:
        report.pop('last_error',None)
    directory=ROOT/'runtime';directory.mkdir(exist_ok=True)
    request_file=directory/'reboot-test-request.json'
    if request_file.exists():
        requested=json.loads(request_file.read_text())
        report['previous_boot_id']=requested.get('before_boot_id')
        report['new_boot_verified']=bool(report['verified'] and report['previous_boot_id']
            and report['previous_boot_id']!=report['boot_id'])
    data=(json.dumps(report,indent=2)+'\n').encode()
    atomic_write(directory/'boot-check.json',data)
    atomic_write(directory/f"boot-check-{report['boot_id']}.json",data)
    print(json.dumps(report))
    return 0 if report['verified'] else 1


if __name__=='__main__':
    raise SystemExit(main())
