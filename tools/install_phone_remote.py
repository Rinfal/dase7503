#!/usr/bin/env python3
"""Install local phone services without changing network/proxy configuration.

Run in the ROS environment. Manual lift targets require explicit authorization.
New targets are not marked as mechanically verified by installation.
"""
import argparse
import datetime
import json
import os
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "robot_ws/src/logistics_robot"))
from logistics_robot.durability import atomic_write

import yaml


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    authorization = parser.add_mutually_exclusive_group(required=True)
    authorization.add_argument('--services-only', action='store_true', help='Preserve all hardware targets and operator settings')
    authorization.add_argument('--authorize-lift-400-540', action='store_true')
    authorization.add_argument('--authorize-lift-400-750', action='store_true')
    authorization.add_argument('--authorize-lift-300-750', action='store_true')
    authorization.add_argument('--authorize-lift-down-us', type=int, choices=range(300, 401, 5),
                               help='Operator-selected lower target 300..400us in 5us steps; upper target 750us')
    parser.add_argument('--authorize-lift-up-us', type=int, choices=(540, 750, 1050, 1300),
                        help='Explicit upper target override; use with a lift authorization option')
    parser.add_argument('--operator-controls-confirmed', action='store_true')
    parser.add_argument('--direct-control', action='store_true')
    parser.add_argument('--manual-lock-control', action='store_true')
    parser.add_argument('--authorize-mecanum-lateral', action='store_true')
    parser.add_argument('--enable', action='store_true', help='Enable and start the full stack at boot')
    args = parser.parse_args()
    if args.services_only and args.authorize_lift_up_us is not None:
        parser.error('--services-only cannot change lift targets')
    os.umask(0o077)
    root = Path(__file__).resolve().parents[1]
    config = Path.home() / '.config/dase7503'
    config.mkdir(mode=0o700, parents=True, exist_ok=True)
    destination = config / 'phone-profile.yaml'
    profile = yaml.safe_load((destination if destination.exists() else
        root / 'robot_ws/src/logistics_robot/config/hardware_profile.yaml').read_text())
    if args.services_only and not destination.exists():
        raise RuntimeError('Install an authorized hardware profile first')
    if not args.services_only:
        evidence = json.loads((root / 'docs/evidence/web_drive_acceptance.json').read_text())
        directions = {row.get('direction') for row in evidence.get('results', []) if row.get('verified') is True}
        if evidence.get('verified') is not True or directions != {'forward', 'backward', 'ccw', 'cw'}:
            raise RuntimeError('Existing HTTP/board drive acceptance evidence missing')
        profile['controller']['web_drive_validated'] = True
        profile['controller']['web_drive_evidence'] = 'docs/evidence/web_drive_acceptance.json'
        if args.direct_control:
            profile['controller']['direct_control'] = True
            profile['controller']['manual_lock_control'] = False
        if args.manual_lock_control:
            profile['controller']['direct_control'] = False
            profile['controller']['manual_lock_control'] = True
        if args.authorize_mecanum_lateral:
            if profile['controller'].get('wheel_type') != 'mecanum':
                raise RuntimeError('Mecanum wheels required for lateral mixing')
            profile['controller']['manual_lateral_authorized'] = True
            profile['controller']['lateral_authorization_note'] = '2026-11-20: operator requested lateral controls; X-roller mixing, physical left/right response awaits observation.'
        if args.operator_controls_confirmed:
            profile['controller']['ground_drive_verified'] = True
            profile['controller']['operator_confirmation'] = '2026-11-20: operator confirmed all phone controls work and requested removal of trial hold limit.'
            profile['servo']['operator_confirmed_pulses_us'] = {'down': 400, 'up': 540}
        down = args.authorize_lift_down_us or (300 if args.authorize_lift_300_750 else 400)
        up = 750 if args.authorize_lift_400_750 or args.authorize_lift_300_750 or args.authorize_lift_down_us else 540
        if args.authorize_lift_up_us is not None:
            up = args.authorize_lift_up_us
        profile['servo'].update(safe_range_verified=False, manual_lift_authorized=True,
                                manual_pulses_us={'down': down, 'up': up},
                                initial_position_us=down,
                                current_physical_position='not_measured_on_startup',
                                manual_authorization_note='Operator requested manual targets; newly requested travel remains subject to physical observation.')
        if destination.exists():
            stamp = datetime.datetime.now().strftime('%Y%m%d-%H%M%S')
            (config / f'phone-profile.{stamp}.bak').write_bytes(destination.read_bytes())
        atomic_write(destination, yaml.safe_dump(profile, sort_keys=False, allow_unicode=True).encode())
    units = Path.home() / '.config/systemd/user'
    units.mkdir(parents=True, exist_ok=True)
    for kind, script in [('agent', 'start_phone_agent.sh'), ('remote', 'start_phone_web.sh'),
                         ('camera', 'start_phone_camera.sh'), ('vision', 'start_phone_vision.sh'),
                         ('health', 'start_phone_health.sh')]:
        dependencies = ('dase7503-agent.service dase7503-camera.service dase7503-vision.service'
                        if kind in ('remote', 'health') else 'dase7503-camera.service' if kind == 'vision' else '')
        after = f'After={dependencies}\nWants={dependencies}\n' if dependencies else ''
        priority = 'Nice=5\n' if kind in ('camera', 'vision') else ''
        watchdog = 'WatchdogSec=30\nNotifyAccess=main\nWatchdogSignal=SIGKILL\n' if kind == 'health' else ''
        content = f'''[Unit]
Description=DASE7503 phone control {kind}
PartOf=dase7503.target
{after}StartLimitIntervalSec=0

[Service]
Type=simple
WorkingDirectory={root}
ExecStart=/bin/bash {root}/tools/{script}
Environment=ROS_DOMAIN_ID=20
Environment=ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
Environment=PYTHONUNBUFFERED=1
Restart=always
RestartSec=5
KillSignal=SIGINT
TimeoutStopSec=8
UMask=0077
{priority}
{watchdog}

[Install]
WantedBy=dase7503.target
'''
        unit = units / f'dase7503-{kind}.service'
        if unit.exists() and unit.read_text() != content:
            previous = unit.read_text()
            if f'WorkingDirectory={root}' not in previous or 'Description=DASE7503 phone control ' not in previous:
                raise RuntimeError(f'Refusing to overwrite unrelated service: {unit}')
            stamp = datetime.datetime.now().strftime('%Y%m%d-%H%M%S')
            unit.with_suffix(f'.{stamp}.bak').write_text(previous)
        atomic_write(unit, content.encode())
    target = units / 'dase7503.target'
    atomic_write(target, b'''[Unit]
Description=DASE7503 complete phone robot stack
Wants=dase7503-agent.service dase7503-camera.service dase7503-vision.service dase7503-remote.service dase7503-health.service dase7503-boot-check.service

[Install]
WantedBy=default.target
''')
    atomic_write(units/'dase7503-boot-check.service', f'''[Unit]
Description=DASE7503 read-only startup acceptance
PartOf=dase7503.target
After=dase7503-remote.service dase7503-health.service
StartLimitIntervalSec=0

[Service]
Type=oneshot
WorkingDirectory={root}
ExecStart=/usr/bin/python3 {root}/tools/check_phone_boot.py
TimeoutStartSec=120
Restart=on-failure
RestartSec=30
UMask=0077
'''.encode())
    if args.enable:
        subprocess.run(['systemctl', '--user', 'daemon-reload'], check=True)
        subprocess.run(['systemctl', '--user', 'disable',
                        *[f'dase7503-{kind}.service' for kind in ('agent','camera','vision','remote')]], check=True)
        subprocess.run(['systemctl', '--user', 'enable', '--now', 'dase7503.target'], check=True)
    print('Installed five services and dase7503.target. No actuator commands or network changes.')


if __name__ == '__main__':
    main()
