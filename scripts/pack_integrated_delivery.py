#!/usr/bin/env python3
"""Package the latest robot report, simulation plan and runnable simulation."""

import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import tarfile
import tempfile


ROOT = Path(__file__).resolve().parents[1]
WORKSPACE = ROOT.parent
PACKAGE_NAME = 'robot_nav_sim_delivery_20261106'


def digest(path):
    sha = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            sha.update(block)
    return sha.hexdigest()


def build(output):
    navigation_archive = ROOT / 'dist/gz_patrol_nav.tar.gz'
    real_report = WORKSPACE / 'docs/reports/2026-11-06_robot_status/报告.md'
    status = real_report.with_name('status_snapshot.json')
    sim_report = WORKSPACE / 'docs/reports/2026-11-06_simulation_plan/仿真模拟方案报告.md'
    for path in (navigation_archive, real_report, status, sim_report):
        if not path.is_file():
            raise FileNotFoundError(path)

    output = Path(output).resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='robot_nav_delivery_') as folder:
        stage = Path(folder) / PACKAGE_NAME
        simulation = stage / 'simulation'
        simulation.mkdir(parents=True)
        with tarfile.open(navigation_archive, 'r:gz') as source:
            for member in source:
                if not member.isfile():
                    raise ValueError(f'Unexpected non-file in navigation package: {member.name}')
                relative = Path(member.name)
                if relative.is_absolute() or '..' in relative.parts:
                    raise ValueError(f'Unsafe navigation package path: {member.name}')
                target = simulation / relative
                target.parent.mkdir(parents=True, exist_ok=True)
                with source.extractfile(member) as item, target.open('wb') as destination:
                    shutil.copyfileobj(item, destination)
                os.chmod(target, member.mode & 0o777)

        reports = stage / 'reports'
        reports.mkdir()
        shutil.copy2(real_report, reports / '最新真机报告.md')
        shutil.copy2(status, reports / '真机状态快照.json')
        shutil.copy2(sim_report, reports / '仿真模拟方案报告.md')
        readme = stage / 'README_交付说明.md'
        readme.write_text(
            '# 导航仿真与报告整合包\n\n'
            '内容：`reports/最新真机报告.md`、`reports/真机状态快照.json`、'
            '`reports/仿真模拟方案报告.md`，以及可独立运行的 `simulation/`。'
            '仿真目录还包含模型、地图、测试场景、配置、脚本、Dockerfile、最新地图的回环 PCD/NPZ、历史聊天快照和自身文件清单。'
            '834 MB 的原始传感器 rosbag 未包含。\n\n'
            '先在压缩包旁运行 `sha256sum -c robot_nav_sim_delivery_20261106.tar.gz.sha256`，'
            '再解压。Ubuntu 22.04、ROS 2 Humble、Gazebo Fortress 等依赖见 '
            '`simulation/SERVER_NAV_README.md`。进入 `simulation/` 后执行 '
            '`./run_server_nav.sh narrow_turns` 做控制基准，或执行 '
            '`./run_server_nav.sh recording_20261106_163030` 查看新地图诊断场景。'
            '两个入口均不自动发送导航目标。\n\n'
            '真机报告记录了碰墙事件和自动导航关闭状态；仿真新地图由栅格拉伸生成，'
            '不能充当缺失墙体的独立物理真值。整合包仅供服务器仿真和分析。\n',
            encoding='utf-8')

        files = []
        for path in sorted(p for p in stage.rglob('*') if p.is_file()):
            files.append({'path': path.relative_to(stage).as_posix(),
                          'size': path.stat().st_size, 'sha256': digest(path)})
        manifest = {'package': PACKAGE_NAME, 'source_navigation_archive_sha256': digest(navigation_archive),
                    'files': files}
        (stage / 'DELIVERY_MANIFEST.json').write_text(
            json.dumps(manifest, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
        with tarfile.open(output, 'w:gz') as archive:
            for path in sorted(p for p in stage.rglob('*') if p.is_file()):
                archive.add(path, arcname=(Path(PACKAGE_NAME) / path.relative_to(stage)).as_posix())
    checksum = output.with_name(output.name + '.sha256')
    checksum.write_text(f'{digest(output)}  {output.name}\n', encoding='utf-8')
    print(output)
    print(checksum)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', nargs='?', default=str(ROOT / 'dist' / f'{PACKAGE_NAME}.tar.gz'))
    build(parser.parse_args().output)
