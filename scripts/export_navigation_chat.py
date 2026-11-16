#!/usr/bin/env python3
"""Export visible user/assistant dialogue from the named local Codex thread."""

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path

THREAD_ID = '01a1a001-0f1e-7333-a446-8b1a19cd30b3'
DEFAULT_SOURCE = Path('/home/a/.codex/sessions/2026/11/03') / (
    f'rollout-2026-11-03T14-03-06-{THREAD_ID}.jsonl')
DEFAULT_OUTPUT = Path(__file__).resolve().parents[1] / 'conversations' / f'{THREAD_ID}.md'


def export(source: Path, output: Path):
    if not source.is_file():
        raise FileNotFoundError(source)
    entries = []
    skipped_incomplete = 0
    with source.open(encoding='utf-8') as stream:
        for line in stream:
            try:
                record = json.loads(line)
            except json.JSONDecodeError:
                skipped_incomplete += 1
                continue
            payload = record.get('payload', {})
            if record.get('type') != 'response_item' or payload.get('type') != 'message':
                continue
            role = payload.get('role')
            if role not in ('user', 'assistant'):
                continue
            parts = [item['text'] for item in payload.get('content', [])
                     if item.get('type') in ('input_text', 'output_text') and item.get('text')]
            if not parts:
                continue
            message = '\n\n'.join(parts)
            if role == 'user' and message.lstrip().startswith('<environment_context>'):
                continue
            entries.append((record.get('timestamp', ''), role, message))
    exported_at = datetime.now(timezone.utc).isoformat(timespec='seconds')
    lines = [f'# 聊天记录：{THREAD_ID}', '',
             f'- 来源：本地 Codex 会话存档 `{THREAD_ID}`',
             f'- 导出时间（UTC）：{exported_at}',
             f'- 消息数：{len(entries)}（仅用户与助手可见消息）',
             '- 范围：截至导出时存档中完整写入的消息；原会话可能继续更新。',
             '- 未包含：内部推理、工具调用、工具输出、系统配置和运行环境消息。',
             '']
    for index, (stamp, role, message) in enumerate(entries, 1):
        label = '用户' if role == 'user' else '助手'
        lines.extend((f'## {index:04d} · {label} · {stamp}', '', message, ''))
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text('\n'.join(lines), encoding='utf-8')
    print(f'{output}: {len(entries)} messages; {skipped_incomplete} incomplete lines skipped')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, default=DEFAULT_SOURCE)
    parser.add_argument('--output', type=Path, default=DEFAULT_OUTPUT)
    options = parser.parse_args()
    export(options.source, options.output)
