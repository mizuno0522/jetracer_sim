#!/bin/bash
# URP 版のプロジェクト unity/MinicarSimURP を作ってビルド・配置する (中身は migrate_pipeline.sh。docs/hdrp.md)。
#   ./scripts/migrate_urp.sh [--shots]       STEPS="3 install" ./scripts/migrate_urp.sh
exec "$(cd "$(dirname "$0")" && pwd)/migrate_pipeline.sh" urp "$@"
