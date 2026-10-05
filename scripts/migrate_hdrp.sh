#!/bin/bash
# HDRP 版のプロジェクト unity/MinicarSimHDRP を作ってビルド・配置する (中身は migrate_pipeline.sh。docs/hdrp.md)。
#   ./scripts/migrate_hdrp.sh [--shots]      STEPS="3 install" ./scripts/migrate_hdrp.sh
exec "$(cd "$(dirname "$0")" && pwd)/migrate_pipeline.sh" hdrp "$@"
