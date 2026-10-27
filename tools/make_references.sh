#!/usr/bin/env bash
# (Heads) Regenerate the BASELINE reference rows of the leaderboard.
# Run inside the sandbox shell:   tools/make_references.sh [mock-all|ifssim-all]
# Real-pipeline rows: tools/benchmark_real.sh on the host.
# Then review results/baseline/ and copy the runs into benchmark/reference/.
set -euo pipefail
SC="${1:-mock-all}"
B=/sandbox/tools/benchmark.py
python3 $B --label baseline --scenario "$SC"                                   # everything baseline
python3 $B --label baseline --scenario "$SC" perception:=gt                    # Level 1 reference
python3 $B --label baseline --scenario "$SC" perception:=gt --gt-noise         # Level 2 reference
if [[ "$SC" == mock-all && -f /sandbox/bags/manual_20260920_154527_indexed.mcap ]]; then
    python3 $B --label baseline --scenario real-bag                            # real car bag
fi
