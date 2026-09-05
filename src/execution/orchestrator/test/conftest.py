import sys
from pathlib import Path

# Automatically configure sys.path for pytest runs within orchestrator
TEST_DIR = Path(__file__).resolve().parent
ORCHESTRATION_DIR = TEST_DIR.parent
SRC_DIR = ORCHESTRATION_DIR.parent.parent

for path in [str(TEST_DIR), str(ORCHESTRATION_DIR), str(SRC_DIR)]:
    if path not in sys.path:
        sys.path.insert(0, path)
