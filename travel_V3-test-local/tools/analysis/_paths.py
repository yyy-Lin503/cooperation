from pathlib import Path
import sys
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tools/replay'))
sys.path.insert(0, str(ROOT / 'ros2_ws/src/my_radar_follower'))
from my_radar_follower.runtime_support import runtime_paths
PATHS = runtime_paths(ROOT)
