#!/usr/bin/env python3
"""Run the same official-SDK adapter installed for navigation, without a default localization gate."""
from pathlib import Path
import runpy

if __name__ == '__main__':
    runpy.run_path(str(Path(__file__).resolve().parents[1]
                      / 'src/robot/go2_twist_bridge/scripts/sdk_twist_bridge.py'),
                   run_name='__main__')
