"""Public package exports for the CBTC/ATC simulator."""

import sys
from pathlib import Path

PACKAGE_DIR = Path(__file__).resolve().parent
if str(PACKAGE_DIR) not in sys.path:
    sys.path.insert(0, str(PACKAGE_DIR))

from GUI.app import App
from GUI.main_gui import Simulation, Train, ZoneController
from MONTECARLO.monte_carlo import MonteCarloConfig, run_batch
from OPERATION.headway_manager import HeadwayDecision, HeadwayManager, HeadwayStats
from REPORT.reporting import build_simulation_report, save_simulation_report

__all__ = [
    "App",
    "HeadwayDecision",
    "HeadwayManager",
    "HeadwayStats",
    "MonteCarloConfig",
    "Simulation",
    "Train",
    "ZoneController",
    "build_simulation_report",
    "run_batch",
    "save_simulation_report",
]
