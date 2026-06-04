"""Public package exports for the CBTC/ATC simulator."""

import sys
from pathlib import Path

PACKAGE_DIR = Path(__file__).resolve().parent
if str(PACKAGE_DIR) not in sys.path:
    sys.path.insert(0, str(PACKAGE_DIR))

from GUI.main_gui import App
from GUI.panels.ats_overview_panel import ATSOverviewPanel
from GUI.panels.train_panel import TrainPanel
from GUI.panels.infrastructure_panel import InfrastructurePanel
from GUI.panels.engineering_panel import DataFlowPanel, EngineeringPanel, TimeDistancePanel
from GUI.panels.diagnostics_panel import DiagnosticsPanel
from GUI.panels.analytics_panel import AnalyticsPanel
from GUI.panels.control_panel import ControlPanel, SpeedLimitsPanel
from GUI.panels.monte_carlo_panel import MonteCarloPanel
from GUI.dialogs.scenario_dialog import AddElementDialog
from GUI.dialogs.train_editor_dialog import TrainEditorDialog
from GUI.dialogs.station_editor_dialog import StationEditorDialog
from GUI.dialogs.fault_dialog import FaultDialog
from GUI.widgets.status_card import StatusCard
from GUI.widgets.curve_plot import CurvePlot
from GUI.widgets.table_view import TableView
from GUI.main_gui import Simulation, Train, ZoneController
from MONTECARLO.monte_carlo import MonteCarloConfig, run_batch
from OPERATION.headway_manager import HeadwayDecision, HeadwayManager, HeadwayStats
from REPORT.reporting import build_simulation_report, save_simulation_report

__all__ = [
    "AddElementDialog",
    "AnalyticsPanel",
    "App",
    "ATSOverviewPanel",
    "ControlPanel",
    "CurvePlot",
    "DataFlowPanel",
    "DiagnosticsPanel",
    "EngineeringPanel",
    "FaultDialog",
    "HeadwayDecision",
    "HeadwayManager",
    "HeadwayStats",
    "InfrastructurePanel",
    "MonteCarloConfig",
    "MonteCarloPanel",
    "Simulation",
    "SpeedLimitsPanel",
    "StationEditorDialog",
    "StatusCard",
    "TableView",
    "TimeDistancePanel",
    "Train",
    "TrainEditorDialog",
    "TrainPanel",
    "ZoneController",
    "build_simulation_report",
    "run_batch",
    "save_simulation_report",
]
