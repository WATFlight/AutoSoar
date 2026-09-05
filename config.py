"""Central configuration for the ArduSoar simulation.

All tunable numbers live here so the rest of the code stays readable. Units are
SI (metres, seconds, radians) unless a name says otherwise (``*_deg``).
"""

import math

# --- Physics ---------------------------------------------------------------
G = 9.81

# --- Simulation timing -----------------------------------------------------
DT = 0.1
SIM_TIME = 400.0

# --- Glider ----------------------------------------------------------------
AIRSPEED = 16.0
BASE_SINK_RATE = 0.7
MAX_BANK_DEG = 45.0

START_X = 0.0
START_Y = 0.0
START_H = 300.0
START_HEADING = math.radians(45.0)

# --- True thermal (unknown to the controller) ------------------------------
THERMAL_X = 200.0
THERMAL_Y = 200.0
THERMAL_W0 = 3.5
THERMAL_R = 50.0

# --- Cruise waypoint -------------------------------------------------------
WAYPOINT_X = 400.0
WAYPOINT_Y = 400.0

# --- Thermal estimator -----------------------------------------------------
WINDOW_SIZE = 50
MIN_POINTS_TO_FIT = 25
DETECT_LIFT_THRESHOLD = 0.6
MIN_THERMAL_STRENGTH = 0.6
ESTIMATE_EVERY = 5

LAMBDA_W0 = 0.05
LAMBDA_R = 0.01
LAMBDA_POS = 0.02

# --- State machine thresholds ----------------------------------------------
PROBE_THRESHOLD = 0.2
THERMAL_THRESHOLD = 0.5

# --- L1 guidance -----------------------------------------------------------
L1_DISTANCE = 30.0
