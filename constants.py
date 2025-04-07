import numpy as np

SPRING_DEFLECTION_FOR_ZERO_IMPEDANCE = 20 # deg
MIN_EXO_TORQUE = 1.5
MAX_EXO_TORQUE = 50

EXO_LEVER_ARM = 0.165 # mm
ANKLE_PTS = np.array([-27.71,-21.98,-15.68,-4.79,13.54,18.70,23.29,26.15,29.01])  # Deg
TORQUE_MULTIPLIER_PTS = np.array([66.72,68.20,69.50,70.64,64.08,55.38,39.54,23.65,3.58])  # Nm/Nm