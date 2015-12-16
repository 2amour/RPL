#!/usr/bin/env python
###########################################################
# Constants for calibrating the robot.
# Author: Andrey Rusakov
# Last modification: October 10, 2014
###########################################################

PROX_MAX_RANGE = [ 0.167,  0.164,  0.178,  0.177,  0.149,   0.13,   0.09];	# maximum range in meters
PROX_MIN_RANGE = [ 0.045,  0.009,  0.012,  0.011,  0.015,   0.01,   0.01];	# minimum range in meters
PROX_MAX_VALUE = [4300.0, 4460.0, 4770.0, 4520.0, 4440.0, 4300.0, 5500.0];	# maximum value
PROX_MIN_VALUE = [1240.0, 1050.0, 1400.0, 1100.0, 1090.0,  800.0, 2600.0];	# minimum value

LEFT_WHEEL_FORWARD_COMPENSATION = 1.00 - 0.083		# change this constant to compensate speed of the left wheel when moving forward
RIGHT_WHEEL_FORWARD_COMPENSATION = 1.00 - 0.081		# change this constant to compensate speed of the right wheel when moving forward

LEFT_WHEEL_BACKWARD_COMPENSATION = 1.00 + 0.05 	# change this constant to compensate speed of the left wheel when moving backward
RIGHT_WHEEL_BACKWARD_COMPENSATION = 1.00 + 0.25	# change this constant to compensate speed of the right wheel when moving backward
