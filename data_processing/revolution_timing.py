# make an array at 0.1 increments from 0.1 to 10.0 corresponding to rpm
# change the speed from miles to inches to go from mph to revs per sec
# divide by circumference of motor to find timing for 1 rev at each rpm

import math as mth
import numpy as np


if __name__ == "__main__":

    converted_speed = 0.0
    speed_array_values = np.arange(0.1,10.1, 0.1)
    rps_array_values = []
    rpm_array_values = []
    one_rev_time = []

    miles_to_inches = 63360
    hours_to_seconds = 3600
    diameter = 1.92
    circumference = mth.pi * diameter

    for i in speed_array_values:
        converted_speed = round(i * miles_to_inches / (hours_to_seconds * circumference), 3)
        rps_array_values.append(converted_speed)
        rpm_array_values.append(converted_speed * 60)
        one_rev_time.append(round(1/converted_speed, 6))

    print("Sp:" , speed_array_values, "\n")
    print("RPS: ", rps_array_values, "\n")
    print("RPM: ", rpm_array_values, "\n")
    print("On Tim: ", one_rev_time)
    