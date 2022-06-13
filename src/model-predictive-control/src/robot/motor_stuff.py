from statistics import mode, median, stdev

import numpy as np
from numpy import sign, polyfit, linspace, arange
from scipy.stats import iqr
from sklearn.linear_model import HuberRegressor
from sklearn.preprocessing import StandardScaler

from src.physics.motor import orbital_20_gearmotor
from src.physics.static_parameters import OZF_TO_N, IN_TO_M, RPM_TO_RADS

print(RPM_TO_RADS)
print(orbital_20_gearmotor.torque(12, 0) / (OZF_TO_N * IN_TO_M))

# moment_of_inertia = 3.33e-7
#
import csv
with open('/home/alec/StudioProjects/kuriosity-power-play/motor_data_voltage_12.66.csv', newline='') as csvfile:
    reader = csv.DictReader(csvfile, delimiter=',')

    new_array = []
    last_velo = 999999
    for row in reader:
        if not row['accel']:
            continue

        row = {k: float(v) for k, v in row.items()}
        if row['velo'] != last_velo:
            new_array.append(row)

        last_velo = row['velo']

    x_data = []
    y_data = []
    for i in range(3, len(new_array)):
        if (abs((new_array[i]['velo'] - new_array[i - 1]['velo']) / (new_array[i]['time'] - new_array[i - 1]['time']))) < 15:
            # x_data.append(abs((new_array[i]['velo'] - new_array[i - 1]['velo']) / (new_array[i]['time'] - new_array[i - 1]['time'])))
            x_data.append((new_array[i]['velo']))
            y_data.append((orbital_20_gearmotor.torque(new_array[i]['power'] * 12.66, new_array[i]['velo'])))

    from matplotlib.pyplot import plot, waitforbuttonpress, hist, ylim, scatter

    # fit model
    print(len(x_data))
    print(len(y_data))
    model = HuberRegressor(max_iter=100, epsilon=100)
    model.fit(np.array(x_data).reshape((-1, 1)), np.array(y_data))    # xseq = linspace(0, 5, num=100)

    x = arange(min(x_data), max(x_data))
    print(y_data)
    plot(x, model.predict(x.reshape((-1,1))))
    scatter(x_data, y_data)

    for row in zip(x_data, y_data):
        print(row)

    waitforbuttonpress()
