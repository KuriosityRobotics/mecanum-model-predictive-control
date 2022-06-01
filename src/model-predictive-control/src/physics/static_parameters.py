import math

import numpy as np

IN_TO_M = 0.0254
MM_TO_M = 0.001

G_TO_KG = 1 / 1000
OZF_TO_N = 1 / 3.5969

RPM_TO_RADS = 2 * math.pi / 60

L = 7.802 * IN_TO_M
l = 4.822 * IN_TO_M
Ll = L + l
r = 50 * MM_TO_M
m = 12.24  # drivetrain
Iz = 333.151959 * G_TO_KG
Iw = 0.3712 * G_TO_KG  # drivetrain
Km = 1.23577 / 11.5  # 1.23577 Newton-metres per amp
Rm = 12 / 11.5
N = 19.2  # 1 / reduciton ratio
nu = .6  # pinion module??  idfk

# friction
a = np.array([.0067, .0291, .0067, .0144]).reshape(4, 1)
b = np.array([.5373, .4540, .3423, .2641]).reshape(4, 1)
