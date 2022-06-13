import math

IN_TO_M = 0.0254
MM_TO_M = 0.001

G_TO_KG = 1 / 1000
OZF_TO_N = 1 / 3.5969

RPM_TO_RADS = 2 * math.pi / 60

L = 7.802 * IN_TO_M
l = 4.822 * IN_TO_M
Ll = L + l
r = 25 * MM_TO_M
m = 12.24  # drivetrain
Iz = 333.151959 * G_TO_KG
Iw = 0.3712 * G_TO_KG  # drivetrain

G = 9.807
