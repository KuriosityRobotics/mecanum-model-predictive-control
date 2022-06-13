from __future__ import annotations

import math
from types import SimpleNamespace
from typing import TypedDict, Dict


def big_len(obj):
    l = 0
    for k, v in obj.items():
        l += len(v)

    return l


labels = {"objective": ["x", "y", "theta"], "weight": [
    "position",
    "angle",
    "energy",
    "slack",
    "use_begin",
    "obs_margin",
    "strafe",
    "strafeMinAngle",
    "strafeMinVelo",

], "threshold": [
    "distanceThreshold",
    "angleThreshold"
]}

p = SimpleNamespace()

n = 0
for k, v in labels.items():
    indices = SimpleNamespace()

    for attr_name in v:
        setattr(indices, attr_name, n)
        n = n + 1
    setattr(p, k, indices)

class _Index:
    u = [0, 1, 2, 3]
    slack = 4
    qr = [5, 6, 7]
    qr_dot = [8, 9, 10]

    class _x:
        qr = [0, 1, 2]
        qr_dot = [3, 4, 5]

    fromz = [5, 6, 7, 8, 9, 10]
    x = _x()
    p = p


index = _Index()
