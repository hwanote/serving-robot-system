"""Explicit conversion at the JSON boundary for sensor-derived NumPy scalars."""
import json
import numpy as np
def native(value):
    if isinstance(value,np.generic):return value.item()
    if isinstance(value,np.ndarray):return value.tolist()
    raise TypeError("Unsupported sensor payload type: "+type(value).__name__)
def dumps(value):return json.dumps(value,default=native)
