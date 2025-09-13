import numpy as np
from python_motion_planning.utils.environment.env import Mountain

env = Mountain(elevation_weight=10.1, elevation_scale=10.0)
print("dtype, shape:", env.z.dtype, env.z.shape)
print("min/max/mean/std:", np.nanmin(env.z), np.nanmax(env.z), np.nanmean(env.z), np.nanstd(env.z))
# show a few sample values
print("samples:", env.z.ravel()[:20])
# histogram to see distribution
import matplotlib.pyplot as plt
plt.hist(env.z.ravel(), bins=80)
plt.title("DEM value distribution")
plt.show()