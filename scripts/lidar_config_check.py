# import os 
# paths = [
# 	"/isaacsim/exts/isaacsim.sensors.rtx/data/lidar_configs/Ouster",
# 	"/isaacsim/exts/omni.sensors.rtx/data/lidar_configs/Ouster",
# ]
# for path in paths:
# 	if os.path.exists(path):
# 		print(f"\nFound at: {path}")
# 		for f in sorted(os.listdir(path)):
# 			print(f"  {f}")


from omni.isaac.sensor import LidarRtx                                                                                                                                                                     
from omni.isaac.core.utils.stage import get_current_stage
import omni.replicator.core as rep

LIDAR_PATH = "/World/spot/body/sensor/lidar"

# 1. Check if the prim exists
stage = get_current_stage()
prim = stage.GetPrimAtPath(LIDAR_PATH)
print(f"Prim valid: {prim.IsValid()}")
print(f"Prim type: {prim.GetTypeName()}")

# 2. Check render product path
lidar = LidarRtx(prim_path=LIDAR_PATH)
try:
    rp_path = lidar.get_render_product_path()
    print(f"get_render_product_path() returned: {rp_path}")
except Exception as e:
    print(f"get_render_product_path() FAILED: {type(e).__name__}: {e}")

# 3. Check what render products exist in the stage
import omni.usd
ctx = omni.usd.get_context()
stage2 = ctx.get_stage()
for prim2 in stage2.Traverse():
    if "RenderProduct" in prim2.GetPath().pathString:
        print(f"Found render product: {prim2.GetPath()}")