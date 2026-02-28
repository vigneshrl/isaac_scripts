#!/usr/bin/env python3
"""
Isaac Sim 4.5 — DR.Nav Sensor Setup Script
============================================
Run from: Isaac Sim → Window → Script Editor → Run

Prerequisites:
  - USD file open with /World/jackal, /World/flat_plane, /World/wall0..2
  - isaacsim.ros2.bridge extension enabled (Window → Extensions → search ROS2)

Topics published:
  /argus/ar0234_front_left/image_raw   10 Hz  RGB image  (infer_vis.py)
  /argus/ar0234_side_left/image_raw    10 Hz  RGB image  (infer_vis.py)
  /argus/ar0234_side_right/image_raw   10 Hz  RGB image  (infer_vis.py)
  /os_cloud_node/points                20 Hz  PointCloud2 (pointcloud_segmenter.py)
  /odom_lidar                          60 Hz  Odometry   (goal_generator.py)
"""

# ═══════════════════════════════════════════════════════════════════════════════
# SECTION 0 — IMPORTS
# ═══════════════════════════════════════════════════════════════════════════════
import omni
import omni.graph.core as og
from omni.isaac.core import World
from omni.isaac.core.utils.stage import get_current_stage
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.core.controllers import BaseController
from omni.isaac.sensor import Camera
from pxr import Gf, UsdGeom
import omni.replicator.core as rep
import numpy as np
import omni.timeline


# ═══════════════════════════════════════════════════════════════════════════════
# SECTION 0.5 — JACKAL CONTROLLER
# ═══════════════════════════════════════════════════════════════════════════════
class Jackal_controller(BaseController):
    def __init__(self):
        super().__init__(name="Jackal_controller")
        # Open-loop unicycle model
        self._wheel_radius = 0.098
        self._wheel_base   = 0.37559

    def forward(self, command):
        # command[0] = forward velocity (m/s), command[1] = angular velocity (rad/s)
        joint_velocities = [0.0, 0.0, 0.0, 0.0]
        joint_velocities[0] = ((2 * command[0]) - (command[1] * self._wheel_base)) / (2 * self._wheel_radius)
        joint_velocities[1] = ((2 * command[0]) + (command[1] * self._wheel_base)) / (2 * self._wheel_radius)
        joint_velocities[2] = ((2 * command[0]) - (command[1] * self._wheel_base)) / (2 * self._wheel_radius)
        joint_velocities[3] = ((2 * command[0]) + (command[1] * self._wheel_base)) / (2 * self._wheel_radius)
        return ArticulationAction(joint_velocities=joint_velocities)
    
# ═══════════════════════════════════════════════════════════════════════════════
# SECTION 1 — ENABLE EXTENSIONS (must come before everything else)
# ═══════════════════════════════════════════════════════════════════════════════
manager = omni.kit.app.get_app().get_extension_manager()
for ext in ["isaacsim.core.nodes", "isaacsim.ros2.bridge"]:
    if not manager.is_extension_enabled(ext):
        manager.set_extension_enabled_immediate(ext, True)
        print(f"[DR.Nav] Enabled: {ext}")
    else:
        print(f"[DR.Nav] Already enabled: {ext}")

# ═══════════════════════════════════════════════════════════════════════════════
# SECTION 2 — CONSTANTS
# ═══════════════════════════════════════════════════════════════════════════════
ROBOT_PRIM  = "/World/jackal"
ODOM_PRIM   = "/World/jackal/base_link"
CAM_FL_PATH = f"{ROBOT_PRIM}/base_link/cam_front"
CAM_SL_PATH = f"{ROBOT_PRIM}/base_link/cam_side_left"
CAM_SR_PATH = f"{ROBOT_PRIM}/base_link/cam_side_right"
LIDAR_PATH  = f"{ROBOT_PRIM}/base_link/lidar"
GRAPH       = "/World/DRNav_Graph"

TOPIC_CAM_FL = "/argus/ar0234_front_left/image_raw"
TOPIC_CAM_SL = "/argus/ar0234_side_left/image_raw"
TOPIC_CAM_SR = "/argus/ar0234_side_right/image_raw"
TOPIC_LIDAR  = "/os_cloud_node/points"
TOPIC_ODOM   = "/odom_lidar"

CAM_W, CAM_H = 640, 480

stage = get_current_stage()
print("[DR.Nav] Stage:", stage.GetRootLayer().identifier)

# ═══════════════════════════════════════════════════════════════════════════════
# SECTION 2.5 — CLEANUP (safe to re-run: remove old prims before recreating)
# ═══════════════════════════════════════════════════════════════════════════════
# 1. Remove sensor/graph prims
for _path in [CAM_FL_PATH, CAM_SL_PATH, CAM_SR_PATH, LIDAR_PATH, GRAPH]:
    if stage.GetPrimAtPath(_path).IsValid():
        stage.RemovePrim(_path)
        print(f"[DR.Nav] Removed existing prim: {_path}")

# 2. Remove stale Replicator render products — these become orphaned when sensor
#    prims are deleted, causing "[omni.hydra] Invalid USD RenderProduct" errors.
_rp_parent = stage.GetPrimAtPath("/Render/OmniverseKit/HydraTextures")
if _rp_parent.IsValid():
    for _child in _rp_parent.GetChildren():
        stage.RemovePrim(_child.GetPath())
        print(f"[DR.Nav] Removed stale render product: {_child.GetPath()}")

# ═══════════════════════════════════════════════════════════════════════════════
# SECTION 3 — ADD CAMERAS
# ═══════════════════════════════════════════════════════════════════════════════
def add_camera(path, translate, rotate_xyz_deg):
    cam = Camera(
        prim_path=path,
        name=path.split("/")[-1],
        translation=translate,
        orientation=None,
        frequency=10,
        resolution=(CAM_W, CAM_H),
    )
    cam.initialize()
    xform = UsdGeom.XformCommonAPI(stage.GetPrimAtPath(path))
    xform.SetTranslate(translate)
    xform.SetRotate(rotate_xyz_deg, UsdGeom.XformCommonAPI.RotationOrderXYZ)
    print(f"[DR.Nav] Camera: {path}")
    return cam

# Jackal params
cam_fl = add_camera(CAM_FL_PATH, Gf.Vec3d( 0.40,  0.15, 0.30), Gf.Vec3f(0, 0,   0))
cam_sl = add_camera(CAM_SL_PATH, Gf.Vec3d( 0.00,  0.20, 0.30), Gf.Vec3f(0, 0,  90))
cam_sr = add_camera(CAM_SR_PATH, Gf.Vec3d( 0.00, -0.20, 0.30), Gf.Vec3f(0, 0, -90))

# ═══════════════════════════════════════════════════════════════════════════════
# SECTION 4 — ADD LiDAR + GET ITS RENDER PRODUCT PATH
# ═══════════════════════════════════════════════════════════════════════════════
# KEY DISCOVERY: in Isaac Sim 4.5, ROS2RtxLidarHelper uses inputs:renderProductPath
# (NOT inputs:lidarPrimPath which no longer exists).
# path=LIDAR_PATH creates the sensor at the exact path so cleanup works on re-run.

lidar_config = "Velodyne_VLS128"
_, sensor = omni.kit.commands.execute(
    "IsaacSensorCreateRtxLidar",
    path=LIDAR_PATH,
    parent=None,
    config=lidar_config,
    translation=(0.3, 0.0, 0.1),
    orientation=Gf.Quatd(1, 0, 0, 0),
)

render_product = rep.create.render_product(sensor.GetPath(), [1, 1])

# Annotator to read the data
annotator = rep.AnnotatorRegistry.get_annotator("RtxSensorCpuIsaacCreateRTXLidarScanBuffer")
annotator.attach(render_product)

# Debug draw writer
writer = rep.writers.get("RtxLidarDebugDrawPointCloudBuffer")
writer.attach(render_product)
print(f"[DR.Nav] LiDAR render product: {render_product.path}")

# ═══════════════════════════════════════════════════════════════════════════════
# SECTION 5 — CAMERA RENDER PRODUCTS
# ═══════════════════════════════════════════════════════════════════════════════
rp_fl = rep.create.render_product(CAM_FL_PATH, (CAM_W, CAM_H))
rp_sl = rep.create.render_product(CAM_SL_PATH, (CAM_W, CAM_H))
rp_sr = rep.create.render_product(CAM_SR_PATH, (CAM_W, CAM_H))
print("[DR.Nav] Camera render products created")

# ═══════════════════════════════════════════════════════════════════════════════
# SECTION 6 — SINGLE OMNIGRAPH FOR ALL SENSORS
# ═══════════════════════════════════════════════════════════════════════════════
# Attribute names confirmed from discover_attrs.py output:
#
#   Node                      Attribute we use             Discovered name
#   ─────────────────────────────────────────────────────────────────────
#   ROS2RtxLidarHelper        inputs:renderProductPath  ✓  (NOT lidarPrimPath)
#   ROS2RtxLidarHelper        inputs:type               ✓  "point_cloud"
#   ROS2RtxLidarHelper        inputs:topicName          ✓
#   ROS2RtxLidarHelper        inputs:fullScan           ✓
#   ROS2RtxLidarHelper        inputs:execIn             ✓
#   ROS2PublishOdometry       inputs:topicName          ✓  (NOT odomTopicName)
#   ROS2PublishOdometry       inputs:chassisFrameId     ✓
#   ROS2PublishOdometry       inputs:odomFrameId        ✓
#   ROS2PublishOdometry       inputs:execIn             ✓
#   ROS2CameraHelper          inputs:renderProductPath  ✓
#   ROS2CameraHelper          inputs:topicName          ✓
#   ROS2CameraHelper          inputs:type               ✓
#   ROS2CameraHelper          inputs:execIn             ✓
#   IsaacComputeOdometry      inputs:chassisPrimPath    ✓  set to ROBOT_PRIM
#   IsaacReadSimulationTime   NO execIn                 ✓  (pure data node — do not wire tick to it)

og.Controller.edit(
    {"graph_path": GRAPH, "evaluator_name": "execution"},
    {
        og.Controller.Keys.CREATE_NODES: [
            # Clock source — fires every sim step
            ("tick",        "omni.graph.action.OnPlaybackTick"),

            # Sim time — pure data node, no execIn, auto-updates each frame
            ("simtime",     "isaacsim.core.nodes.IsaacReadSimulationTime"),

            # Cameras
            ("camfl",       "isaacsim.ros2.bridge.ROS2CameraHelper"),
            ("camsl",       "isaacsim.ros2.bridge.ROS2CameraHelper"),
            ("camsr",       "isaacsim.ros2.bridge.ROS2CameraHelper"),

            # LiDAR
            ("lidar",       "isaacsim.ros2.bridge.ROS2RtxLidarHelper"),

            # Odometry
            ("odomcompute", "isaacsim.core.nodes.IsaacComputeOdometry"),
            ("odompub",     "isaacsim.ros2.bridge.ROS2PublishOdometry"),
        ],

        og.Controller.Keys.CONNECT: [
            # ── tick → all execution-triggered nodes ─────────────────────────
            # (simtime is NOT here — it has no execIn, it's a pure data node)
            ("tick.outputs:tick",   "camfl.inputs:execIn"),
            ("tick.outputs:tick",   "camsl.inputs:execIn"),
            ("tick.outputs:tick",   "camsr.inputs:execIn"),
            ("tick.outputs:tick",   "lidar.inputs:execIn"),
            ("tick.outputs:tick",   "odomcompute.inputs:execIn"),
            ("tick.outputs:tick",   "odompub.inputs:execIn"),

            # ── simtime → odom publisher (no exec needed, just data flow) ────
            ("simtime.outputs:simulationTime",       "odompub.inputs:timeStamp"),

            # ── odom compute → odom publish ───────────────────────────────────
            ("odomcompute.outputs:position",         "odompub.inputs:position"),
            ("odomcompute.outputs:orientation",      "odompub.inputs:orientation"),
            ("odomcompute.outputs:linearVelocity",   "odompub.inputs:linearVelocity"),
            ("odomcompute.outputs:angularVelocity",  "odompub.inputs:angularVelocity"),
        ],

        og.Controller.Keys.SET_VALUES: [
            # ── Cameras ──────────────────────────────────────────────────────
            ("camfl.inputs:renderProductPath", rp_fl.path),
            ("camfl.inputs:topicName",         TOPIC_CAM_FL),
            ("camfl.inputs:type",              "rgb"),

            ("camsl.inputs:renderProductPath", rp_sl.path),
            ("camsl.inputs:topicName",         TOPIC_CAM_SL),
            ("camsl.inputs:type",              "rgb"),

            ("camsr.inputs:renderProductPath", rp_sr.path),
            ("camsr.inputs:topicName",         TOPIC_CAM_SR),
            ("camsr.inputs:type",              "rgb"),

            # ── LiDAR ─────────────────────────────────────────────────────────
            # renderProductPath (not lidarPrimPath — confirmed by discover_attrs.py)
            # type "point_cloud" → publishes sensor_msgs/PointCloud2
            ("lidar.inputs:renderProductPath", render_product.path),
            ("lidar.inputs:topicName",         TOPIC_LIDAR),
            ("lidar.inputs:type",              "point_cloud"),
            ("lidar.inputs:fullScan",          True),

            # ── Odometry ─────────────────────────────────────────────────────
            # topicName (not odomTopicName — confirmed by discover_attrs.py)
            # chassisFrameId = "base_link" (Jackal body link, not "body" which is Spot)
            ("odomcompute.inputs:chassisPrimPath", ODOM_PRIM),
            ("odompub.inputs:topicName",       TOPIC_ODOM),
            ("odompub.inputs:chassisFrameId",  "base_link"),
            ("odompub.inputs:odomFrameId",     "odom"),
        ],
    }
)
print(f"[DR.Nav] OmniGraph built: {GRAPH}")

# ═══════════════════════════════════════════════════════════════════════════════
# SECTION 7 — DIFFERENTIAL CONTROLLER
# ═══════════════════════════════════════════════════════════════════════════════
# Module-level setup — no class needed in Script Editor context.
_world = World.instance()
if _world is not None:
    _jackal = _world.scene.get_object("jackal_robot")
    _my_controller = Jackal_controller()

    def _send_robot_actions(step_size):
        _jackal.apply_action(_my_controller.forward(command=[0.20, np.pi / 4]))

    _world.add_physics_callback("sending_actions", callback_fn=_send_robot_actions)
    print("[DR.Nav] Physics callback registered")
else:
    print("[DR.Nav] WARNING: World not initialized — skipping controller setup")

# ═══════════════════════════════════════════════════════════════════════════════
# SECTION 8 — START SIMULATION
# ═══════════════════════════════════════════════════════════════════════════════
omni.timeline.get_timeline_interface().play()
print("[DR.Nav] Simulation playing")

print("""
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Verify in a terminal:
  source /opt/ros/humble/setup.bash
  ros2 topic list
  ros2 topic hz /argus/ar0234_front_left/image_raw   # expect ~10 Hz
  ros2 topic hz /os_cloud_node/points                # expect ~20 Hz
  ros2 topic hz /odom_lidar                          # expect ~60 Hz

Then launch DR.Nav:
  ros2 launch map_contruct mapless.launch.py method:=dram
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
""")
