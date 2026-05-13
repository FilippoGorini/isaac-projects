"""
Kinova Gen3 6-DoF + Robotiq 2F-85 policy transforms for pi0.

Modeled on examples/ur5/README.md in openpi (UR5e is also 6-DoF + 1 gripper = 7 action dims).

Observation expected from the ROS 2 client:
  joints       float32 (6,)   arm joint positions in radians
  gripper      float32 (1,)   gripper position (0.0 = open, 1.0 = closed)
  base_rgb     uint8   (H,W,3) external/overhead camera
  wrist_rgb    uint8   (H,W,3) wrist camera
  prompt       str             language instruction

Action returned to the ROS 2 client:
  actions      float32 (horizon, 7)  columns 0:6 = arm joints, column 6 = gripper
"""
