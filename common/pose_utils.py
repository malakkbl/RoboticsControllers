"""Turn a ROS pose message into the [x, y, theta] the controllers expect.
No ROS import here (it only reads attributes), so it can be tested on a laptop without ROS."""
import math

# message kinds supported by ros/pid_node.py  ->  (python module, class name)
POSE_MSG_TYPES = {
    "PoseStamped":               ("geometry_msgs.msg", "PoseStamped"),
    "Pose":                      ("geometry_msgs.msg", "Pose"),
    "PoseWithCovarianceStamped": ("geometry_msgs.msg", "PoseWithCovarianceStamped"),
    "Odometry":                  ("nav_msgs.msg", "Odometry"),
    "Pose2D":                    ("geometry_msgs.msg", "Pose2D"),   # has a plain angle, no quaternion
}


def yaw_from_quat(x, y, z, w):
    """Heading (rotation about z, radians in [-pi, pi]) from a quaternion."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def extract_pose(msg, kind):
    """Return (x, y, theta) from a message of type `kind` (a key of POSE_MSG_TYPES)."""
    if kind == "Pose2D":
        return float(msg.x), float(msg.y), float(msg.theta)
    if kind == "PoseStamped":
        pose = msg.pose
    elif kind == "Pose":
        pose = msg
    elif kind in ("PoseWithCovarianceStamped", "Odometry"):
        pose = msg.pose.pose
    else:
        raise ValueError(f"unknown pose message kind {kind!r}; use one of {list(POSE_MSG_TYPES)}")
    q = pose.orientation
    return float(pose.position.x), float(pose.position.y), yaw_from_quat(q.x, q.y, q.z, q.w)
