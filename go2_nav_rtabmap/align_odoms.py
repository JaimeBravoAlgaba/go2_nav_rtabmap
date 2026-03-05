#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from tf_transformations import quaternion_multiply, quaternion_inverse


def pose_to_tuple(pose):
    p = pose.position
    q = pose.orientation
    return (p.x, p.y, p.z, q.x, q.y, q.z, q.w)


def tf_to_tuple(t: TransformStamped):
    tr = t.transform.translation
    rq = t.transform.rotation
    return (tr.x, tr.y, tr.z, rq.x, rq.y, rq.z, rq.w)


def quat_rotate(q, v_xyz):
    vx, vy, vz = v_xyz
    v = (vx, vy, vz, 0.0)
    q_inv = quaternion_inverse(q)
    v_rot = quaternion_multiply(quaternion_multiply(q, v), q_inv)
    return (v_rot[0], v_rot[1], v_rot[2])


def compose(Ta, Tb):
    ax, ay, az, aqx, aqy, aqz, aqw = Ta
    bx, by, bz, bqx, bqy, bqz, bqw = Tb

    q_a = (aqx, aqy, aqz, aqw)
    b_rot = quat_rotate(q_a, (bx, by, bz))

    tx = ax + b_rot[0]
    ty = ay + b_rot[1]
    tz = az + b_rot[2]

    q_out = quaternion_multiply(q_a, (bqx, bqy, bqz, bqw))
    return (tx, ty, tz, q_out[0], q_out[1], q_out[2], q_out[3])


def invert(T):
    tx, ty, tz, qx, qy, qz, qw = T
    q = (qx, qy, qz, qw)
    q_inv = quaternion_inverse(q)
    t_rot = quat_rotate(q_inv, (tx, ty, tz))
    return (-t_rot[0], -t_rot[1], -t_rot[2], q_inv[0], q_inv[1], q_inv[2], q_inv[3])


def yaw_from_quat(qx, qy, qz, qw):
    return math.atan2(2.0*(qw*qz + qx*qy), 1.0 - 2.0*(qy*qy + qz*qz))


class AlignOdomsFromInitialPose(Node):
    def __init__(self):
        super().__init__("align_odoms_from_initialpose")

        self.declare_parameter("robot_odom_frame", "odom")
        self.declare_parameter("rtabmap_odom_frame", "rtabmap/odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("publish_rate_hz", 30.0)
        # ✅ frame en el que interpretamos /initialpose
        #   Si RViz Fixed Frame = rtabmap/odom, esto cuadra perfecto.
        self.declare_parameter("initialpose_frame", "rtabmap/odom")
        self.declare_parameter("initialpose_topic", "/initialpose")

        self.robot_odom_frame = self.get_parameter("robot_odom_frame").value
        self.rtabmap_odom_frame = self.get_parameter("rtabmap_odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.publish_rate = float(self.get_parameter("publish_rate_hz").value)
        self.initialpose_frame = self.get_parameter("initialpose_frame").value
        self.initialpose_topic = self.get_parameter("initialpose_topic").value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        # ✅ Publica identidad desde el inicio
        self.T_rtabmap_to_robotodom = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)

        self.sub = self.create_subscription(
            PoseWithCovarianceStamped,
            self.initialpose_topic,
            self.on_initialpose,
            10
        )

        period = 1.0 / max(self.publish_rate, 1.0)
        self.timer = self.create_timer(period, self.on_timer)

        self.get_logger().info(
            f"Publicando TF desde el arranque: {self.rtabmap_odom_frame} -> {self.robot_odom_frame} (identidad). "
            f"Al recibir {self.initialpose_topic}, alinearé usando la pose deseada en frame '{self.initialpose_frame}'.\n"
            f"TIP: Pon en RViz Fixed Frame = '{self.initialpose_frame}' antes de usar 2D Pose Estimate."
        )

    def on_initialpose(self, msg: PoseWithCovarianceStamped):
        # 1) Construye T_(initialpose_frame -> base) deseado desde el mensaje
        #    Si msg.header.frame_id no coincide con initialpose_frame, intentamos transformarlo.
        desired = pose_to_tuple(msg.pose.pose)
        src_frame = msg.header.frame_id.strip() if msg.header.frame_id else ""

        if src_frame and src_frame != self.initialpose_frame:
            # Transformar pose a initialpose_frame usando TF si es posible
            try:
                # Buscamos T_(initialpose_frame -> src_frame) y lo componemos con desired (src_frame->base)
                t = self.tf_buffer.lookup_transform(self.initialpose_frame, src_frame, rclpy.time.Time())
                T_init_src = tf_to_tuple(t)  # initialpose_frame -> src_frame
                desired = compose(T_init_src, desired)
            except Exception as e:
                self.get_logger().warn(
                    f"/initialpose llega en '{src_frame}', pero no pude transformarlo a '{self.initialpose_frame}': {e}"
                )
                return

        # 2) Lee odom actual: T_(odom -> base)
        try:
            t_odom_base = self.tf_buffer.lookup_transform(
                self.robot_odom_frame,
                self.base_frame,
                rclpy.time.Time()
            )
        except Exception as e:
            self.get_logger().warn(f"No pude leer TF {self.robot_odom_frame}->{self.base_frame}: {e}")
            return

        Tob = tf_to_tuple(t_odom_base)

        # 3) Calcula T_(rtabmap/odom -> odom) = T_(initframe->base)_desired * inverse(T_(odom->base)_current)
        T_ro = compose(desired, invert(Tob))

        self.T_rtabmap_to_robotodom = T_ro

        tx, ty, tz, qx, qy, qz, qw = T_ro
        yaw = yaw_from_quat(qx, qy, qz, qw)
        self.get_logger().info(
            f"Alineamiento aplicado: {self.rtabmap_odom_frame}->{self.robot_odom_frame} "
            f"t=({tx:.3f},{ty:.3f},{tz:.3f}) yaw={yaw:.3f} rad"
        )

    def on_timer(self):
        tx, ty, tz, qx, qy, qz, qw = self.T_rtabmap_to_robotodom

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.rtabmap_odom_frame
        t.child_frame_id = self.robot_odom_frame
        t.transform.translation.x = float(tx)
        t.transform.translation.y = float(ty)
        t.transform.translation.z = float(tz)
        t.transform.rotation.x = float(qx)
        t.transform.rotation.y = float(qy)
        t.transform.rotation.z = float(qz)
        t.transform.rotation.w = float(qw)

        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = AlignOdomsFromInitialPose()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
