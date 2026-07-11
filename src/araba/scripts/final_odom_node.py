#!/usr/bin/env python3
"""
final_odom_node — dünya çerçeveli nihai odometri (SİM UYARLAMASI).

Kaynak: DEOS_MimariDev/perception/sensor_fusion/final_odom_node.py
Mimaride bu node bir SEÇİCİdir: taze ise LiDAR ICP odomu, değilse EKF odomu
50 Hz'de /deos/localization/odom/final'e aynen geçirir. Sim'de ICP/EKF yığını
yok; onların işlevini (driftsiz dünya-çerçeveli konumlama) şu füzyon üstlenir:

  - POZİSYON  ← /gps/fix   (NavSat; dünya SDF'indeki datumdan ENU'ya çevrilir)
  - YÖNELİM   ← /imu/data  (Gazebo IMU'su spawn yönüne göreli ve driftsizdir;
                            spawn_yaw ile bileşkelenince gerçek dünya yaw'ı verir.
                            Tekerlek odometrisinin yaw'ı DRİFT eder — kullanılmaz.)
  - HIZ (twist) ← /odom    (gövde çerçevesinde; dönüşüm gerekmez)

İlk GPS fix'i gelene kadar pozisyon, /odom'un spawn pozuyla dünyaya taşınmış
hali olarak yayınlanır (yedek yol; tekerlek odometrisi olduğu için driftlidir).

Çıkış: nav_msgs/Odometry, frame_id='world' (ENU: +X=Doğu, +Y=Kuzey), 50 Hz.

YÖN SÖZLEŞMESİ: orientation STANDART ENU yaw taşır (0=Doğu, saat yönü tersi).
mission_planning'in beklediği pusula yönü (0=Kuzey, saat yönü) TÜKETİCİDE
çevrilmelidir: compass_deg = (90 − enu_yaw_deg) % 360. (3. adım uyarlamasında
mission_planning_node'a sim parametresi olarak eklenecek.)

datum_lat/lon dünya SDF'indeki <spherical_coordinates> ile, spawn_* launch'taki
`create` argümanlarıyla AYNI kalmalıdır; gazebo.launch.py hepsini tek yerden besler.
"""

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix

# WGS84
_A = 6378137.0
_E2 = 6.69437999014e-3


class FinalOdomNode(Node):
    def __init__(self):
        super().__init__("final_odom_node")

        # Launch'taki spawn pozuyla senkron tutulur (gazebo.launch.py SPAWN_* sabitleri)
        self.declare_parameter("spawn_x", 45.31)
        self.declare_parameter("spawn_y", 16.0)
        self.declare_parameter("spawn_z", 0.51)
        self.declare_parameter("spawn_yaw", 1.58)
        # Dünya SDF'indeki <spherical_coordinates> ile senkron tutulur
        self.declare_parameter("datum_lat", 40.7899)
        self.declare_parameter("datum_lon", 29.5089)
        # GPS anteninin base_link'e göre montaj ofseti (URDF gps_sensor joint origin'i).
        # NavSat pozisyonu anten konumudur; araç merkezine bu ofset geri çekilir.
        self.declare_parameter("gps_offset_x", -0.81835)
        self.declare_parameter("gps_offset_y", -0.675)
        # Sim köprüsü topic'leri (mimaride deos_topic_layout'tan gelir)
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("gps_topic", "/gps/fix")
        self.declare_parameter("imu_topic", "/imu/data")
        self.declare_parameter("out_topic", "/localization/odom/final")

        self._sx = float(self.get_parameter("spawn_x").value)
        self._sy = float(self.get_parameter("spawn_y").value)
        self._sz = float(self.get_parameter("spawn_z").value)
        self._syaw = float(self.get_parameter("spawn_yaw").value)
        self._cos = math.cos(self._syaw)
        self._sin = math.sin(self._syaw)
        # Spawn yaw'ını saf-z quaternion olarak önden hazırla (q_world = q_spawn ⊗ q_imu)
        self._qs_z = math.sin(self._syaw / 2.0)
        self._qs_w = math.cos(self._syaw / 2.0)

        # Datum civarı yerel metre/derece katsayıları (ellipsoid; ±birkaç yüz m'de mm hassas)
        lat0 = math.radians(float(self.get_parameter("datum_lat").value))
        self._lat0_deg = float(self.get_parameter("datum_lat").value)
        self._lon0_deg = float(self.get_parameter("datum_lon").value)
        s2 = math.sin(lat0) ** 2
        n_rad = _A / math.sqrt(1.0 - _E2 * s2)                     # normal yarıçap N(φ0)
        m_rad = _A * (1.0 - _E2) / (1.0 - _E2 * s2) ** 1.5         # meridyen yarıçapı M(φ0)
        self._m_per_deg_lat = math.radians(1.0) * m_rad
        self._m_per_deg_lon = math.radians(1.0) * n_rad * math.cos(lat0)

        self._gps_ox = float(self.get_parameter("gps_offset_x").value)
        self._gps_oy = float(self.get_parameter("gps_offset_y").value)

        self._odom: Odometry | None = None
        self._gps: NavSatFix | None = None
        self._imu: Imu | None = None

        self.create_subscription(
            Odometry, str(self.get_parameter("odom_topic").value), self._odom_cb, 10
        )
        self.create_subscription(
            NavSatFix, str(self.get_parameter("gps_topic").value), self._gps_cb, 10
        )
        self.create_subscription(
            Imu, str(self.get_parameter("imu_topic").value), self._imu_cb, 20
        )
        self._pub = self.create_publisher(Odometry, str(self.get_parameter("out_topic").value), 10)

        self.create_timer(0.02, self._tick)  # 50 Hz — mimarideki kadans
        self.get_logger().info(
            f"final_odom_node ready — GPS+IMU füzyonu, datum=({self._lat0_deg}, {self._lon0_deg}), "
            f"spawn=({self._sx:.2f}, {self._sy:.2f}, yaw={self._syaw:.2f}) "
            f"-> out={str(self.get_parameter('out_topic').value)}"
        )

    def _odom_cb(self, msg: Odometry) -> None:
        self._odom = msg

    def _gps_cb(self, msg: NavSatFix) -> None:
        if math.isfinite(msg.latitude) and math.isfinite(msg.longitude):
            self._gps = msg

    def _imu_cb(self, msg: Imu) -> None:
        self._imu = msg

    def _tick(self) -> None:
        if self._odom is None and self._gps is None:
            return

        out = Odometry()
        out.header.frame_id = "world"
        out.child_frame_id = "base_link"

        if self._gps is not None:
            # Pozisyon: GPS → datum civarı yerel ENU; anten ofseti araç merkezine geri çekilir
            out.header.stamp = self._gps.header.stamp
            gx = (self._gps.longitude - self._lon0_deg) * self._m_per_deg_lon
            gy = (self._gps.latitude - self._lat0_deg) * self._m_per_deg_lat
            wyaw = self._syaw
            if self._imu is not None:
                q = self._imu.orientation
                wyaw += math.atan2(
                    2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
                )
            c, s = math.cos(wyaw), math.sin(wyaw)
            out.pose.pose.position.x = gx - (c * self._gps_ox - s * self._gps_oy)
            out.pose.pose.position.y = gy - (s * self._gps_ox + c * self._gps_oy)
            out.pose.pose.position.z = float(self._gps.altitude)
        else:
            # Yedek yol (ilk fix öncesi): tekerlek odomunu spawn pozuyla dünyaya taşı
            out.header.stamp = self._odom.header.stamp
            px = self._odom.pose.pose.position.x
            py = self._odom.pose.pose.position.y
            out.pose.pose.position.x = self._sx + px * self._cos - py * self._sin
            out.pose.pose.position.y = self._sy + px * self._sin + py * self._cos
            out.pose.pose.position.z = self._sz + self._odom.pose.pose.position.z

        if self._imu is not None:
            # Yönelim: q_world = q_spawn(saf z) ⊗ q_imu  (IMU spawn'a göreli, driftsiz)
            q = self._imu.orientation
            out.pose.pose.orientation.w = self._qs_w * q.w - self._qs_z * q.z
            out.pose.pose.orientation.x = self._qs_w * q.x - self._qs_z * q.y
            out.pose.pose.orientation.y = self._qs_w * q.y + self._qs_z * q.x
            out.pose.pose.orientation.z = self._qs_w * q.z + self._qs_z * q.w
        elif self._odom is not None:
            # Yedek yol: tekerlek odomu yaw'ı (drift eder — sadece IMU yokken)
            q = self._odom.pose.pose.orientation
            out.pose.pose.orientation.w = self._qs_w * q.w - self._qs_z * q.z
            out.pose.pose.orientation.x = self._qs_w * q.x - self._qs_z * q.y
            out.pose.pose.orientation.y = self._qs_w * q.y + self._qs_z * q.x
            out.pose.pose.orientation.z = self._qs_w * q.z + self._qs_z * q.w
        else:
            out.pose.pose.orientation.w = 1.0

        if self._odom is not None:
            out.twist = self._odom.twist  # gövde çerçevesi; dönüşüm gerekmez

        self._pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FinalOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
