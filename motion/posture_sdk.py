"""SDK 姿态适配：只负责起立/切 WALK/读姿态/刹车，不参与实时运动。"""
import logging


class SdkPosture:
    def __init__(self, ip: str, speed_ratio: int = 60,
                 pose_timeout_s: float = 0.30,
                 pose_cache_s: float = 0.05):
        self.ip = ip
        self.speed_ratio = speed_ratio
        self.pose_timeout_s = pose_timeout_s
        self.pose_cache_s = pose_cache_s
        self._link = None

    def connect(self):
        from robot_link import RobotLink
        self._link = RobotLink(
            self.ip, self.speed_ratio,
            pose_timeout_s=self.pose_timeout_s,
            pose_cache_s=self.pose_cache_s,
        )
        self._link.connect()

    def stand(self):
        self._link.stand()

    def walk(self):
        self._link.walk()

    def get_pose(self, refresh: bool = False):
        return self._link.get_pose(refresh=refresh) if self._link else None

    def brake(self):
        if self._link is None:
            return
        try:
            self._link.brake()
        except Exception as e:
            logging.warning("[ROS2] SDK 刹车失败: %s", e)

    def close(self):
        if self._link is not None:
            self._link.close()
            self._link = None
