#!/usr/bin/env python3
"""
ROS /ctrl_cmd_0 -> MORAI gRPC ControlVehicle bridge

사용 예:
  cd <워크스페이스 루트>
  source devel/setup.bash
  python3 grpc_inha_univ/src/ros_ctrl_to_grpc_bridge.py

환경변수(선택):
  MORAI_SIM_ADDRESS=127.0.0.1
  MORAI_SIM_PORT=7789
  MORAI_EGO_ID=Ego
  MORAI_CLIENT_KEY=Morai_Grpc_Ros_Bridge
  MORAI_SET_AUTO_MODE=1  # 시작 시 외부제어(AUTO_MODE)로 설정
"""

import os
import sys

import rospy
from morai_msgs.msg import CtrlCmd

CURRENT_PATH = os.path.dirname(os.path.realpath(__file__))
sys.path.append(CURRENT_PATH)
sys.path.append(os.path.join(CURRENT_PATH, "proto"))

from proto.sim_adapter import SimAdapter
from proto.morai.actor.actor_control_pb2 import VehicleCtrlCmd
from proto.morai.actor.actor_set_pb2 import VehicleControlModeParam
from proto.morai.actor.actor_enum_pb2 import VehicleControlMode
from proto.morai.common.object_info_pb2 import ObjectInfo
from proto.morai.common.object_identifier_pb2 import ObjectIdentifier
from proto.morai.common.enum_pb2 import OBJECT_TYPE_VEHICLE


class RosCtrlToGrpcBridge:
    def __init__(self):
        self.address = os.getenv("MORAI_SIM_ADDRESS", "127.0.0.1")
        self.port = int(os.getenv("MORAI_SIM_PORT", "7789"))
        self.ego_id = os.getenv("MORAI_EGO_ID", "Ego")
        self.client_key = os.getenv("MORAI_CLIENT_KEY", "Morai_Grpc_Ros_Bridge")
        self.set_auto_mode = os.getenv("MORAI_SET_AUTO_MODE", "1") == "1"

        self.adapter = SimAdapter()
        self.adapter.connect(self.address, self.port)

        rospy.loginfo(
            "[grpc-bridge] connected to %s:%s (ego_id=%s, client_key=%s)",
            self.address,
            self.port,
            self.ego_id,
            self.client_key,
        )

        if self.set_auto_mode:
            self._set_vehicle_auto_mode()

        self.sub = rospy.Subscriber("/ctrl_cmd_0", CtrlCmd, self.ctrl_cb, queue_size=10)

    def _build_object_info(self) -> ObjectInfo:
        info = ObjectInfo()
        info.id.CopyFrom(ObjectIdentifier(value=self.ego_id))
        info.object_type = OBJECT_TYPE_VEHICLE
        info.client_key = self.client_key
        return info

    def _set_vehicle_auto_mode(self):
        req = VehicleControlModeParam()
        req.actor_info.CopyFrom(self._build_object_info())
        req.mode = VehicleControlMode.VEHICLE_CONTROL_AUTO_MODE

        result = self.adapter.set_vehicle_control_mode(req)
        if result is None:
            rospy.logwarn("[grpc-bridge] set_vehicle_control_mode returned None")
            return

        rospy.loginfo(
            "[grpc-bridge] set auto mode result: status=%s desc=%s",
            getattr(result, "status", "unknown"),
            getattr(result, "description", ""),
        )

    def ctrl_cb(self, msg: CtrlCmd):
        req = VehicleCtrlCmd()
        req.actor_info.CopyFrom(self._build_object_info())
        req.long_cmd_type = msg.longlCmdType
        req.throttle = msg.accel
        req.brake = msg.brake
        req.steer = msg.steering
        req.velocity = msg.velocity
        req.acceleration = msg.acceleration

        result = self.adapter.control_vehicle(req)
        if result is None:
            rospy.logwarn_throttle(2.0, "[grpc-bridge] control_vehicle result is None")
            return

        if hasattr(result, "status") and result.status != 1:
            rospy.logwarn_throttle(
                2.0,
                "[grpc-bridge] control_vehicle failed: status=%s desc=%s",
                result.status,
                getattr(result, "description", ""),
            )


if __name__ == "__main__":
    rospy.init_node("ros_ctrl_to_grpc_bridge")
    bridge = RosCtrlToGrpcBridge()
    rospy.loginfo("[grpc-bridge] subscribing /ctrl_cmd_0 and forwarding to gRPC")
    rospy.spin()
