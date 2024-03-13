from pymavlink import mavutil
from utilities.connect_to_sysid import connect_to_sysid
from utilities.wait_for_position_aiding import wait_until_position_aiding
from utilities.get_autopilot_info import get_autopilot_info
import time

class DroneControl():
    def __init__(self):
        self.the_connection = mavutil.mavlink_connection('udpin:localhost:14551')
        self.the_connection.wait_heartbeat()
        print("Heartbeat from system (system %u component %u)" %
            (self.the_connection.target_system, self.the_connection.target_component))

    def establish_connection(self):
        the_connection = mavutil.mavlink_connection('udpin:localhost:14551')
        the_connection.wait_heartbeat()
        print("Heartbeat from system (system %u component %u)" %
            (the_connection.target_system, the_connection.target_component))   

    def face_north(self):
        self.the_connection.mav.command_long_send(self.the_connection.target_system,self.the_connection.target_component, 
                                        mavutil.mavlink.MAV_CMD_CONDITION_YAW, 0,0,25,0,0,0,0,0)
            
    def move(self, x: float = 0, y: float = 0, z: float = 0):
        self.the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, self.the_connection.target_system,
                            self.the_connection.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_OFFSET_NED, int(0b000111111000), x, y, z, 0, 0, 0, 0, 0, 0, 0, 0))
    def takeoff(self, takeoff_altitude: float = 2, tgt_sys_id: int = 1, tgt_comp_id=1):

        print("Heartbeat from system (system %u component %u)" %
            (self.the_connection.target_system, self.the_connection.target_component))

        wait_until_position_aiding(self.the_connection)

        autopilot_info = get_autopilot_info(self.the_connection, tgt_sys_id)

        if autopilot_info["autopilot"] == "ardupilotmega":
            print("Connected to ArduPilot autopilot")
            mode_id = self.the_connection.mode_mapping()["GUIDED"]
            takeoff_params = [0, 0, 0, 0, 0, 0, takeoff_altitude]


        # Change mode to guided (Ardupilot) or takeoff (PX4)
        self.the_connection.mav.command_long_send(tgt_sys_id, tgt_comp_id, mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                                    0, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id, 0, 0, 0, 0, 0)
        ack_msg =self.the_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
        print(f"Change Mode ACK:  {ack_msg}")

        # Arm the UAS
        self.the_connection.mav.command_long_send(tgt_sys_id, tgt_comp_id,
                                            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)

        arm_msg = self.the_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
        print(f"Arm ACK:  {arm_msg}")

        # Command Takeoff
        self.the_connection.mav.command_long_send(tgt_sys_id, tgt_comp_id,
                                            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, takeoff_params[0], takeoff_params[1], takeoff_params[2], takeoff_params[3], takeoff_params[4], takeoff_params[5], takeoff_params[6])

        takeoff_msg = self.the_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
        print(f"Takeoff ACK:  {takeoff_msg}")

        return takeoff_msg.result
