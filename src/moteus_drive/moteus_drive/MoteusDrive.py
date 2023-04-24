import asyncio
import math
from time import sleep
import moteus
try:
    import moteus_pi3hat
except ImportError:
    pass
from rclpy.node import Node

class MoteusDrive(Node):
    def __init__(self, Connector, IDs):
        super().__init__('MoteusDriveCycle')
        self.ids = IDs
        self.Connector = Connector            
        self.conn = []
        self.state = "stop"
        self.terminate = False
        self.get_logger().info('MoteusDrive: Initialized')
        self.get_logger().info('MoteusDriveState: %s' % self.state)
        self.raw_feedback = None
        self.rezero = False
        self.servo_command = None
        self.feedback_position_device = None
        self.bus_map = {1: [1], 2: [2]}
        self.imu_data = None
    
    async def run(self):
        for idx, device_id in enumerate(self.ids):
           self.bus_map[idx+1] = [int(device_id)]
        
        # create transport init device
        if self.Connector == "fdcanusb":
            transport = moteus.Fdcanusb()
        elif self.Connector == "pi3hat":
            transport = moteus_pi3hat.Pi3HatRouter(
                servo_bus_map=self.bus_map
            )
            
        for idx, device_id in enumerate(self.ids):
            self.conn.append(moteus.Controller(id = device_id))
        while True:            
            if self.state == "stop":
                self.make_stop = []  
                for idx, device_id in enumerate(self.ids):
                    self.make_stop.append(self.conn[idx].make_stop(query=True))
                if self.Connector == "pi3hat":
                    self.set_feedback(await transport.cycle(self.make_stop, request_attitude=True))
                else:
                    self.set_feedback(await transport.cycle(self.make_stop))
                
            if self.rezero:
                self.make_rezero = []  
                for idx, device_id in enumerate(self.ids):
                    self.make_rezero.append(self.conn[idx].make_rezero(query=True))
                if self.Connector == "pi3hat":
                    self.set_feedback(await transport.cycle(self.make_rezero, request_attitude=True))
                else:
                    self.set_feedback(await transport.cycle(self.make_rezero))
                self.rezero = False
                
            while self.terminate is False and self.state == "start" and self.servo_command is not None:
                self.make_position = []
                for idx, device_id in enumerate(self.ids):
                    self.make_position.append(
                        self.conn[idx].make_position(
                            position=math.nan, 
                            velocity=self.servo_command[device_id]["velocity"], 
                            maximum_torque=self.servo_command[device_id]["maximum_torque"], 
                            query=True
                        )
                    ) 
                if self.Connector == "pi3hat":
                    self.set_feedback(await transport.cycle(self.make_position, request_attitude=True))
                else:
                    self.set_feedback(await transport.cycle(self.make_position))
                await asyncio.sleep(0.01)
            await asyncio.sleep(0.01)
            
            if self.state == "brake" and self.servo_command is not None:
                self.make_brake = []
                for idx, device_id in enumerate(self.ids):
                    self.make_brake.append(
                        self.conn[idx].make_position(
                            position = math.nan, 
                            velocity = 0.0, 
                            maximum_torque = self.servo_command[device_id]["maximum_torque"],
                            stop_position = self.servo_command[device_id]["position"] + self.servo_command[device_id]["velocity"],
                            query=True
                        )
                    ) 
                if self.Connector == "pi3hat":
                    self.set_feedback(await transport.cycle(self.make_brake, request_attitude=True))
                else:
                    self.set_feedback(await transport.cycle(self.make_brake))
                # self.state = "stop"
                # self.servo_command = None
                self.get_logger().info('MoteusDriveState: %s' % self.state)
                await asyncio.sleep(0.01)
                self.state == "braked"
            
    def set_feedback(self, feedback):
        if self.Connector == "pi3hat" and len(feedback) >= 1:
            imu_result = [x for x in feedback if x.id == -1 and isinstance(x, moteus_pi3hat.CanAttitudeWrapper)][0]
            feedback.pop(-1)
            
            # att = imu_result.attitude
            # print("\n")
            # print(f"attitude={att.w:.4f},{att.x:.4f},{att.y:.4f},{att.z:.4f}")
            # rate_dps = imu_result.rate_dps
            # print(f"rate_dps={rate_dps.x:.3f},{rate_dps.y:.3f},{rate_dps.z:.3f}")
            # accel_mps2 = imu_result.accel_mps2
            # print(f"accel_mps2={accel_mps2.x:.3f},{accel_mps2.y:.3f},{accel_mps2.z:.3f}")
            # euler_rad = imu_result.euler_rad
            # print(f"euler_rad= r={euler_rad.roll:.3f},p={euler_rad.pitch:.3f},y={euler_rad.yaw:.3f}")
            
            self.raw_feedback = feedback
            self.imu_data_feedback = imu_result
        else:
            self.raw_feedback = feedback
    
    def get_feedback(self):
        return self.raw_feedback
    
    def run_start(self):
        asyncio.run(self.run())
        
    def set_state_start(self):
        self.state = "start"
        self.get_logger().info('MoteusDriveState: %s' % self.state)
        
    def set_state_stop(self):
        self.state = "stop"
        self.get_logger().info('MoteusDriveState: %s' % self.state)
        
    def set_state_rezero(self):
        self.rezero = True
        self.get_logger().info('MoteusDriveRezero: %s' % self.rezero)
    
    def set_state_terminated(self):
        self.set_state_stop()
        self.terminate = True
        self.get_logger().info('MoteusDriveState: %s' % self.state)
        
    def set_state_brake(self):
        # self.state = "brake"
        self.state = "stop"
        self.get_logger().info('MoteusDriveState: %s' % self.state)
    
    def set_state_command(self, command):
        self.servo_command = command
        if self.state != "start":
            self.state = "start"