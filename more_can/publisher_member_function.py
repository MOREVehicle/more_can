# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License
from .PCANBasic import *
import rclpy
from rclpy.node import Node
from std_msgs.msg import String 


class CANPublisher(Node):
    def __init__(self):
        super().__init__('can_publisher')
        self.publisher = self.create_publisher(String, 'can_topic', 10)
        self.timer_period = 1  # Timer interval in seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # Initialize PCANBasic
        try:
            self.m_objPCANBasic = PCANBasic()
            self.m_DLLFound = self.check_for_library()
            if not self.m_DLLFound:
                self.get_logger().error("Unable to find the PCANBasic library.")
                return
        except Exception as e:
            self.get_logger().error(f"Error initializing PCANBasic: {str(e)}")
            return

        # Initialize CAN
        self.initialize_can()

    def check_for_library(self):
        try:
            self.m_objPCANBasic.Uninitialize(PCAN_NONEBUS)
            return True
        except Exception as e:
            self.get_logger().error(f"Unable to find the PCANBasic library: {str(e)}")
            return False

    def initialize_can(self):
        # Initialization of the selected channel (adapt based on your requirements)
        self.PcanHandle = PCAN_USBBUS1
        self.IsFD = False
        self.Bitrate = PCAN_BAUD_500K

        if self.IsFD:
            stsResult = self.m_objPCANBasic.InitializeFD(self.PcanHandle, self.BitrateFD)
        else:
            stsResult = self.m_objPCANBasic.Initialize(self.PcanHandle, self.Bitrate)

        if stsResult != PCAN_ERROR_OK:
            self.get_logger().error("Failed to initialize CAN. Please check the defines in the code.")
            self.show_status(stsResult)
            return

        self.get_logger().info("Successfully initialized CAN.")
        self.read_messages()

    def timer_callback(self):
        self.read_messages()

    def read_messages(self):
        while True:
            if self.IsFD:
                stsResult, msg, timestamp = self.m_objPCANBasic.ReadFD(self.PcanHandle)
            else:
                stsResult, msg, timestamp = self.m_objPCANBasic.Read(self.PcanHandle)

            if stsResult == PCAN_ERROR_OK:
                self.process_message(msg, timestamp)
            elif stsResult == PCAN_ERROR_QRCVEMPTY:
                break  # No more messages in the queue
            else:
                self.show_status(stsResult)
                break
                
     

    def process_message(self, msg, timestamp):
        # Convert the received CAN message to a ROS message
        ros_msg = self.convert_to_ros_message(msg, timestamp)
        if ros_msg:
            self.publisher.publish(ros_msg)

    def convert_to_ros_message(self, msg, timestamp):
        # Customize this method to convert CAN message to ROS message
        # Extract relevant data from 'msg' and create a String ROS message
        # Example:
        data_str = ' '.join(format(byte, '02x') for byte in msg.DATA)
        id_str = '{:03x}'.format(msg.ID) if msg.MSGTYPE == PCAN_MESSAGE_STANDARD else '{:08x}'.format(msg.ID)
        timestamp_str = str(timestamp.millis) + "ms " 

        ros_msg = String()
        ros_msg.data = f"ID: {id_str} | Data: {data_str} | Timestamp: {timestamp_str}"
        self.publisher.publish(ros_msg)
        self.get_logger().info('Publishing: "%s"' % ros_msg.data)
        return ros_msg


    def show_status(self, status):
        # Customize this method to handle status/errors as per your requirement
        self.get_logger().error(f"PCAN status: {status}")
        
 
def main(args=None):
    rclpy.init(args=args)
    can_publisher = CANPublisher()
    rclpy.spin(can_publisher)
    can_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    write()

