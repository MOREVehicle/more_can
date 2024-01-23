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
from more_interfaces.msg import Can 
ros_msg = Can()

class CANPublisher(Node):
    def __init__(self):
        super().__init__('can_publisher')
        self.publisher = self.create_publisher(Can, 'can_topic', 10)
        self.timer_period = 0.5  # Timer interval in seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        self.subscription = self.create_subscription(
            Can,
            'more_can_publisher',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        
        # Initialize PCANBasic
        try:
            self.m_objPCANBasic = PCANBasic()
            self.m_DLLFound = self.check_for_library()
            if not self.m_DLLFound:
                self.get_logger().error("Unable to find the PCANBasic library.")
                return

            # Initialize CAN
            self.initialize_can()
        except Exception as e:
            self.get_logger().error(f"Error during initialization: {str(e)}")

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

    def timer_callback(self):
        self.write_messages()
    
    
    def listener_callback(self, msg):
    
        msgout.id = msg.id
        msgout.data[0] = msg.data[0]
        msgout.data[1] = msg.data[1]
        msgout.data[2] = msg.data[2]
        msgout.data[3] = msg.data[3]
        msgout.data[4] = msg.data[4]
        msgout.data[5] = msg.data[5]
        msgout.data[6] = msg.data[6]
        msgout.data[7] = msg.data[7]
        
        self.get_logger().info('I heard: "%s"' % msgout)
    
    
    def convert_to_ros_message(self, msg):

        ros_msg = Can()
        
        ros_msg.id = msg.ID
        ros_msg.data[0] = msg.DATA[0]
        ros_msg.data[1] = msg.DATA[1]
        ros_msg.data[2] = msg.DATA[2]
        ros_msg.data[3] = msg.DATA[3]
        ros_msg.data[4] = msg.DATA[4]
        ros_msg.data[5] = msg.DATA[5]
        ros_msg.data[6] = msg.DATA[6]
        ros_msg.data[7] = msg.DATA[7]
        
        self.publisher.publish(ros_msg)
        self.get_logger().info('Publishing: "%s"' % ros_msg)
        
        return ros_msg
    
        
    def write_messages(self):
    
        self.read_messages()    
        
        msgCanMessage = TPCANMsg()
        msgCanMessage.ID = msgout.id
        msgCanMessage.LEN = 8
        msgCanMessage.MSGTYPE = PCAN_MESSAGE_STANDARD.value
        
        msgCanMessage.DATA[0] = msgout.data[0]
        msgCanMessage.DATA[1] = msgout.data[1]
        msgCanMessage.DATA[2] = msgout.data[2]
        msgCanMessage.DATA[3] = msgout.data[3]
        msgCanMessage.DATA[4] = msgout.data[4]
        msgCanMessage.DATA[5] = msgout.data[5]
        msgCanMessage.DATA[6] = msgout.data[6]
        msgCanMessage.DATA[7] = msgout.data[7]
         
        result = self.m_objPCANBasic.Write(self.PcanHandle, msgCanMessage)
    
        if result != PCAN_ERROR_OK:
   
            result = self.m_objPCANBasic.GetErrorText(result)
            print (result)
        else:
            print ("Message sent successfully")
            print (msgCanMessage.ID)
            
            
           
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
                #self.show_status(stsResult)
                break
                
     

    def process_message(self, msg, timestamp):
        #Convert the received CAN message to a ROS message
        ros_msg = self.convert_to_ros_message(msg)
        #if ros_msg:
            #self.publisher.publish(ros_msg)
 
def main(args=None):
    rclpy.init(args=args)
    can_publisher = CANPublisher()
    rclpy.spin(can_publisher)
    can_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

