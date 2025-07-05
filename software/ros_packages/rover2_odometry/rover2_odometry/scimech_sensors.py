import rclpy
import rclpy.logging
from rclpy.node import Node
import serial
from std_msgs.msg import Float32MultiArray
from time import time, sleep

class ScimechSensors(Node):

    def __init__(self):

        super().__init__('scimech_sensors')

        self.scimech_data_publisher_ = self.create_publisher(Float32MultiArray, 'scimech/data', 10)
        
        timer_period = 1  # seconds

        self.logger = self.get_logger()

        self.humidity = 0.0
        self.hydrogen = 0.0
        self.ozone = 0.0
        self.temperature = 0.0

        self.scimech_arduino = serial.Serial('/dev/ttyACM0',9600, timeout=1)
        self.scimech_arduino.flush()
        self.timer = self.create_timer(timer_period, self.timer_callback)        
  

    def read_scimech_serial_data(self):
        while True:
            if self.scimech_arduino.in_waiting > 0:
                try: 
                    line = self.scimech_arduino.readline().decode('utf-8').rstrip()
                    if line and line!="ACKSERVO":
                        self.logger.info("Reading from arduino: {}".format(line))
                        data_array = line.split(",")
                        if(len(data_array)>1):
                        
                            self.temperature = float(data_array[0])
                            self.humidity = float(data_array[1])
                            self.hydrogen = float(data_array[2])
                            self.ozone = float(data_array[3])
                        self.scimech_arduino.flush()
                    
                
                except Exception as e:
                    self.logger.info(e)
                break

    def timer_callback(self):
        
        self.read_scimech_serial_data()
        msg = Float32MultiArray()
        msg.data = [self.humidity,self.hydrogen,self.ozone,self.temperature]
        self.scimech_data_publisher_.publish(msg)
        self.logger.info('Publishing: "%s"' % msg.data)
        with open("/home/makemorerobot/Rover_2023_2024/software/ros_packages/rover2_odometry/rover2_odometry/sensor_data.csv","a") as file:

            file.write(f"{self.temperature},{self.humidity},{self.hydrogen},{self.ozone}\n")

def main(args=None):
    rclpy.init(args=args)

    scimech_sensors = ScimechSensors()

    rclpy.spin(scimech_sensors)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    scimech_sensors.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
