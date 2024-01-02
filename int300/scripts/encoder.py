#!/usr/bin/python
import time
import serial
import rospy
import numpy as np
from std_msgs.msg import Int32


print("Starting UART....")

print("NVIDIA Jetson Nano Developer Kit")


serial_port = serial.Serial(
    port="/dev/ttyUSB0",
    baudrate=9600,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
)
# Wait a second to let the port initialize
time.sleep(0.03)

try:
    #serial_port.write("UART Demonstration Program\r\n".encode())
    #serial_port.write("NVIDIA Jetson Nano Developer Kit\r\n".encode())
    while not rospy.is_shutdown():
    	lpub = rospy.Publisher("lwheel", Int32, queue_size = 20)
    	rpub = rospy.Publisher("rwheel", Int32, queue_size = 20)
    	rospy.init_node('encoder', anonymous = True)
    	rate = rospy.Rate(34)
        if serial_port.inWaiting() > 0:
            #print("in")
            data = serial_port.readline()
            #print(data)
            split_data = data.split(":")
            if (len(split_data) > 3):
	        left = (long(str(split_data[1])))
                right = (long(split_data[2]))
	        lpub.publish(left)
	        rpub.publish(right)
                #print(data.split())
                #print("left: ", left)
	       # print("right: ", right)
	        #rate.sleep()

            else:
                rospy.logerr("Encoder data not recieved")


except rospy.ROSInterruptException:
        rosp.loginfo("Exiting...")

except KeyboardInterrupt:
        print("Exiting Program")

except Exception as exception_error:
        print("Error occurred. Exiting Program")
        print("Error: " + str(exception_error))

finally:
        serial_port.close()
        pass
