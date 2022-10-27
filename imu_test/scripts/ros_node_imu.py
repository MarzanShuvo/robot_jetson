#!/usr/bin/env python3
import math
import rospy
from BMI160_i2c import Driver
from BMI160_i2c import definitions
import constants

from sensor_msgs.msg import Imu
from std_msgs.msg import Header
from geometry_msgs.msg import (Quaternion, Vector3)


# sensor = Driver(constants.BMI160_ADDR)
# rospy.init_node('imu_data_publisher')
# imu_pub = rospy.Publisher('/imu_msg/raw', Imu, queue_size=10)
# rate = rospy.Rate(10)


def imu_data_read(sensor, imu_i2c_address, imu_i2c_bus_id):
    try:
        sensor = Driver(imu_i2c_address)
        sensor.setFullScaleAccelRange(definitions.ACCEL_RANGE_4G, constants.ACCEL_RANGE_4G_FLOAT)
        sensor.setFullScaleGyroRange(definitions.GYRO_RANGE_250, constants.GYRO_RANGE_250_FLOAT)

        sensor.autoCalibrateXAccelOffset(0)
        sensor.autoCalibrateYAccelOffset(0)
        sensor.autoCalibrateZAccelOffset(-1)
        sensor.setAccelOffsetEnabled(True)
        return sensor

    
    except Exception as ex:
        rospy.loginfo("Failed to create IMU monitor: {}".format(ex))
        raise ex

        rospy.loginfo('Initialization and calibration of IMU sensor done.')




def publish_imu_message(sensor, imu_pub, rate):
    try:
        imu_msg = Imu()
        data = sensor.getMotion6()

        gyro = Vector3()
        gyro.x = data[0] / constants.CONVERSION_MASK_16BIT_FLOAT * constants.GYRO_RANGE_250_FLOAT * (math.pi / 180)
        gyro.y = data[1] / constants.CONVERSION_MASK_16BIT_FLOAT * constants.GYRO_RANGE_250_FLOAT * (math.pi / 180)
        gyro.z = data[2] / constants.CONVERSION_MASK_16BIT_FLOAT * constants.GYRO_RANGE_250_FLOAT * (math.pi / 180)
        
        accel = Vector3()
        accel.x = data[3] * constants.GRAVITY_CONSTANT / constants.CONVERSION_MASK_16BIT_FLOAT * constants.ACCEL_RANGE_4G_FLOAT 
        accel.y = data[4] * constants.GRAVITY_CONSTANT / constants.CONVERSION_MASK_16BIT_FLOAT * constants.ACCEL_RANGE_4G_FLOAT 
        accel.z = data[5] * constants.GRAVITY_CONSTANT / constants.CONVERSION_MASK_16BIT_FLOAT * constants.ACCEL_RANGE_4G_FLOAT
        imu_msg.angular_velocity = gyro
        imu_msg.angular_velocity_covariance = constants.EMPTY_ARRAY_9
        imu_msg.linear_acceleration = accel
        imu_msg.linear_acceleration_covariance = constants.EMPTY_ARRAY_9
        imu_msg.orientation_covariance = constants.EMPTY_ARRAY_9
        imu_msg.orientation_covariance[0] = 0.0
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.header.frame_id = 'imu_link'
        imu_pub.publish(imu_msg)
        rate.sleep()


    except Exception as ex:
        rospy.loginfo("Error in publishing sensor message: {}".format(ex))


def main(args=None):
    
    try:
       rospy.init_node('imu_data_publisher', anonymous=True)
       rate = rospy.Rate(25) # ROS Rate at 5Hz
       imu_pub = rospy.Publisher('imu/data_raw', Imu, queue_size=10)
       sensor = Driver(constants.BMI160_ADDR)
       imu_i2c_address = constants.BMI160_ADDR
       imu_i2c_bus_id = constants.I2C_BUS_ID
       sensor = imu_data_read(sensor, imu_i2c_address, imu_i2c_bus_id)
       while not rospy.is_shutdown():
            publish_imu_message(sensor, imu_pub, rate)
       if(rospy.is_shutdown):
            raise SystemExit 
            

    except KeyboardInterrupt:
        print("Raising SystemExit")
        raise SystemExit    

if __name__ == "__main__":
    while True:
        main()