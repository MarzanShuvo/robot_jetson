#!/usr/bin/env python3
import math
import rospy
from BMI160_i2c import Driver
from BMI160_i2c import definitions
import constants


sensor = Driver(constants.BMI160_ADDR)

def imu_data_read(imu_i2c_address, imu_i2c_bus_id):
    try:
        sensor = Driver(imu_i2c_address)
        sensor.setFullScaleAccelRange(definitions.ACCEL_RANGE_4G, constants.ACCEL_RANGE_4G_FLOAT)
        sensor.setFullScaleGyroRange(definitions.GYRO_RANGE_250, constants.GYRO_RANGE_250_FLOAT)

        sensor.autoCalibrateXAccelOffset(0)
        sensor.autoCalibrateYAccelOffset(0)
        sensor.autoCalibrateZAccelOffset(-1)


    
    except Exception as ex:
        print("Failed to create IMU monitor: {}".format(ex))
        raise ex

    print('Initialization and calibration of IMU sensor done.')


def publish_imu_message():
    try:
        data = sensor.getMotion6()
        gyro_x = data[0] / constants.CONVERSION_MASK_16BIT_FLOAT * constants.GYRO_RANGE_250_FLOAT * (math.pi / 180)
        gyro_y = data[1] / constants.CONVERSION_MASK_16BIT_FLOAT * constants.GYRO_RANGE_250_FLOAT * (math.pi / 180)
        gyro_z = data[2] / constants.CONVERSION_MASK_16BIT_FLOAT * constants.GYRO_RANGE_250_FLOAT * (math.pi / 180)
        print("gyro_x: {} gyro_y: {}  gyro_z: {}\n".format(gyro_x, gyro_y, gyro_z))
        accel_x = data[3] * constants.GRAVITY_CONSTANT / constants.CONVERSION_MASK_16BIT_FLOAT * constants.ACCEL_RANGE_4G_FLOAT 
        accel_y = data[4] * constants.GRAVITY_CONSTANT / constants.CONVERSION_MASK_16BIT_FLOAT * constants.ACCEL_RANGE_4G_FLOAT 
        accel_z = data[5] * constants.GRAVITY_CONSTANT / constants.CONVERSION_MASK_16BIT_FLOAT * constants.ACCEL_RANGE_4G_FLOAT 
        print("accel_x: {} accel_y: {}  accel_z: {}\n".format(gyro_x, gyro_y, gyro_z))

    except Exception as ex:
        print("Error in publishing sensor message: {}".format(ex))


def main(args=None):
    
    try:
       imu_i2c_address = constants.BMI160_ADDR
       imu_i2c_bus_id = constants.I2C_BUS_ID
       imu_data_read(imu_i2c_address, imu_i2c_bus_id)
       publish_imu_message()

    except KeyboardInterrupt:
        print("Raising SystemExit")
        raise SystemExit    

if __name__ == "__main__":
    while True:
        main()