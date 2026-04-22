from machine import Pin, I2C
from utime import sleep
from math import pi


class MPU6050:
    def __init__(self, pow_id=3, scl_id=5, sda_id=4, i2c_addr=0x68, dlpf_level=4):
        self.board_led = Pin(25, Pin.OUT)
        self.pow_pin = Pin(pow_id, Pin.OUT, value=1)
        sleep(0.5)
        self.i2c = I2C(0, scl=Pin(scl_id), sda=Pin(sda_id), freq=400_000)
        self.i2c_addr = i2c_addr
        self.i2c.writeto_mem(
            self.i2c_addr,
            0x6B,  # PWR_MGMT_1 register address
            bytes([0x00]),  # data
        )  # wake up sensor
        assert 0 <= dlpf_level <= 6
        self.i2c.writeto_mem(
            self.i2c_addr, 0x1A, bytes([dlpf_level])
        )  # filter out high freq noise, level 3 ~= 42 Hz
        # Variables
        self.lin_acc_x = 0.0
        self.lin_acc_y = 0.0
        self.lin_acc_z = 0.0
        self.ang_vel_x = 0.0
        self.ang_vel_y = 0.0
        self.ang_vel_z = 0.0
        # Constant
        self.gyro_bias_x = 0.0
        self.gyro_bias_y = 0.0
        self.gyro_bias_z = 0.0
        sleep(1)  # hands off
        self.calibrate_gyro()

    def read_data(self):
        """
        MPU6050 uses 2 bytes to represent values of an entity.
        Acc_X, Acc_Y, Acc_Z, Temp, Gyro_X, Gyro_Y, Gyro_Z are stored in contiguous registers.
        To read'em all, grab 14 bytes starting at the ACCEL_XOUT_H register: 0x3B.
        """

        def process_raw(data, scale):
            """
            Args:
                data: a list contains n words(2 bytes) of sensor data
                scale: a constant coefficient scaling raw data.
                       accelerometer: 16384 per g
                       gyro: 131 per deg/s, TODO: use radians
            Returns:
                value: human readible value in m/s^2 or deg/s
            """
            if data > 32768:
                value = (data - 65535) / scale
            else:
                value = data / scale

            return value

        words = self.i2c.readfrom_mem(
            self.i2c_addr,
            0x3B,  # ACCEL_XOUT_H register address
            14,  # number of bytes
        )  # retrieve raw sensor data in bytes
        # Preprocess bytes, split 2 bytes as a group
        data = [words[i] << 8 | words[i + 1] for i in range(0, len(words), 2)]
        # Calculate human readibles
        self.lin_acc_x = process_raw(data[0], 16384) * 9.80665
        self.lin_acc_y = process_raw(data[1], 16384) * 9.80665
        self.lin_acc_z = process_raw(data[2], 16384) * 9.80665
        self.ang_vel_x = process_raw(data[4], 131) * pi / 180 - self.gyro_bias_x
        self.ang_vel_y = process_raw(data[5], 131) * pi / 180 - self.gyro_bias_y
        self.ang_vel_z = process_raw(data[6], 131) * pi / 180 - self.gyro_bias_z

        return {
            "acc_x": self.lin_acc_x,
            "acc_y": self.lin_acc_y,
            "acc_z": self.lin_acc_z,
            "omg_x": self.ang_vel_x,
            "omg_y": self.ang_vel_y,
            "omg_z": self.ang_vel_z,
        }

    def calibrate_gyro(self, num_samples=1000):
        """
        One-time calibration
        """
        omg_x_deposite = 0.0
        omg_y_deposite = 0.0
        omg_z_deposite = 0.0
        # print("Calibrating Gyro... DO NOT MOVE ROBOT")  # debug
        for i in range(num_samples):
            data = self.read_data()
            omg_x_deposite += data["omg_x"]
            omg_y_deposite += data["omg_y"]
            omg_z_deposite += data["omg_z"]
            # Small delay to let the sensor refresh
            sleep(0.005)
            if i % 50 == 0:  # debug
                self.board_led.toggle()
        self.gyro_bias_x = omg_x_deposite / num_samples
        self.gyro_bias_y = omg_y_deposite / num_samples
        self.gyro_bias_z = omg_z_deposite / num_samples
        # print(f"Calibration Done. Gyro biases x={self.gyro_bias_x}, y={self.gyro_bias_y}, z={self.gyro_bias_z}")  # debug


if __name__ == "__main__":
    from utime import ticks_ms, sleep_ms

    # SETUP
    sensor = MPU6050()

    # LOOP
    while True:
        stamp = ticks_ms()
        data = sensor.read_data()
        # Logging, enable plotter
        # print(
        #     f"acc(m/s/s): x={data['acc_x']:.4f}, y={data['acc_y']:.4f}, z={data['acc_z']:.4f}"
        # )
        print(
            f"angv(deg/s): x={data['omg_x']:.4f} rad/s, y={data['omg_y']:.4f} rad/s, z={data['omg_z']:.4f} rad/s"
        )
        sleep_ms(100)  # 10Hz
