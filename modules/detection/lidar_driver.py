"""
Lightware SF45/B Driver
Lidar documentation: https://support.lightware.co.za/sf45b/#/commands
"""

import time
import struct
import serial


class LidarDriver:
    """
    Wrapper for lidar
    """

    MIN_UPDATE_RATE = 1
    MAX_UPDATE_RATE = 12

    MAX_HIGH_ANGLE = 170
    MIN_HIGH_ANGLE = 5

    MIN_LOW_ANGLE = -170
    MAX_LOW_ANGLE = -5

    MAX_SPEED = 5
    MIN_SPEED = 2000

    NO_WRITE = 0
    WRITE = 1

    PRODUCT_NAME = 0
    FIRMWARE_VERSION = 2
    SERIAL_NUMBER = 3
    DISTANCE_OUTPUT = 27
    STREAM = 30
    DISTANCE = 44
    UPDATE_RATE = 66
    ROTATION_SPEED = 85
    LOW_ANGLE = 98
    HIGH_ANGLE = 99

    def __init__(self, port_name, baudrate, timeout):
        """
        port_name: port that the lidar is connected to
        baudrate: baudrate of the lidar port
        timeout: timeout for connecting to serial port
        """
        self.serial_port = serial.Serial(port_name, baudrate, timeout=timeout)
        self._packet_parse_state = 0
        self._packet_payload_size = 0
        self._packet_size = 0
        self._packet_data = []

    @staticmethod
    def __create_crc(data):
        """
        Create a CRC-16-CCITT 0x1021 hash of the specified data.
        """
        crc = 0

        for i in data:
            code = crc >> 8
            code ^= int(i)
            code ^= code >> 4
            crc = crc << 8
            crc ^= code
            code = code << 5
            crc ^= code
            code = code << 7
            crc ^= code
            crc &= 0xFFFF

        return crc

    def __build_packet(self, command, write, data=None):
        """
        Create raw bytes for a packet.
        """
        if data is None:
            data = []

        payload_length = 1 + len(data)
        flags = (payload_length << 6) | (write & 0x1)
        packet_bytes = [0xAA, flags & 0xFF, (flags >> 8) & 0xFF, command]
        packet_bytes.extend(data)
        crc = self.__create_crc(packet_bytes)
        packet_bytes.append(crc & 0xFF)
        packet_bytes.append((crc >> 8) & 0xFF)

        return bytearray(packet_bytes)

    def __parse_packet(self, byte):
        """
        Check for packet in byte stream.
        """
        if self._packet_parse_state == 0:
            if byte == 0xAA:
                self._packet_parse_state = 1
                self._packet_data = [0xAA]

        elif self._packet_parse_state == 1:
            self._packet_parse_state = 2
            self._packet_data.append(byte)

        elif self._packet_parse_state == 2:
            self._packet_parse_state = 3
            self._packet_data.append(byte)
            self._packet_payload_size = (self._packet_data[1] | (self._packet_data[2] << 8)) >> 6
            self._packet_payload_size += 2
            self._packet_size = 3

            if self._packet_payload_size > 1019:
                self._packet_parse_state = 0

        elif self._packet_parse_state == 3:
            self._packet_data.append(byte)
            self._packet_size += 1
            self._packet_payload_size -= 1

            if self._packet_payload_size == 0:
                self._packet_parse_state = 0
                crc = self._packet_data[self._packet_size - 2] | (
                    self._packet_data[self._packet_size - 1] << 8
                )
                verify_crc = self.__create_crc(self._packet_data[0:-2])

                if crc == verify_crc:
                    return True

        return False

    def __wait_for_packet(self, port, command, timeout=1):
        """
        Wait (up to timeout) for a packet of the specified command to be received.
        """
        self._packet_parse_state = 0
        self._packet_data = []
        self._packet_payload_size = 0
        self._packet_size = 0

        end_time = time.time() + timeout

        while time.time() < end_time:
            c = port.read(1)

            if len(c) != 0:
                b = ord(c)
                if self.__parse_packet(b) is True:
                    if self._packet_data[3] == command:
                        return self._packet_data
        return

    def __execute_command(self, port, command, write, data=None, timeout=1, retries=4):
        """
        Send a request packet and wait (up to timeout) for a response.
        """
        if data is None:
            data = []

        packet = self.__build_packet(command, write, data)

        for _ in range(0, retries):
            port.write(packet)

            response = self.__wait_for_packet(port, command, timeout)

            if response is not None:
                return True, response

        return False, None

    @staticmethod
    def get_str16_from_packet(packet):
        """
        Extract a 16 byte string from a string packet.
        """
        str16 = ""
        for i in range(0, 16):
            if packet[4 + i] == 0:
                break
            str16 += chr(packet[4 + i])

        return str16

    def print_product_information(self, port):
        """
        Prints product information to console.
        """
        result, response = self.__execute_command(
            port, self.PRODUCT_NAME, self.NO_WRITE, timeout=0.1
        )
        if not result:
            return False
        print("Product: " + self.get_str16_from_packet(response))

        result, response = self.__execute_command(
            port, self.FIRMWARE_VERSION, self.NO_WRITE, timeout=0.1
        )
        if not result:
            return False
        print(f"Firmware: {response[6]}.{response[5]}.{response[4]}")

        result, response = self.__execute_command(
            port, self.SERIAL_NUMBER, self.NO_WRITE, timeout=0.1
        )
        if not result:
            return False
        print("Serial: " + self.get_str16_from_packet(response))

        return True

    def set_update_rate(self, port, value):
        """
        Set the frequency of the lidar.
        Value can be one of:
        1  = 50 Hz
        2  = 100 Hz
        3  = 200 Hz
        4  = 400 Hz
        5  = 500 Hz
        6  = 625 Hz
        7  = 1000 Hz
        8  = 1250 Hz
        9  = 1538 Hz
        10 = 2000 Hz
        11 = 2500 Hz
        12 = 5000 Hz
        """

        if value < self.MIN_UPDATE_RATE or value > self.MAX_UPDATE_RATE:
            return False

        result, response = self.__execute_command(port, self.UPDATE_RATE, self.WRITE, [value])
        if not result:
            return False, None

        return True, response

    def set_default_distance_output(self, port, use_last_return=False):
        """
        Configures the data output when using the 44. Distance data command.
        Each bit toggles the output of specific data.
        """
        if use_last_return is True:
            # Configure output to have 'last return raw' and 'yaw angle'.
            self.__execute_command(port, self.DISTANCE_OUTPUT, self.WRITE, [1, 1, 0, 0])
        else:
            # Configure output to have 'first return raw' and 'yaw angle'.
            self.__execute_command(port, self.DISTANCE_OUTPUT, self.WRITE, [8, 1, 0, 0])

    def set_distance_stream_enable(self, port, enable):
        """
        Enable and disable streaming from the lidar.
        """
        enable_data = [5, 0, 0, 0]
        disable_data = [0, 0, 0, 0]
        if enable is True:
            self.__execute_command(port, self.STREAM, self.WRITE, enable_data)
        else:
            self.__execute_command(port, self.STREAM, self.WRITE, disable_data)

    def wait_for_reading(self, port, timeout=1):
        """
        Gets lidar reading (distance in m and angle).
        """
        response = self.__wait_for_packet(port, self.DISTANCE, timeout)

        if response is None:
            return -1, 0

        distance_in_cm = response[4] << 0 | response[5] << 8
        distance_in_metres = distance_in_cm / 100.0

        yaw_angle = response[6] << 0 | response[7] << 8
        if yaw_angle > 32000:
            yaw_angle = yaw_angle - 65535

        yaw_angle /= 100.0

        return distance_in_metres, yaw_angle

    def set_speed(self, port, value):
        """
        Sets spin speed of lidar.
        Value must be between 5 and 2000 (inclusive).
        The greater the value, the slower the lidar rotates.
        """
        if value < self.MAX_SPEED or value > self.MIN_SPEED:
            return False

        low_byte = value & 0xFF
        high_byte = (value >> 8) & 0xFF

        self.__execute_command(port, self.ROTATION_SPEED, self.WRITE, [low_byte, high_byte])
        return True

    def set_low_angle(self, port, value):
        """
        Set minimum angle.
        Value must be between -170 and -5 inclusive.
        """
        if value < self.MIN_LOW_ANGLE or value > self.MAX_LOW_ANGLE:
            return False

        self.__execute_command(port, self.LOW_ANGLE, self.WRITE, list(struct.pack("<f", value)))
        return True

    def set_high_angle(self, port, value):
        """
        Set maximum angle.
        Value must be between 5 and 170 inclusive.
        """
        if value < self.MIN_HIGH_ANGLE or value > self.MAX_HIGH_ANGLE:
            return False

        self.__execute_command(port, self.HIGH_ANGLE, self.WRITE, list(struct.pack("<f", value)))
        return True
