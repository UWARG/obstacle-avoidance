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

    MAX_HIGH_ANGLE = 170  # degrees
    MIN_HIGH_ANGLE = 5  # degrees

    MIN_LOW_ANGLE = -170  # degrees
    MAX_LOW_ANGLE = -5  # degrees

    MAX_SPEED = 5
    MIN_SPEED = 2000

    __NO_WRITE = 0
    __WRITE = 1

    __PRODUCT_NAME = 0
    __FIRMWARE_VERSION = 2
    __SERIAL_NUMBER = 3
    __DISTANCE_OUTPUT = 27
    __STREAM = 30
    __DISTANCE = 44
    __UPDATE_RATE = 66
    __ROTATION_SPEED = 85
    __LOW_ANGLE = 98
    __HIGH_ANGLE = 99

    __USE_LAST_RETURN_DATA = [1, 1, 0, 0]
    __USE_FIRST_RETURN_DATA = [8, 1, 0, 0]

    __ENABLE_STREAMING_DATA = [5, 0, 0, 0]
    __DISABLE_STREAMING_DATA = [0, 0, 0, 0]

    __YAW_ANGLE_THRESHOLD = 32000
    __YAW_ANGLE_OFFSET = 65535

    def __init__(self, port_name: str, baudrate: int, timeout: float) -> None:
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
    def __create_crc(data: "list[int]") -> int:
        """
        Create raw bytes for a packet.
        """
        if data is None:
            data = []

        payload_length = 1 + len(data)
        flags = (payload_length << 6) | (write & 0x1)
        packet_bytes = [0xAA, flags & 0xFF, (flags >> 8) & 0xFF, command]
        packet_bytes.extend(data)
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

    def __build_packet(self, command: int, write: int, data: "list[int]" = None) -> bytearray:
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

    def __parse_packet(self, byte: int) -> bool:
        """
        Check for packet in byte stream.
        """
        if self._packet_parse_state == 0:
            if byte == 0xAA:
                self._packet_parse_state = 1
                self._packet_data = [0xAA]

            return False

        if self._packet_parse_state == 1:
            self._packet_parse_state = 2
            self._packet_data.append(byte)

            return False

        if self._packet_parse_state == 2:
            self._packet_parse_state = 3
            self._packet_data.append(byte)
            self._packet_payload_size = (self._packet_data[1] | (self._packet_data[2] << 8)) >> 6
            self._packet_payload_size += 2
            self._packet_size = 3

            if self._packet_payload_size > 1019:
                self._packet_parse_state = 0

            return False

        if self._packet_parse_state == 3:
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

        return False

    def __wait_for_packet(
        self, port: serial.Serial, command: int, timeout: float = 1
    ) -> "list[int] | None":
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
        return None

    def __execute_command(
        self,
        port: serial.Serial,
        command: int,
        write: int,
        data: "list[int]" = None,
        timeout: float = 1,
        retries: int = 4,
    ) -> "tuple[bool, list[int]]":
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
    def get_str16_from_packet(packet: "list[int]") -> str:
        """
        Extract a 16 byte string from a string packet.
        """
        str16 = ""
        for i in range(0, 16):
            if packet[4 + i] == 0:
                break
            str16 += chr(packet[4 + i])

        return str16

    def print_product_information(self, port: serial.Serial) -> bool:
        """
        Prints product information to console.
        """
        result, response = self.__execute_command(
            port, self.__PRODUCT_NAME, self.__NO_WRITE, timeout=0.1
        )
        if not result:
            return False
        print("Product: " + self.get_str16_from_packet(response))

        result, response = self.__execute_command(
            port, self.__FIRMWARE_VERSION, self.__NO_WRITE, timeout=0.1
        )
        if not result:
            return False
        print(f"Firmware: {response[6]}.{response[5]}.{response[4]}")

        result, response = self.__execute_command(
            port, self.__SERIAL_NUMBER, self.__NO_WRITE, timeout=0.1
        )
        if not result:
            return False
        print("Serial: " + self.get_str16_from_packet(response))

        return True

    def set_update_rate(self, port: serial.Serial, value: int) -> bool:
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

        result, _ = self.__execute_command(port, self.__UPDATE_RATE, self.__WRITE, [value])
        if not result:
            return False

        return True

    def set_default_distance_output(
        self, port: serial.Serial, use_last_return: bool = False
    ) -> bool:
        """
        Configures the data output when using the 44. Distance data command.
        Each bit toggles the output of specific data.
        """
        if use_last_return is True:
            # Configure output to have 'last return raw' and 'yaw angle'.
            result, _ = self.__execute_command(
                port, self.__DISTANCE_OUTPUT, self.__WRITE, self.__USE_LAST_RETURN_DATA
            )
            if not result:
                return False
            return True

        # Configure output to have 'first return raw' and 'yaw angle'.
        result, _ = self.__execute_command(
            port, self.__DISTANCE_OUTPUT, self.__WRITE, self.__USE_FIRST_RETURN_DATA
        )
        if not result:
            return False
        return True

    def set_distance_stream_enable(self, port: serial.Serial, enable: bool) -> bool:
        """
        Enable and disable streaming from the lidar.
        """
        enable_data = [5, 0, 0, 0]
        disable_data = [0, 0, 0, 0]
        if enable is True:
            result, _ = self.__execute_command(
                port, self.__STREAM, self.__WRITE, self.__ENABLE_STREAMING_DATA
            )
            if not result:
                return False
            return True

        result, _ = self.__execute_command(
            port, self.__STREAM, self.__WRITE, self.__DISABLE_STREAMING_DATA
        )
        if not result:
            return False
        return True

    def wait_for_reading(self, port: serial.Serial, timeout: float = 1) -> "tuple[float, float]":
        """
        Gets lidar reading (distance in m and angle).
        """
        response = self.__wait_for_packet(port, self.__DISTANCE, timeout)

        if response is None:
            return -1, 0

        distance_in_cm = response[4] << 0 | response[5] << 8
        distance_in_metres = distance_in_cm / 100.0

        if distance_in_metres < 0 or distance_in_metres > 50:
            return -1, 0

        angle_in_hundreth_degrees = response[6] << 0 | response[7] << 8

        yaw_angle = angle_in_hundreth_degrees / 100.0

        if yaw_angle > self.__YAW_ANGLE_THRESHOLD:
            yaw_angle = yaw_angle - self.__YAW_ANGLE_OFFSET

        if yaw_angle <= self.MIN_LOW_ANGLE or yaw_angle >= self.MAX_HIGH_ANGLE:
            return -1, 0

        return distance_in_metres, yaw_angle

    def set_speed(self, port: serial.Serial, value: int) -> bool:
        """
        Sets spin speed of lidar.
        Value must be between 5 and 2000 (inclusive).
        The greater the value, the slower the lidar rotates.
        """
        if value < self.MAX_SPEED or value > self.MIN_SPEED:
            return False

        low_byte = value & 0xFF
        high_byte = (value >> 8) & 0xFF

        result, _ = self.__execute_command(
            port, self.__ROTATION_SPEED, self.__WRITE, [low_byte, high_byte]
        )
        if not result:
            return False
        return True

    def set_low_angle(self, port: serial.Serial, value: float) -> bool:
        """
        Set minimum angle.
        Value must be between -170 and -5 inclusive.
        """
        if value < self.MIN_LOW_ANGLE or value > self.MAX_LOW_ANGLE:
            return False

        result, _ = self.__execute_command(
            port, self.__LOW_ANGLE, self.__WRITE, list(struct.pack("<f", value))
        )
        if not result:
            return False
        return True

    def set_high_angle(self, port: serial.Serial, value: float) -> bool:
        """
        Set maximum angle.
        Value must be between 5 and 170 inclusive.
        """
        if value < self.MIN_HIGH_ANGLE or value > self.MAX_HIGH_ANGLE:
            return False

        result, _ = self.__execute_command(
            port, self.__HIGH_ANGLE, self.__WRITE, list(struct.pack("<f", value))
        )
        if not result:
            return False
        return True
