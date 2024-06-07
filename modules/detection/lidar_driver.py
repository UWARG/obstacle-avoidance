"""
Lightware SF45/B Driver
Lidar documentation: https://support.lightware.co.za/sf45b/#/commands
NEEDS TO BE TESTED
"""

import time
import struct
import serial


class LidarDriver:
    """
    Wrapper for lidar
    """

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

    def _create_crc(self, data):
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

    def _build_packet(self, command, write, data=None):
        """
        Create raw bytes for a packet.
        """
        if data is None:
            data = []

        payload_length = 1 + len(data)
        flags = (payload_length << 6) | (write & 0x1)
        packet_bytes = [0xAA, flags & 0xFF, (flags >> 8) & 0xFF, command]
        packet_bytes.extend(data)
        crc = self._create_crc(packet_bytes)
        packet_bytes.append(crc & 0xFF)
        packet_bytes.append((crc >> 8) & 0xFF)

        return bytearray(packet_bytes)

    def _parse_packet(self, byte):
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
                verify_crc = self._create_crc(self._packet_data[0:-2])

                if crc == verify_crc:
                    return True

        return False

    def _wait_for_packet(self, port, command, timeout=1):
        """
        Wait (up to timeout) for a packet of the specified command to be received.
        """
        self._packet_parse_state = 0
        self._packet_data = []
        self._packet_payload_size = 0
        self._packet_size = 0

        end_time = time.time() + timeout

        while True:
            if time.time() >= end_time:
                return None

            c = port.read(1)

            if len(c) != 0:
                b = ord(c)
                if self.parse_packet(b) is True:
                    if self._packet_data[3] == command:
                        return self._packet_data

    def _execute_command(self, port, command, write, data=None, timeout=1):
        """
        Send a request packet and wait (up to timeout) for a response.
        """
        if data is None:
            data = []

        packet = self._build_packet(command, write, data)
        retries = 4

        while retries > 0:
            retries -= 1
            port.write(packet)

            response = self._wait_for_packet(port, command, timeout)

            if response is not None:
                return response

        # pylint: disable=broad-exception-raised
        raise Exception("LWNX command failed to receive a response.")
        # pylint: enable=broad-exception-raised

    def get_str16_from_packet(self, packet):
        """
        Extract a 16 byte string from a string packet.
        """
        str16 = ""
        for i in range(0, 16):
            if packet[4 + i] == 0:
                break
            else:
                str16 += chr(packet[4 + i])

        return str16

    def print_product_information(self, port):
        """
        Prints product information to console.
        """
        response = self._execute_command(port, 0, 0, timeout=0.1)
        print("Product: " + self.get_str16_from_packet(response))

        response = self._execute_command(port, 2, 0, timeout=0.1)
        print(f"Firmware: {response[6]}.{response[5]}.{response[4]}")

        response = self._execute_command(port, 3, 0, timeout=0.1)
        print("Serial: " + self.get_str16_from_packet(response))

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

        if value < 1 or value > 12:
            raise ValueError("Invalid update rate value.")

        self._execute_command(port, 66, 1, [value])

    def set_default_distance_output(self, port, use_last_return=False):
        """
        Configures the data output when using the 44. Distance data command.
        Each bit toggles the output of specific data.
        """
        if use_last_return is True:
            # Configure output to have 'last return raw' and 'yaw angle'.
            self._execute_command(port, 27, 1, [1, 1, 0, 0])
        else:
            # Configure output to have 'first return raw' and 'yaw angle'.
            self._execute_command(port, 27, 1, [8, 1, 0, 0])

    def set_distance_stream_enable(self, port, enable):
        """
        Enable and disable streaming from the lidar.
        """
        if enable is True:
            self._execute_command(port, 30, 1, [5, 0, 0, 0])
        else:
            self._execute_command(port, 30, 1, [0, 0, 0, 0])

    def wait_for_reading(self, port, timeout=1):
        """
        Gets lidar reading (distance and angle).
        """
        response = self._wait_for_packet(port, 44, timeout)

        if response is None:
            return -1, 0

        distance = (response[4] << 0 | response[5] << 8) / 100.0

        yaw_angle = response[6] << 0 | response[7] << 8
        if yaw_angle > 32000:
            yaw_angle = yaw_angle - 65535

        yaw_angle /= 100.0

        return distance, yaw_angle

    def set_speed(self, port, value):
        """
        Sets spin speed of lidar.
        Value must be between 5 and 2000 (inclusive).
        The greater the value, the slower the lidar rotates.
        """
        if value < 5 or value > 2000:
            raise ValueError("Invalid speed value.")

        self._execute_command(port, 85, 1, [value & 0xFF, (value >> 8) & 0xFF])

    def set_low_angle(self, port, value):
        """
        Set minimum angle.
        Value must be between -170 and -5 inclusive.
        """
        if value < -170 or value > -5:
            raise ValueError("Invalid low scan angle value.")

        self._execute_command(port, 98, 1, list(struct.pack("<f", value)))

    def set_high_angle(self, port, value):
        """
        Set maximum angle.
        Value must be between 5 and 170 inclusive.
        """
        if value < 5 or value > 170:
            raise ValueError("Invalid high scan angle value.")

        self._execute_command(port, 99, 1, list(struct.pack("<f", value)))
