'''
Written by yufeng.wu0902@gmail.com

Helper functions for operate.py.
'''

import serial.tools.list_ports
import logging
import sys


class Q8Logger:
    """
    Logging wrapper for Q8bot application.

    Provides consistent logging across all modules with configurable log levels.
    Default level is INFO, use debug mode for detailed output.
    """

    _instance = None
    _logger = None

    def __init__(self, debug=False):
        """
        Initialize Q8Logger.

        Args:
            debug: bool, if True sets level to DEBUG, otherwise INFO
        """
        if Q8Logger._logger is None:
            Q8Logger._logger = logging.getLogger('q8bot')
            Q8Logger._logger.setLevel(logging.DEBUG if debug else logging.INFO)

            # Create console handler
            handler = logging.StreamHandler(sys.stdout)
            handler.setLevel(logging.DEBUG if debug else logging.INFO)

            # Create formatter
            formatter = logging.Formatter('%(levelname)s: %(message)s')
            handler.setFormatter(formatter)

            # Add handler
            Q8Logger._logger.addHandler(handler)

    @staticmethod
    def get_logger():
        """Get the logger instance."""
        if Q8Logger._logger is None:
            Q8Logger()
        return Q8Logger._logger

    @staticmethod
    def debug(msg):
        """Log debug message."""
        Q8Logger.get_logger().debug(msg)

    @staticmethod
    def info(msg):
        """Log info message."""
        Q8Logger.get_logger().info(msg)

    @staticmethod
    def warning(msg):
        """Log warning message."""
        Q8Logger.get_logger().warning(msg)

    @staticmethod
    def error(msg):
        """Log error message."""
        Q8Logger.get_logger().error(msg)


class XiaoPortFinder:
    """
    Utility class for detecting and validating Seeed Studio XIAO ESP32C3 devices.

    The XIAO ESP32C3 is identified by its unique USB Vendor ID (VID) and Product ID (PID).
    """

    # Seeed Studio XIAO ESP32C3 USB identifiers
    VID = 0x303A
    PID = 0x1001

    @classmethod
    def find(cls):
        """
        Find the first connected XIAO ESP32C3 device.

        Returns:
            str: COM port device name (e.g., "COM3" or "/dev/ttyUSB0"), or None if not found

        Example:
            >>> port = XiaoPortFinder.find()
            >>> if port:
            ...     print(f"Found XIAO at {port}")
        """
        ports = serial.tools.list_ports.comports()
        for port in ports:
            if port.vid == cls.VID and port.pid == cls.PID:
                return port.device
        return None

    @classmethod
    def validate(cls, com_port):
        """
        Validate if a given COM port is a XIAO ESP32C3 device.

        Args:
            com_port (str): COM port device name to check

        Returns:
            bool: True if the port is a XIAO device, False otherwise

        Example:
            >>> if XiaoPortFinder.validate("COM3"):
            ...     print("COM3 is a valid XIAO device")
        """
        ports = serial.tools.list_ports.comports()
        for port in ports:
            if port.device == com_port and port.vid == cls.VID and port.pid == cls.PID:
                return True
        return False

    @classmethod
    def list_all(cls):
        """
        List all connected XIAO ESP32C3 devices.

        Returns:
            list: List of COM port device names for all connected XIAO devices

        Example:
            >>> devices = XiaoPortFinder.list_all()
            >>> print(f"Found {len(devices)} XIAO device(s)")
        """
        ports = serial.tools.list_ports.comports()
        xiao_ports = []
        for port in ports:
            if port.vid == cls.VID and port.pid == cls.PID:
                xiao_ports.append(port.device)
        return xiao_ports



