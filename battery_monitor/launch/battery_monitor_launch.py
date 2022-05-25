from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

baudRate = 115200
portMode = '8N1'

def generate_launch_description():
    deviceName = LaunchConfiguration('device_name')
    serialPortName = LaunchConfiguration('serial_port_name')

    deviceNameLaunchArg = DeclareLaunchArgument(
        'device_name',
        default_value = 'agv0'
    )

    serialPortNameLaunchArg = DeclareLaunchArgument(
        'serial_port_name',
        default_value = 'ttyS0'
    )

    batteryMeterNode = Node(
        package = "battery_monitor",
        executable = "meter",
        namespace = deviceName,
        name = "battery",
        output = "screen",
        emulate_tty = True,
        parameters = [
            {"port_name": serialPortName},
            {"baud_rate": baudRate},
            {"port_mode": portMode}
        ]
    )

    return LaunchDescription([
        deviceNameLaunchArg,
        serialPortNameLaunchArg,
        batteryMeterNode
    ])