from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    deviceName = LaunchConfiguration('device_name')
    canId = LaunchConfiguration('can_id')

    deviceNameLaunchArg = DeclareLaunchArgument(
        'device_name',
        default_value = 'agv0'
    )

    canIdLaunchArg = DeclareLaunchArgument(
        'can_id',
        default_value = '1'
    )

    batteryMeterNode = Node(
        package = "battery_monitor",
        executable = "meter",
        namespace = deviceName,
        name = "battery",
        output = "screen",
        emulate_tty = True,
        parameters = [
            {"can_id": canId},
        ]
    )

    return LaunchDescription([
        deviceNameLaunchArg,
        canIdLaunchArg,
        batteryMeterNode
    ])