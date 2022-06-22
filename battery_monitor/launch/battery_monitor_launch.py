from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    deviceName = LaunchConfiguration('device_name')
    canbusInterfaceName = LaunchConfiguration('canbus_interface_name')
    canId = LaunchConfiguration('can_id')

    deviceNameLaunchArg = DeclareLaunchArgument(
        'device_name',
        default_value = 'agv0'
    )

    canbusInterfaceNameLaunchArg = DeclareLaunchArgument(
        'canbus_interface_name',
        default_value = 'can0'
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
        ],
        remappings = [
            ("battery/input/can", ["/", deviceName, "/", canbusInterfaceName, "/output/data"]),
        ],
    )

    return LaunchDescription([
        deviceNameLaunchArg,
        canbusInterfaceNameLaunchArg,
        canIdLaunchArg,
        batteryMeterNode
    ])