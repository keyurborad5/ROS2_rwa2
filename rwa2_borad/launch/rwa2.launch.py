# pull in some Python launch modules.
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


# This function is needed
def generate_launch_description():
    ld = LaunchDescription()

    # Declare a command-line argument "cmd_line_parameter"
    cmd_line_parameter = DeclareLaunchArgument(
        "cmd_line_parameter",
        default_value="default_value",
        description="A parameter from the command line.",
    )

    # Path to the parameters file
    node_params = PathJoinSubstitution(
        [FindPackageShare("rwa2_borad"), "config", "params.yaml"]
    )

   
    
    # C++ nodes
    thermostat_house_cpp = Node(
        package="rwa2_borad",
        executable="thermostat_house",
        parameters=[{"cmd_line_parameter": LaunchConfiguration("cmd_line_parameter")},
                        node_params,],
    )
    # av_sensors_cpp = Node(
    #     package="av_demo",
    #     executable="av_sensors_demo",
    #     remappings=[("av_lidar","av_lidar2")],
    #     parameters=[
    #         {"cmd_line_parameter": LaunchConfiguration("cmd_line_parameter")},
    #         node_params,
    #     ],
    # )

    ld.add_action(cmd_line_parameter)
    ld.add_action(thermostat_house_cpp)


    return ld
