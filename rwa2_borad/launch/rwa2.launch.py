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
        "mode",
        default_value="away",
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
        parameters=[{"mode": LaunchConfiguration("mode")},
                        node_params,],
    )

    thermostat_kitchen_cpp = Node(
        package="rwa2_borad",
        executable="thermostat_house",
        name="thermostat_kitchen",
        output='screen',
        remappings=[("temperature","thermostat_kitchen")],
        # parameters=[node_params],
        parameters=[{"mode": LaunchConfiguration("mode")},
                        node_params,],
    )

    thermostat_livingroom_cpp = Node(
        package="rwa2_borad",
        executable="thermostat_house",
        name="thermostat_livingroom",
        output='screen',
        remappings=[("temperature","thermostat_livingroom")],
        # parameters=[node_params],
        parameters=[{"mode": LaunchConfiguration("mode")},
                        node_params,],
    )
    thermostat_bedroom_cpp = Node(
        package="rwa2_borad",
        executable="thermostat_house",
        name="thermostat_bedroom",
        output='screen',
        remappings=[("temperature","thermostat_bedroom")],
        # parameters=[node_params],
        parameters=[{"mode": LaunchConfiguration("mode")},
                        node_params,],
    )
    # 
    ld.add_action(cmd_line_parameter)
    # ld.add_action(thermostat_house_cpp)
    ld.add_action(thermostat_kitchen_cpp)
    ld.add_action(thermostat_livingroom_cpp)
    ld.add_action(thermostat_bedroom_cpp)



    return ld
