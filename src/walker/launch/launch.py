from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    # Declare a launch argument to enable or disable rosbag recording
    record_bag_arg = DeclareLaunchArgument(
        "record_bag", 
        default_value="False", 
        choices=["True", "False"],
        description="Set this to True to start recording a rosbag file during the launch."
    )

    # Declare a launch argument to stop the launch after a set duration (30 seconds)
    stop_args = DeclareLaunchArgument(
        "stop", 
        default_value="False", 
        choices=["True", "False"],
        description="Set this to True to automatically stop the launch after 30 seconds."
    )

    # Define the Walker node that controls the robot's movement and behavior
    walker_node = Node(
        package='walker',         
        executable='walker_node', 
        name='walker',            
        output='screen'           
    )

    # Define a process to record a rosbag if the 'record_bag' argument is set to True
    recorder_node = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration("record_bag")),  
        cmd=[
            "bash", "-c",  
            # Check and clean up any previous rosbag files in the 'results/bag_list/' directory
            "if [ -d 'results/bag_list/' ]; then rm -rf results/bag_list/; fi; "
            # Start recording all topics except those starting with '/camera/*'
            "ros2 bag record -o results/bag_list/ --all -x '/camera/*'"
        ],
        output="screen"
    )

    # Create a timer action to automatically stop the launch after 30 seconds if 'stop' argument is True
    stop_action = TimerAction(
        period=30.0,  # Duration (in seconds) before shutting down the launch
        actions=[Shutdown()],  
        condition=IfCondition(LaunchConfiguration("stop"))  # Run only if 'stop' is True
    )

    # Return a LaunchDescription object that includes all the declared arguments, nodes, and actions
    return LaunchDescription([
        record_bag_arg, 
        stop_args,      
        walker_node,    
        recorder_node, 
        stop_action      
    ])
