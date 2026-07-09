from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node

def generate_launch_description():
    
    plant = Node(
        package="plant",
        executable="plant",  
        name="plant",
        output="screen"
    )

    wrench_controller = Node(
        package="tpam_controller",
        executable="wrench_controller",
        name="wrench_controller",
        output="screen"
    )

    torque_dob = Node(
        package="tpam_controller",
        executable="torque_dob",
        name="torque_dob",
        output="screen"
    )

    allocator_controller = Node(
        package="tpam_controller",
        executable="allocator_controller",
        name="allocator_controller",	
        output="screen"
    )

    start_controllers_after_plant = RegisterEventHandler(
        OnProcessStart(
            target_action=plant,
            on_start=[wrench_controller, torque_dob, allocator_controller]
        )
    )


    return LaunchDescription([
        plant,
        start_controllers_after_plant
    ])
