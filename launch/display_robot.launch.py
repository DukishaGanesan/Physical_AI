from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import os

def generate_launch_description():
        pkg_share = os.path.join(os.environ['HOME'], 'ros2_ws', 'src', 'urdf_demo')
            urdf_file = os.path.join(pkg_share, 'urdf', 'simple_robot.xacro')

                # Use xacro to convert to URDF on the fly (xacro is installed).
                    robot_state_pub_cmd = ExecuteProcess(
                                    cmd=['xacro', urdf_file],
                                            output='screen'
                                                )

                        # robot_state_publisher launching using the URDF from xacro
                            rsp_node = Node(
                                            package='robot_state_publisher',
                                                    executable='robot_state_publisher',
                                                            name='robot_state_publisher',
                                                                    output='screen',
                                                                            parameters=[{
                                                                                            'robot_description': None  # we will pass via stdin from xacro
                                                                                                    }]
                                                                                )

                                # simpler approach: run xacro and pipe to robot_state_publisher (two terminals)
                                    # but here we show running xacro first to generate URDF and then launching RSP manually.

                                        return LaunchDescription([
                                                    # We will actually generate URDF then run robot_state_publisher from terminal below.
                                                            # Keep this launch file to open rviz2 and a joint_state_publisher_gui:
                                                                    Node(
                                                                                    package='joint_state_publisher_gui',
                                                                                                executable='joint_state_publisher_gui',
                                                                                                            name='joint_state_publisher_gui',
                                                                                                                        output='screen'
                                                                                                                                ),
                                                                            Node(
                                                                                            package='rviz2',
                                                                                                        executable='rviz2',
                                                                                                                    name='rviz2',
                                                                                                                                output='screen',
                                                                                                                                            arguments=['-d', os.path.join(pkg_share, 'rviz', 'display.rviz')] if os.path.exists(os.path.join(pkg_share, 'rviz', 'display.rviz')) else []
                                                                                                                                                    ),
                                                                                ])

