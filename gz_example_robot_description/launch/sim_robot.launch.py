import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch_ros.actions import Node, SetParameter
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import xacro


def generate_launch_description():
    pkg_project_gazebo = get_package_share_directory("leo_gz_bringup")
    ld = LaunchDescription()


    # Specify the name of the package and path to xacro file within the package
    pkg_name = 'gz_example_robot_description'

    file_subpath = 'urdf/leo_sim.urdf.xacro'

    # Set ignition resource path (so it can find your world files)
    ign_resource_path = SetEnvironmentVariable(name='IGN_GAZEBO_RESOURCE_PATH',
    value=[os.path.join(get_package_share_directory(pkg_name),'worlds')])

    # Use xacro to process the file
    xacro_file = os.path.join(get_package_share_directory(pkg_name),file_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()


    # Include extra models in the world
    sdf_path = os.path.join(get_package_share_directory(pkg_name), 'worlds', 'tb3.sdf')

    if 'IGN_GAZEBO_RESOURCE_PATH' in os.environ:
        gz_world_path =  os.environ['IGN_GAZEBO_RESOURCE_PATH'] + os.pathsep + os.path.join(get_package_share_directory(pkg_name), "worlds")
    else:
        gz_world_path =  os.path.join(get_package_share_directory(pkg_name), "worlds")

    ign_resource_path_update = SetEnvironmentVariable(name='IGN_GAZEBO_RESOURCE_PATH',value=[gz_world_path])

    # Include the Gazebo launch file
    launch_gazebo = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([get_package_share_directory('ros_gz_sim'), '/launch', '/gz_sim.launch.py']),
    launch_arguments={
    'gz_args' : '-r empty.sdf'
    }.items(),
    )

        # Add features
    gz_spawn_objects = Node(package='ros_gz_sim', executable='create',
    arguments=['-file', sdf_path,
    '-x', '2.0',
    '-y', '0.5',
    '-z', '0.0'],
    output='both'
    )
 
    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    node_spawn_entity = Node(package='ros_gz_sim', executable='create',
                        arguments=['-topic', '/robot_description',
                                   '-z', '0.5'],
                        output='screen')

    # robot state publisher node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[
            {"use_sim_time": True},
            {'robot_description': robot_description_raw},
        ],  # add other parameters here if required
    )
    # Bridge
    # https://github.com/gazebosim/ros_gz/tree/humble/ros_gz_bridge

    node_ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=  [
                    "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
                    "/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist",
                    "/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
                    "/lidar/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
                    "/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V",
                    "/imu/data_raw@sensor_msgs/msg/Imu[ignition.msgs.IMU",
                    "/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
                    # "/camera/image_raw@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
                    "/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model",

            ],
        parameters= [{
                    "qos_overrides./tf_static.publisher.durability": "transient_local",
                    "qos_overrides./gz_example_robot.subscriber.reliability": "reliable",
        }],

        output='screen'
    )
    #Camera image bridge
    image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        name="image_bridge",
        arguments=["/camera/image_raw"],
        output="screen",
    )
    # joint state publisher node
    node_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
    )

    # Rviz node
    node_rviz = Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        arguments=['-d' + os.path.join(get_package_share_directory(pkg_name), 'rviz', 'nav2.rviz')]
    )


    # Add actions to LaunchDescription
    ld.add_action(SetParameter(name='use_sim_time', value=True))
    ld.add_action(ign_resource_path)
    ld.add_action(ign_resource_path_update)
    ld.add_action(launch_gazebo)
    ld.add_action(gz_spawn_objects)
    ld.add_action(node_spawn_entity)
    #Add joint state pulisher
    #ld.add_action(node_joint_state_publisher)
    ld.add_action(node_robot_state_publisher)
    ld.add_action(node_ros_gz_bridge)
    ld.add_action(image_bridge)
    # ld.add_action(node_joint_state_publisher)
    ld.add_action(node_rviz)
    return ld