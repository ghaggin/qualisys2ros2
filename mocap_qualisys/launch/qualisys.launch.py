import launch
import launch.actions
import launch.substitutions
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'node_prefix',
            default_value=[launch.substitutions.EnvironmentVariable('USER'), '_'],
            description='Prefix for node names'
        ),
        launch_ros.actions.Node(
            package='mocap_qualisys', 
            node_executable='mocap_qualisys_node',
            output='screen',
            emulate_tty=True,
            node_name=[
                launch.substitutions.LaunchConfiguration('node_prefix'),
                'mocap_qualisys_node'],
            parameters=[{
                'server_address':'192.168.129.216',
                'server_base_port':'22222',
                'frame_rate':'100',
                'max_accel':'10.0',
                'publish_tf':'true',
                'fixed_frame_id':'mocap',
                'model_list':['1'],
            }]
        ),
    ])