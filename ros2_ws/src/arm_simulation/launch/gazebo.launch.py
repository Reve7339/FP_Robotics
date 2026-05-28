import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # 1. Obtener la ruta del paquete y el archivo xacro
    pkg_share = get_package_share_directory('arm_simulation')
    xacro_file = os.path.join(pkg_share, 'urdf', 'arm.urdf.xacro')

    # Configurar GAZEBO_MODEL_PATH para que Gazebo encuentre las mallas y los modelos del sistema (sun, ground_plane)
    pkg_share_parent = os.path.dirname(pkg_share)
    default_gazebo_models = '/usr/share/gazebo-11/models:/usr/share/gazebo/models'
    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] = pkg_share_parent + ':' + default_gazebo_models + ':' + os.environ['GAZEBO_MODEL_PATH']
    else:
        os.environ['GAZEBO_MODEL_PATH'] = pkg_share_parent + ':' + default_gazebo_models

    # Desactivar la base de datos de modelos online para evitar bloqueos por timeout
    os.environ['GAZEBO_MODEL_DATABASE_URI'] = ''

    # 2. Procesar el archivo Xacro a URDF
    import re
    robot_description_config = xacro.process_file(xacro_file)
    robot_desc = robot_description_config.toxml()
    # Eliminar todos los comentarios XML (incluyendo cabeceras autogeneradas por xacro)
    # para evitar fallos de parseo en el sistema de parámetros de ROS 2 y gazebo_ros2_control
    robot_desc = re.sub(r'<!--.*?-->', '', robot_desc, flags=re.DOTALL)

    # 3. robot_state_publisher: Publica transformaciones de coordenadas TF
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]
    )

    # 4. Lanzamiento de Gazebo Classic (gzserver y gzclient)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'verbose': 'true'}.items()
    )

    # 5. Nodo para instanciar (spawn) el robot en la simulación física de Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'arm_3gdl', '-timeout', '120'],
        output='screen'
    )

    # 6. Spawner del Joint State Broadcaster (publica en /joint_states)
    spawn_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output='screen'
    )

    # 7. Spawner del controlador de posición de las articulaciones (JointGroupPositionController)
    spawn_arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
        output='screen'
    )

    # Retornar la descripción controlando la secuencia de lanzamiento:
    # 1. Ejecutar Gazebo, robot_state_publisher y spawn_entity.
    # 2. Al finalizar spawn_entity, instanciar joint_state_broadcaster.
    # 3. Al finalizar joint_state_broadcaster, instanciar arm_controller.
    return LaunchDescription([
        gazebo,
        robot_state_publisher_node,
        spawn_entity,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[spawn_joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_joint_state_broadcaster,
                on_exit=[spawn_arm_controller],
            )
        )
    ])
