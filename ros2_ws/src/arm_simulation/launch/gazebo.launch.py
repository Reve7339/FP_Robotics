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

    # 4. Lanzamiento de Gazebo Classic (gzserver y gzclient) con mundo que contiene gazebo_ros_state
    world_file = os.path.join(pkg_share, 'config', 'empty_state.world')
    if not os.path.exists(world_file):
        source_world = '/home/reve/Documents/UC/robotica/finalProject/ros2_ws/src/arm_simulation/config/empty_state.world'
        if os.path.exists(source_world):
            world_file = source_world

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={
            'verbose': 'true',
            'world': world_file
        }.items()
    )


    # 5. Nodo para instanciar (spawn) el robot en la simulación física de Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'arm_3gdl', '-timeout', '120'],
        output='screen'
    )

    # Spawn de la esfera del espacio de trabajo
    sphere_urdf_file = os.path.join(pkg_share, 'urdf', 'workspace_sphere.urdf')
    if not os.path.exists(sphere_urdf_file):
        source_urdf = '/home/reve/Documents/UC/robotica/finalProject/ros2_ws/src/arm_simulation/urdf/workspace_sphere.urdf'
        if os.path.exists(source_urdf):
            sphere_urdf_file = source_urdf

    spawn_sphere = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-file', sphere_urdf_file, '-entity', 'workspace_sphere', '-x', '0.0', '-y', '0.0', '-z', '0.096173'],
        output='screen'
    )

    # Spawn de la cinta transportadora (conveyor_belt)
    conveyor_urdf_file = os.path.join(pkg_share, 'urdf', 'conveyor_belt.urdf')
    if not os.path.exists(conveyor_urdf_file):
        source_conveyor = '/home/reve/Documents/UC/robotica/finalProject/ros2_ws/src/arm_simulation/urdf/conveyor_belt.urdf'
        if os.path.exists(source_conveyor):
            conveyor_urdf_file = source_conveyor

    spawn_conveyor = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-file', conveyor_urdf_file, '-entity', 'conveyor_belt', '-x', '0.50', '-y', '0.0', '-z', '0.02'],
        output='screen'
    )

    # Spawn de los 7 cilindros con letras aleatorias no repetidas
    import random
    import json
    
    pool_letters = ['A', 'D', 'R', 'I', 'N', 'E', 'O']
    random.shuffle(pool_letters)
    
    cylinder_names = ['cylinder_A1', 'cylinder_D', 'cylinder_R', 'cylinder_I', 'cylinder_A2', 'cylinder_N', 'cylinder_A3']
    y_positions = [0.0, -0.375, -0.75, -1.125, -1.50, -1.875, -2.25]
    
    mapping = {}
    cylinder_configs = []
    for name, pos, letter in zip(cylinder_names, y_positions, pool_letters):
        mapping[name] = letter
        urdf_file = f"cutting_target_{letter}.urdf"
        cylinder_configs.append((name, urdf_file, pos))
        
    ui_letters_path = "/home/reve/Documents/UC/robotica/finalProject/ui/target_letters.json"
    try:
        with open(ui_letters_path, "w") as f:
            json.dump(mapping, f, indent=2)
        print(f"Mapeo de letras aleatorias guardado en: {ui_letters_path}")
    except Exception as e:
        print(f"Error escribiendo target_letters.json: {e}")
    
    cylinders_to_spawn = []

    for entity_name, urdf_filename, y_pos in cylinder_configs:
        urdf_path = os.path.join(pkg_share, 'urdf', urdf_filename)
        if not os.path.exists(urdf_path):
            source_urdf = os.path.join('/home/reve/Documents/UC/robotica/finalProject/ros2_ws/src/arm_simulation/urdf', urdf_filename)
            if os.path.exists(source_urdf):
                urdf_path = source_urdf

        spawn_node = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-file', urdf_path, '-entity', entity_name, '-x', '0.50', '-y', str(y_pos), '-z', '0.055'],
            output='screen'
        )
        cylinders_to_spawn.append(spawn_node)

    # Spawn de la luz del puntero láser (laser_beam)
    laser_urdf_file = os.path.join(pkg_share, 'urdf', 'laser_beam.urdf')
    if not os.path.exists(laser_urdf_file):
        source_laser = '/home/reve/Documents/UC/robotica/finalProject/ros2_ws/src/arm_simulation/urdf/laser_beam.urdf'
        if os.path.exists(source_laser):
            laser_urdf_file = source_laser

    spawn_laser = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-file', laser_urdf_file, '-entity', 'laser_beam', '-x', '0.0', '-y', '0.0', '-z', '-10.0'],
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
    launch_entities = [
        gazebo,
        robot_state_publisher_node,
        spawn_entity,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[
                    spawn_joint_state_broadcaster,
                    spawn_sphere,
                    spawn_conveyor,
                    spawn_laser
                ] + cylinders_to_spawn,
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_joint_state_broadcaster,
                on_exit=[spawn_arm_controller],
            )
        )
    ]

    return LaunchDescription(launch_entities)
