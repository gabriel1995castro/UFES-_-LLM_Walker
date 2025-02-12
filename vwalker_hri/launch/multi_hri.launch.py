# Librerias para configuracion de launch
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

# Funcion para generacion de launch
def generate_launch_description():
    # Definicionde variables de paquetes propios
    vwalker_hri_pkg = get_package_share_directory("vwalker_hri")
    # Extraccion de path de paquetes de configuracion
    force_filter_node_path = Path(vwalker_hri_pkg, 'param', 'force_filter_node.yaml')
    admittance_controller_node_path = Path(vwalker_hri_pkg, 'param', 'admittance_controller_node.yaml')
    superviser_node_path = Path(vwalker_hri_pkg, 'param', 'superviser_node.yaml')

    # Seccion de creacion de nodos para ejecucion de launch
    force_filter_node_cmd = Node(
        package = "vwalker_hri",
        executable='force_filter_node',
        name='force_filter',
        output='screen',
        parameters=[force_filter_node_path]
    )
    admittance_controller_node_cmd = Node(
        package = "vwalker_hri",
        executable='multi_admitance_controller_node',
        name='multi_admitance_controller',
        output='screen',
        parameters=[admittance_controller_node_path]
    )
    superviser_node_cmd = Node(
        package = "vwalker_hri",
        executable='superviser_node',
        name='superviser',
        output='screen',
        parameters=[superviser_node_path]
    )
    path_following_cmd = Node(
        package='vwalker_hri',
        executable='path_following',
        name='path_following',
        output='screen',
    )

    # Creacion de ejecucion final
    ld = LaunchDescription()
    ld.add_action(force_filter_node_cmd)
    ld.add_action(admittance_controller_node_cmd)
    ld.add_action(superviser_node_cmd)
    ld.add_action(path_following_cmd)
    
    return ld
