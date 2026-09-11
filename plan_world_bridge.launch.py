from pathlib import Path

from launch import LaunchDescription
from launch_ros.actions import Node

from vrx_gz.model import Model


WORLD_NAME = 'plan_world'
MODEL_NAME = 'wamv'
MODEL_TYPE = 'wam-v'

MODEL_SDF = Path.home() / 'vrx_ws/my_plan_world/boat_real/model.sdf'


def generate_launch_description():

    if not MODEL_SDF.exists():
        raise RuntimeError(f'boat_real SDF not found: {MODEL_SDF}')

    sdf_text = MODEL_SDF.read_text()

    # 建立一个 VRX Model 对象，但不负责生成 / spawn 船。
    model = Model(
        MODEL_NAME,
        MODEL_TYPE,
        (0, 0, 0, 0, 0, 0),
    )

    # 直接复用 VRX 原本的 SDF -> payload 解析机制。
    model.payload = model.payload_from_sdf(sdf_text)

    print('\n========== BOAT PAYLOAD ==========')
    for name, value in model.payload.items():
        print(f'{name}: link={value[0]}, type={value[1]}')
    print('===================================\n')

    bridges, nodes, _ = model.payload_bridges(WORLD_NAME)

    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='wamv_payload_bridge',
        output='screen',
        arguments=[bridge.argument() for bridge in bridges],
        remappings=[bridge.remapping() for bridge in bridges],
    )

    return LaunchDescription(
        nodes + [bridge_node]
    )
