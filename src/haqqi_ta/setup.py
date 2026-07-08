from setuptools import setup
import os
from glob import glob

package_name = 'haqqi_ta'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        # Wajib: registrasi package ke ament index
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        # Wajib: package.xml
        (
            'share/' + package_name,
            ['package.xml']
        ),
        # Launch files
        (
            os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))
        ),
        # Parameter files (yaml)
        (
            os.path.join('share', package_name, 'param'),
            glob(os.path.join('param', '*.yaml'))
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Faiza Haqqi',
    maintainer_email='5022221135@student.its.ac.id',
    description='Multi-Agent Coordination Control — 3 Yahboom Mecanum Robots',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Layer 2 — Global Path (Dijkstra, refactor internal)
            'global_path_node = haqqi_ta.global_path_node:main',

            # Layer 3 — Modified DWA
            'modified_dwa_node = haqqi_ta.modified_dwa_node:main',

            # Layer 4 — Average Consensus
            'consensus_node = haqqi_ta.consensus_node:main',

            # Layer 5 — Stop-and-Go Priority Manager
            'priority_manager_node = haqqi_ta.priority_manager_node:main',

            # Support — Software Fault Injection
            'fault_injector_node = haqqi_ta.fault_injector_node:main',

            # Support — Experiment Logger
            'experiment_logger_node = haqqi_ta.experiment_logger_node:main',
            # Backward-compatible alias: command lama membuka master CLI baru.
            'experiment_cli = haqqi_ta.experiment_master_cli:main',
            'experiment_master_cli = haqqi_ta.experiment_master_cli:main',
            'udp_sender_node = haqqi_ta.udp_sender_node:main',
            'udp_receiver_node = haqqi_ta.udp_receiver_node:main',
            'udp_bridge_pc = haqqi_ta.udp_bridge_pc:main',

            # Support — Sync Monitor (debugging sinkronisasi arrival)
            'sync_monitor_node = haqqi_ta.sync_monitor_node:main',

            # Support — Formation Manager (distribusi goal ke semua robot)
            'formation_manager_node = haqqi_ta.formation_manager_node:main',
        ],
    },
)
