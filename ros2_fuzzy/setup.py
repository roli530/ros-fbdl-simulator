from setuptools import setup
import os
from glob import glob
import sys
python_version = f"{sys.version_info.major}.{sys.version_info.minor}"
package_name = 'ros2_fuzzy'
submodules = "fribe"
submodules2 = "grammars/simple"
submodulesX = "exprail"  #így lehet alkönyvtárakat is bevinni.
setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name,submodules,submodules2,submodulesX],
    include_package_data=True,
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), [

                              'launch/simulation_controller.py',                           
                                              ]),

        (os.path.join('share', package_name, 'msg'), glob('msg/*')),
        (os.path.join('lib',f'python{python_version}','site-packages/ros2_fuzzy/grammars/simple'), glob(submodules2+ '/*.grammar'))
    ],
    
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='berquor',
    maintainer_email='berquor@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "agent_node = ros2_fuzzy.agent : main",
            
            "velo_pub = ros2_fuzzy.velocity_publisher : main",
            "tester_node = ros2_fuzzy.tester : main",
            "spawner = ros2_fuzzy.agent_spawner: main",
            "fbdl = ros2_fuzzy.fbdl_controller: main"
        ],
    },
)
