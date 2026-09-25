import os

base_dir = "/home/aditi/ros-bootcamp-task-2/250057_Aditi/src"
packages = ["simulation", "robot1_slam", "robot2_slam", "bringup"]

for pkg in packages:
    os.makedirs(os.path.join(base_dir, pkg), exist_ok=True)
    os.makedirs(os.path.join(base_dir, pkg, "launch"), exist_ok=True)
    os.makedirs(os.path.join(base_dir, pkg, pkg), exist_ok=True)
    
    with open(os.path.join(base_dir, pkg, pkg, "__init__.py"), "w") as f:
        pass

    package_xml = f"""<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>{pkg}</name>
  <version>0.0.0</version>
  <description>TODO: Package description</description>
  <maintainer email="user@todo.todo">user</maintainer>
  <license>TODO: License declaration</license>
  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>
  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
"""
    with open(os.path.join(base_dir, pkg, "package.xml"), "w") as f:
        f.write(package_xml)

    setup_py = f"""from setuptools import find_packages, setup
import os
from glob import glob

package_name = '{pkg}'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={{
        'console_scripts': [
        ],
    }},
)
"""
    with open(os.path.join(base_dir, pkg, "setup.py"), "w") as f:
        f.write(setup_py)
        
    setup_cfg = f"""[develop]
script_dir=$base/lib/{pkg}
[install]
install_scripts=$base/lib/{pkg}
"""
    with open(os.path.join(base_dir, pkg, "setup.cfg"), "w") as f:
        f.write(setup_cfg)
        
    os.makedirs(os.path.join(base_dir, pkg, "resource"), exist_ok=True)
    with open(os.path.join(base_dir, pkg, "resource", pkg), "w") as f:
        pass
    
    os.makedirs(os.path.join(base_dir, pkg, "config"), exist_ok=True)
