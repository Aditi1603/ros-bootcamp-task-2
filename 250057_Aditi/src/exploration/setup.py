from setuptools import find_packages, setup

package_name = 'exploration'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'opencv-python', 'numpy'],
    zip_safe=True,
    maintainer='Aditi',
    maintainer_email='aditi@todo.todo',
    description='Frontier-based Autonomous Exploration Node',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'frontier_explorer = exploration.frontier_explorer:main',
        ],
    },
)
