from setuptools import find_packages, setup


package_name = 'conveyor_system_gui'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gunwon',
    maintainer_email='dlrjsdnjs111@tukorea.ac.kr',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "ui_node = conveyor_system_gui.ui_node:main",
            "image_ui = conveyor_system_gui.image_ui:main",
            "gui = conveyor_system_gui.gui:main",
        ],
    },
)
