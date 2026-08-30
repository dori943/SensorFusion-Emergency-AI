from setuptools import find_packages, setup

package_name = 'sos_voice'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/models', ['models/model_distilled_float.pt']),
    ],
    install_requires=['setuptools', 'torch', 'numpy', 'librosa'],
    zip_safe=True,
    maintainer='TODO',
    maintainer_email='TODO@example.com',
    description='노인 응급상황 소리 감지 ROS2 노드 (경량 CNN, 라즈베리파이용)',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'emergency_detector_node = sos_voice.emergency_detector_node:main',
        ],
    },
)
