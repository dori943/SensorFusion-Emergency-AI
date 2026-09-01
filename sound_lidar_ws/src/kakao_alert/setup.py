from setuptools import setup

package_name = 'kakao_alert'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 설정 예시는 share에도 깔아둔다. 실제 토큰이 든
        # kakao_config.json은 커밋하지 말 것(.gitignore).
        ('share/' + package_name + '/config',
            ['kakao_alert/kakao_config.example.json']),
    ],
    # 노드가 __file__ 기준으로 설정 파일을 찾으므로 패키지에 포함시킨다.
    # KAKAO_CONFIG 환경변수로 경로를 덮어쓸 수 있다.
    package_data={package_name: ['kakao_config*.json']},
    include_package_data=True,
    install_requires=['setuptools', 'requests'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='카카오톡 알림 노드 (낙상/SOS 최종 판정 수신)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'kakao_alert_node = kakao_alert.kakao_alert_node:main',
        ],
    },
)