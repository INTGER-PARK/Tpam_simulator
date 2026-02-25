from setuptools import setup
from glob import glob

package_name = 'plant'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
	('share/' + package_name + '/xml', glob('xml/*.xml') + glob('xml/*.stl') + glob('xml/*.STL')),
	('share/' + package_name + '/xml/arm_assets', glob('xml/arm_assets/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ryung',
    maintainer_email='tlsgksfbd@seoultech.ac.kr',
    description='MuJoCo plant loader',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'plant = plant.plant:main',
        ],
    },
)
