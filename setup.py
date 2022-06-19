from setuptools import setup

package_name = 'image_manager'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='black_wraith',
    maintainer_email='orpiczymejl@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_saving_node = image_manager.image_saving_node:main',
            'image_recognizing_node = image_manager.image_recognizing_node:main',
        ],
    },
)
