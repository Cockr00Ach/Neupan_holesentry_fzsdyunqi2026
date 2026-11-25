from setuptools import find_packages, setup

package_name = 'cmdtreeautoaim'

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
    maintainer='baymaxwish',
    maintainer_email='baymaxwish@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher = cmdtreeautoaim.publisher_node:main',
            'number_publisher = cmdtreeautoaim.number_publisher:main',
        ],
    },
)
