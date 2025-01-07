from setuptools import find_packages, setup

package_name = 'propeller_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'jinja2',
        'markupsafe',
        ],
    zip_safe=True,
    maintainer='koki-22',
    maintainer_email='amakou2626@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'propeller_control = propeller_control.propeller_control_node:main',
        ],
    },
)
