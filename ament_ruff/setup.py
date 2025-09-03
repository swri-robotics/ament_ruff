from setuptools import find_packages, setup

package_name = 'ament_ruff'

setup(
    name=package_name,
    version='0.0.2',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'argcomplete',
        'ruff',
        'setuptools',
        'unidiff',
    ],
    package_data={
        '': [
            'configuration/ruff.toml',
        ],
    },
    zip_safe=True,
    maintainer='Ben Andrew',
    maintainer_email='benjamin.andrew@swri.org',
    description='Python linter/formatter using ruff',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['ament_ruff = ament_ruff.main:main'],
        'pytest11': [
            'ament_ruff = ament_ruff.pytest_marker',
        ],
    },
)
