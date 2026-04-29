from setuptools import find_packages, setup  # Import setuptools helpers: find_packages() discovers Python packages; setup() defines package metadata/install rules

package_name = 'tutorial_pubsub_py'  # Define the ROS 2 package name (used repeatedly to avoid hardcoding strings)

setup(  # Begin the setuptools configuration block (this is what "colcon build" / pip uses to install the Python package)
    name=package_name,  # Package distribution name (what gets installed/registered)
    version='0.0.0',  # Package version string (semantic versioning is typical; tutorial templates often start at 0.0.0)
    packages=find_packages(exclude=['test']),  # Automatically include all Python subpackages except the 'test' folder
    data_files=[  # List of non-Python files to install into the share/ directories so ROS 2 can discover the package
        ('share/ament_index/resource_index/packages',  # Install location for ament index resources (so ROS 2 can find the package)
            ['resource/' + package_name]),  # Install the resource marker file named after the package (required for ament)
        ('share/' + package_name, ['package.xml']),  # Install package.xml into the package share directory (ROS metadata)
    ],
    install_requires=['setuptools'],  # Runtime dependency needed to install/use the package (setuptools must be available)
    zip_safe=True,  # Whether the package can be safely installed/used as a zipped .egg (often True in templates)
    maintainer='ros',  # Name of the maintainer (person responsible for the package)
    maintainer_email='202304626@alu.comillas.edu',  # Email of the maintainer (contact information)
    description='Examples of minimal publisher/subscriber using rclpy',  # Human-readable package description (should be replaced with a real description)
    license='Apache License 2.0',  # License identifier/text (should be replaced, e.g., 'Apache License 2.0')
    extras_require={  # Optional extra dependencies (installed only if requested), commonly used for testing tools
        'test': [  # Extra group name 'test' (used for development/testing)
            'pytest',  # Add pytest as a testing dependency
        ],
    },
    entry_points={  # Define executable entry points (scripts) that can be run from the command line
        'console_scripts': [  # ROS 2 uses this to expose nodes as executables via "ros2 run <pkg> <executable>" the should be named as it is needed!
                'talker = tutorial_pubsub_py.publisher_member_function:main',
                'listener = tutorial_pubsub_py.subscriber_member_function:main',
        ],  # Empty list means no executables are currently registered (you must add your node entry points here)
    },
)  # End of setup() call

