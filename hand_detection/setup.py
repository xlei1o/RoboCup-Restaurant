from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
import pcl_ros
d = generate_distutils_setup(
    packages=['hand_detection'],
    package_dir={'': 'scripts'}
)
setup(**d)