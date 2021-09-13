from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
  
d = generate_distutils_setup(
    packages=['rosen_abstract_node'],
    package_dir={'': 'src'},
)

setup(**d)