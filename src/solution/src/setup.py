from setuptools import setup
from Cython.Build import cythonize

setup(
    name='Detection module for puzzlebot',
    ext_modules=cythonize("color_detection.pyx"),
    zip_safe=False,
)