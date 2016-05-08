from distutils.core import setup
from distutils.extension import Extension
from Cython.Build import cythonize

setup(ext_modules = cythonize([
    Extension("tm1637",
            sources=["tm1637.pyx"],                 # our Cython source
            libraries=["wiringPi"]
            )]))
