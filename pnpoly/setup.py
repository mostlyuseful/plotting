from distutils.core import setup, Extension
from Cython.Build import cythonize

ext = Extension('cypnpoly', sources = ['cypnpoly.pyx', 'pnpoly.c'])
setup(name="C pnpoly", ext_modules = cythonize([ext]))
