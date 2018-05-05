from setuptools import setup, Extension

USE_CYTHON = True   # command line option, try-import, ...
ext = '.pyx' if USE_CYTHON else '.c'

extensions = [
  #Extension("plotting.pyxfill", ["plotting/pyxfill"+ext]),
  Extension("plotting.polyfill._polyfill", 
            ["plotting/polyfill/polyfill_wrap.cxx"],
            include_dirs=[
              "plotting/polyfill/external/range-v3/include/",
              "plotting/polyfill/external/debugbreak/"
            ],
            extra_compile_args=["-std=c++1z"],
            )
]

#if USE_CYTHON:
#    from Cython.Build import cythonize
#    extensions = cythonize(extensions)

setup(name='plotting',
      version='0.1',
      description='Plotting',
      url='http://github.com/mostlyuseful/plotting',
      author='Maurice-Pascal Sonnemann',
      author_email='mpsonnemann@gmail.com',
      license='MIT',
      packages=['plotting'],
      #py_modules= ["plotting.polyfill"],
      ext_modules = extensions,
      zip_safe=False)
