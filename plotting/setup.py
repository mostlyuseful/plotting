from setuptools import setup, Extension

extensions = [
    Extension("plotting.polyfill._polyfill",
              # ["plotting/polyfill/polyfill_wrap_boost.cxx"],
              [
                "plotting/polyfill/polyfill_wrap_pybind.cxx",
                "plotting/polyfill/external/polyclipping/cpp/clipper.cpp"
              ],
              include_dirs=[
                  "plotting/polyfill/external/pybind11/include",
                  "plotting/polyfill/external/range-v3/include",
                  "plotting/polyfill/external/debugbreak",
                  "plotting/polyfill/external/polyclipping/cpp"
              ],
              libraries=["boost_python-py35", "python3.5m"],
              extra_compile_args=["-std=c++1z"] #, "-ftime-report"],
              )
]

setup(name='plotting',
      version='0.1',
      description='Plotting',
      url='http://github.com/mostlyuseful/plotting',
      author='Maurice-Pascal Sonnemann',
      author_email='mpsonnemann@gmail.com',
      license='MIT',
      packages=['plotting'],
      #py_modules= ["plotting.polyfill"],
      ext_modules=extensions,
      zip_safe=False)
