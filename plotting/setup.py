from setuptools import setup, Extension

source_names = [
    'blow_up.cpp',
    'candidate.cpp',
    'dpath_utils.cpp',
    'edge.cpp',
    'intersection.cpp',
    'line.cpp',
    'mergepaths.cpp',
    'merger_rtree.cpp',
    'parametricline.cpp',
    'pnpoly_rtree.cpp',
    'path.cpp',
    'polygon.cpp',
    'raster.cpp',
    'polyfill_wrap_pybind.cxx',
    'external/polyclipping/cpp/clipper.cpp'

]
source_paths = ["plotting/polyfill/" + name for name in source_names]

extensions = [
    Extension("plotting.polyfill._polyfill",
              source_paths,
              include_dirs=[
                  "plotting/polyfill/external/pybind11/include",
                  "plotting/polyfill/external/range-v3/include",
                  "plotting/polyfill/external/debugbreak",
                  "plotting/polyfill/external/polyclipping/cpp",
                  "plotting/polyfill/external/cppitertools"
              ],
              libraries=["python3.5m"],
              # , "-ftime-report"],
              extra_compile_args=["-std=c++1z",
                                  "-fdiagnostics-color", "-march=native"]
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
      # py_modules= ["plotting.polyfill"],
      ext_modules=extensions,
      zip_safe=False)
