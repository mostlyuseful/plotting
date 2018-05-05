from setuptools import setup, Extension

extensions = [
  Extension("_polyfill", 
            ["polyfill_wrap.cxx"],
            include_dirs=[
              "external/range-v3/include/",
              "external/debugbreak/"
              ],
            extra_compile_args=["-std=c++1z"],
            )
  ]

setup(name='polyfill',
      version='0.1',
      description='',
      url='http://github.com/mostlyuseful/plotting',
      author='Maurice-Pascal Sonnemann',
      author_email='mpsonnemann@gmail.com',
      license='MIT',
      ext_modules = extensions,
      py_modules= ["polyfill"],
      zip_safe=False)
