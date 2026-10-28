from setuptools import find_packages, setup

setup(name='factorynav_dynamic',version='0.2.0',packages=find_packages(),
      data_files=[('share/ament_index/resource_index/packages',['resource/factorynav_dynamic']),
                  ('share/factorynav_dynamic',['package.xml'])],
      install_requires=['setuptools'],license='MIT',
      maintainer='Abhinav Reddy', maintainer_email='abhinavreddyg08@gmail.com',
      entry_points={'console_scripts':['traffic = factorynav_dynamic.traffic:main',
                                       'forecast = factorynav_dynamic.forecast:main',
                                       'benchmark = factorynav_dynamic.benchmark:main']})
