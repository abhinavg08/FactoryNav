from setuptools import find_packages, setup

setup(name='factorynav_perception', version='0.1.0', packages=find_packages(),
      data_files=[('share/ament_index/resource_index/packages', ['resource/factorynav_perception']),
                  ('share/factorynav_perception', ['package.xml'])],
      install_requires=['setuptools'], license='MIT',
      entry_points={'console_scripts': ['yolo_detector = factorynav_perception.yolo_detector:main']})
