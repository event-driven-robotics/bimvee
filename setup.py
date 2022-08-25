# Always prefer setuptools over distutils
from setuptools import setup, find_packages
# To use a consistent encoding
from os import path

here = path.abspath(path.dirname(__file__))

# Get the long description from the README file
with open(path.join(here, 'README.md'), encoding='utf-8') as f:
    long_description = f.read()
    
setup(
  name = 'bimvee',
  packages=['bimvee', 'bimvee.importRosbag', 'bimvee.importRosbag.importRosbag', 'bimvee.importRosbag.importRosbag.messageTypes', 'bimvee.visualisers'],
  version = '1.0.17',
  license='gpl',
  description = 'Batch Import, Manipulation, Visualisation and Export of Events etc',
  long_description=long_description,
  long_description_content_type='text/markdown',
  author = 'Event-driven Perception for Robotics group at Istituto Italiano di Tecnologia: Simeon Bamford, Suman Ghosh, Aiko Dinale, Massimiliano Iacono, Ander Arriandiaga, etc',
  author_email = 'simbamford@gmail.com',
  url = 'https://github.com/event-driven-robotics/bimvee',
  download_url = 'https://github.com/event-driven-robotics/bimvee_pkg/archive/v1.0.tar.gz',
  keywords = ['event', 'event camera', 'event-based', 'event-driven', 'spike', 'dvs', 'dynamic vision sensor', 'neuromorphic', 'aer', 'address-event representation' 'spiking neural network', 'davis', 'atis', 'celex' ],
  install_requires=[
          'numpy',
          'tqdm',
          'setuptools',
          'matplotlib',
          'seaborn',
          'imageio',
          'hickle',
          'opencv-python'
      ],
  classifiers=[
    'Development Status :: 3 - Alpha',
    'Intended Audience :: Developers',
    'Topic :: Software Development :: Build Tools',
    'License :: OSI Approved :: GNU General Public License (GPL)',
    'Programming Language :: Python :: 3',
    'Programming Language :: Python :: 3.7',
  ],
)
