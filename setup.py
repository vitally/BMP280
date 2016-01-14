from ez_setup import use_setuptools
use_setuptools()
from setuptools import setup, find_packages

setup(name 				= 'BMP280',
	  version 			= '1.0.0',
	  author			= 'Vitally Tezhe',
	  author_email		= 'vitally.tezhe@gmail.com',
	  description		= 'Library for accessing pressure and temperature sensors on the BMP280 chip .',
	  license			= 'MIT',
	  url				= 'https://github.com/vitally/BMP280',
	  dependency_links	= ['https://github.com/adafruit/Adafruit_Python_GPIO/tarball/master#egg=Adafruit-GPIO-0.6.5'],
	  install_requires	= ['Adafruit-GPIO>=0.6.5'],
	  packages 			= find_packages())
