from setuptools import setup
from setuptools import find_packages

setup(
    name = "maildelivery",
    version = "1.0.0",
    description = "bla bla",
    author = "bla bla",
    url = "https://github.com/AlonSpinner/maildelivery",
    packages = find_packages(exclude = ('tests*')),
    )
