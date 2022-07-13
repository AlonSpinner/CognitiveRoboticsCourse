from setuptools import setup
from setuptools import find_packages

package_data = {
    '': [
        "./lpg-td",
        "./optic",
        "./optic-rewrite-no-lp",
	"./docker/*"
    ]
}

setup(
    name = "maildelivery",
    version = "1.0.0",
    description = "bla bla",
    author = "alon spinner",
    url = "https://github.com/AlonSpinner/CognitiveRoboticsCourse",
    packages = find_packages(),
    include_package_data=True,
    package_data = package_data,
    )
