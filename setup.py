#!/usr/bin/env python

import os
import sys
from os import path, walk
from setuptools import setup, find_packages

sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'biconvex_mpc'))

def print_error(*args, **kwargs):
    """ Print in stderr. """
    print(*args, file=sys.stderr, **kwargs)

# Package name.
package_name = "biconvex_mpc"

# Long description from the readme.
with open("README.md", "r") as fh:
    long_description = fh.read()

# Install the package.xml.
data_files_to_install = [(path.join("share", package_name), ["package.xml"])]

# Install nodes and demos.
scripts_list = []
for (root, _, files) in walk(path.join("demos")):
    for demo_file in files:
        if demo_file[-2:] == "py":
            scripts_list.append(path.join(root, demo_file))

# Final setup.
setup(
    name=package_name,
    version="1.0.0",
    package_dir={package_name: package_name},
    packages=[package for package in find_packages()
                if package.startswith('py_biconvex_mpc')],
    data_files=data_files_to_install,
    scripts=scripts_list,
    install_requires=[
          'numpy>=1.10.4',
      ],
    zip_safe=True,
    maintainer="Paarth Shah",
    maintainer_email="paarth@oxfordrobotics.institute",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/pypa/sampleproject",
    description="This repo contains the centroidal motion planner based on a Biconvex solver",
    license="BSD-3-clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: BSD-3-clause",
        "Operating System :: OS Independent",
    ],
    python_requires=">=3.6",
)