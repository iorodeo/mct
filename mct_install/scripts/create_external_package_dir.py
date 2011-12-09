#!/usr/bin/env python
from __future__ import print_function
import os
from install_tools import create_dir

def create_external_package_dir():
    package_dir = os.environ['MCT_EXTERNAL_PACKAGE_DIR']
    create_dir(package_dir)

# -----------------------------------------------------------------------------
if __name__ == '__main__':
    create_external_package_dir()




