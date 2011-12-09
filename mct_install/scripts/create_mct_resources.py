#!/usr/bin/env python
import os
from install_tools import create_dir

def create_mct_resources():
    """
    Checks for resources directory and creates it if is doesn't already exist.
    """
    mct_res_dir = os.environ['MCT_RESOURCES']
    create_dir(mct_res_dir)

# -----------------------------------------------------------------------------
if __name__ == '__main__':
    create_mct_resources()
