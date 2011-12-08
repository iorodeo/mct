#!/usr/bin/env python
from __future__ import print_function
import os
import os.path
from create_mct_resources import create_mct_resources
from install_tools import create_dir

def create_python_virtualenv():
    """
    Create the python virtual environment for the multicamera tracking system
    if it doesn't already exist.
    """
    # Make sure that the correct directories exist - if not create them
    create_mct_resources()
    virtualenv_dir = os.environ['MCT_PYTHON_VIRTUALENV']
    pyenv_dir, mct_pyenv = os.path.split(virtualenv_dir)
    create_dir(pyenv_dir)

    # Look for python virtual env
    print('checking for python virtualenv {0} ... '.format(virtualenv_dir),end='')
    if os.path.isdir(virtualenv_dir):
        print('exists')
    else:
        print('creating')
        cmd = 'virtualenv {0}'.format(virtualenv_dir)
        os.system(cmd)

# -----------------------------------------------------------------------------
if __name__ == '__main__': 
    create_python_virtualenv()


