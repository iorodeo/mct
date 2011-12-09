#!/usr/bin/env python
from __future__ import print_function
import os
import os.path
from subprocess import call
from create_mct_resources import create_mct_resources
from create_python_virtualenv import create_python_virtualenv
from create_external_package_dir import create_external_package_dir

def install_jinja2():
    # Make sure that virtual environment exists
    create_python_virtualenv()

    # Use pip to get jinja2 from within virtualenv
    python_virtualenv = os.environ['MCT_PYTHON_VIRTUALENV']
    install_cmds = []
    install_cmds.append('source {0}/bin/activate'.format(python_virtualenv))
    install_cmds.append('pip install jinja2')
    install_cmds = '; '.join(install_cmds)
    call(install_cmds, shell=True, executable='/bin/bash')

# -----------------------------------------------------------------------------
if __name__ == '__main__':
    install_jinja2()
