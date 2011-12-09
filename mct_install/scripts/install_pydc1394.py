#!/usr/bin/env python
from __future__ import print_function
import os
import os.path
from subprocess import call
from create_mct_resources import create_mct_resources
from create_python_virtualenv import create_python_virtualenv
from create_external_package_dir import create_external_package_dir

def install_pydc1394():
    """
    Installs the pyDC1394 package
    """
    # Make sure that virtual environment exists
    create_python_virtualenv()
    create_external_package_dir()

    # Get pydc1394 package
    print('checking for package pydc1394 ...',end='')
    pydc1394_dir = os.path.join(os.environ['MCT_EXTERNAL_PACKAGE_DIR'], 'pydc1394')
    if os.path.isdir(pydc1394_dir):
        print('exists')
    else:
        print('downloading')
        call('bzr branch lp:pydc1394 {0}'.format(pydc1394_dir), shell=True)
        python_virtualenv = os.environ['MCT_PYTHON_VIRTUALENV']
        print('installing pydc1394 into virtualenv {0}'.format(python_virtualenv))
        install_cmds = []
        install_cmds.append('source {0}/bin/activate'.format(python_virtualenv))
        install_cmds.append('cd {0}'.format(pydc1394_dir))
        install_cmds.append('python setup.py install')
        install_cmds = '; '.join(install_cmds)
        call(install_cmds, shell=True, executable='/bin/bash')

# -----------------------------------------------------------------------------
if __name__ == '__main__':
    install_pydc1394()

