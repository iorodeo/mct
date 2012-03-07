#!/usr/bin/env python
from __future__ import print_function
import os
import os.path
import shutil
from subprocess import call
from create_mct_resources import create_mct_resources
from create_python_virtualenv import create_python_virtualenv
from create_external_package_dir import create_external_package_dir
from check_for_ubuntu_pkg import check_for_ubuntu_pkg

# Url for source code - may want to move this to config file
pkg_name = 'cvblob'
cvblob_version = '0.10'
cvblob_release = '3'

def install_cvblob():
    """
    Installs the cvblob library from source
    """
    ext_pkg_dir = os.environ['MCT_EXTERNAL_PACKAGE_DIR']
    tarball = 'cvblob-{0}.{1}-src.tgz'.format(cvblob_version,cvblob_release)
    src_url = 'http://cvblob.googlecode.com/files/{0}'.format(tarball)
    tarball_dir = os.path.join(ext_pkg_dir,tarball)
    src_dir = os.path.join(ext_pkg_dir,'cvblob')
    build_dir = os.path.join(src_dir,'build')

    # Make sure that virtual environment exists
    create_python_virtualenv()
    create_external_package_dir()

    # Check to see if cvblob is installed - if so we are done.
    print('checking for cvblob ... ',end='')
    if check_for_ubuntu_pkg('cvblob'):
        print('installed')
        return;

    else:
        print('not found')

        # Remove any old tar files
        if os.path.isfile(tarball_dir):
            print('removing old tar file: {0}'.format(tarball_dir))
            os.remove(tarball_dir)

        # Reomve old source directory 
        if os.path.isdir(src_dir):
            print('removing old source directory: {0}'.format(src_dir))
            shutil.rmtree(src_dir)

        # Download the library using wget
        print('downloading')
        call('wget -P {0} {1}'.format(ext_pkg_dir, src_url),shell=True)

        print('unpacking')
        cmd = 'tar -C {0} -xvzf {1}'.format(ext_pkg_dir, tarball_dir)
        call('tar -C {0} -xvzf {1}'.format(ext_pkg_dir, tarball_dir),shell=True)

        print('removing tarball {0}'.format(tarball_dir))
        os.remove(tarball_dir)

        if os.path.isdir(build_dir):
            print('removing old build directory')
            shutil.rmtree(build_dir)

        print('creating new build directory {0}'.format(build_dir))
        os.mkdir(build_dir)

        print('running cmake')
        cmake_cmd = 'cd {0}; cmake --clean-first {1}'.format(build_dir,src_dir)
        print(cmake_cmd)
        call(cmake_cmd,shell=True)

        print('running make')
        make_cmd = 'cd {0}; make'.format(build_dir)
        print(make_cmd)
        call(make_cmd,shell=True)

        print('installing with checkinstall')
        cmd_args = (build_dir,pkg_name,cvblob_version,cvblob_release)
        install_cmd  = 'cd {0};sudo checkinstall --pkgname={1} --pkgversion={2} --pkgrelease={3} -y'.format(*cmd_args)
        print(install_cmd)
        call(install_cmd,shell=True)

# --------------------------------------------------------------------------------------------
if __name__ == '__main__':
    install_cvblob()
