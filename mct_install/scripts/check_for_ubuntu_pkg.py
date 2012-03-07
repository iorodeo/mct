from __future__ import print_function
import os
import subprocess

def check_for_ubuntu_pkg(pkg_name):
    devnull = open(os.devnull,'w')
    rval = subprocess.call(['dpkg','-s',pkg_name],stdout=devnull,stderr=subprocess.STDOUT)
    devnull.close()
    if rval !=0:
        return False
    else:
        return True

# -----------------------------------------------------------------------------------------
if __name__ == '__main__':

    import sys
    pkg_name = sys.argv[1]
    test = check_for_ubuntu_pkg(pkg_name)
    print('{0} installed = {1}'.format(pkg_name,test))


