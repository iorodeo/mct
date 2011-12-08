from __future__ import print_function
import os
import os.path

def create_dir(dir_path):
    """
    Creates a directory if it doesn't already exist.
    """
    print('checking for {0} directory ... '.format(dir_path),end='')
    if os.path.isdir(dir_path):
        print('exists')
    else:
        print('creating')
        os.mkdir(dir_path)
