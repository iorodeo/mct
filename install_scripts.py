#!/usr/bin/env python
from __future__ import print_function
import os
import os.path

script_list = ['mct_computer_admin/scripts/mct_cluster_control']

bin_dir = os.path.join(os.environ['HOME'],'bin')
for script in script_list:
    src = os.path.join(os.environ['MCT_NAME'],script)
    dummy, script_name = os.path.split(src)
    print('installing {0}'.format(script_name)
    dst = os.path.join(bin_dir, script_name)
    os.symlink(src,dst)

