#!/usr/bin/env python
from __future__ import print_function
import os
import os.path

script_dict = {'mct': 'mct_computer_admin/scripts/mct_cluster_control'}
bin_dir = os.path.join(os.environ['HOME'],'bin')

print('installing scripts')
for name, script in script_dict.iteritems():
    src = os.path.join(os.environ['MCT_NAME'],script)
    dst = os.path.join(bin_dir,name)
    print('  {0} -> {1}'.format(src,dst))
    os.symlink(src,dst)

