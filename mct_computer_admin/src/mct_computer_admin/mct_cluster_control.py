#!/usr/bin/env python
import roslib
roslib.load_manifest('mct_computer_admin')
import sys
from mct_computer_admin import cluster_control_fab
cluster_control_fab.main(sys.argv)

