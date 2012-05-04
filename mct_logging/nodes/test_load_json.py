from __future__ import print_function
import sys
import simplejson as json

filename = sys.argv[1]

with open(filename,'r') as f:
    data = json.load(f)






