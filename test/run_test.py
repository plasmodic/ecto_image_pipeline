#!/usr/bin/env python

import sys
import subprocess
path_script = sys.argv[1]
script = sys.argv[2]
args = sys.argv[3:]
call =['.',path_script,'&&',script]+args
call = ' '.join(call)
subprocess.check_call(['sh','-c',call],stdout=sys.stdout,stderr=sys.stderr,stdin=sys.stdin)


