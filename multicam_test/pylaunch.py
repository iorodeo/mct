import subprocess
import signal
import time

print 'launching multicam'

proc = subprocess.Popen(['roslaunch', 'multicam.launch'])
print 'done'

print 'sleeping'
for i in range(0,10):
    print('sleeping: %d'%(i,))
    time.sleep(0.5)

print 'sending signal'
proc.send_signal(signal.SIGINT)
print 'done'
