import numpy as np

global_steptime = 0.00003

steptime = global_steptime

anlauf = 50

for i in np.arange(0,anlauf,1):
    print(str(i/anlauf) + ' ' + str((np.cos(i/anlauf)+0.5)*steptime))
    