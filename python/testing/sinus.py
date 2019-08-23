import numpy as np

for i in np.arange(2940,3200,1):
    print(str(i-2900) + ' ' + str(np.sin(((i-2900)/50)*0.0003)))
    