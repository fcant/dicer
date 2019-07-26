import numpy as np

for i in range(2931,3200):
    print(str(i) + ' ' + str(np.sin((i-2900)*0.00001)))
    