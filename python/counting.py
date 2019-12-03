﻿import random
import numpy as np

import matplotlib.pyplot as plt




i = 0
rows = [0] * 6
rows[0] = [0] * 10
rows[1] = [0] * 10
rows[2] = [0] * 10
rows[3] = [0] * 10
rows[4] = [0] * 10
rows[5] = [0] * 10

file = open('raw_numbers', 'r')
count = len(file.readlines())
print(count)
file.close()

file = open('raw_numbers', 'r')
old_number = 0
next_number = 0
row_size = 1

while i < count:
    i = i+1
    old_number = next_number
    next_number = int(file.readline())
    if next_number is old_number:
        row_size += 1
    elif row_size > 1:
        rows[int(old_number)-1][row_size-1] +=1
        print('reihe gefunden:' ,row_size,'x', old_number)
        row_size = 1
    else:
        row_size = 1
file.close()


if next_number is old_number:
    print('reihe gefunden:', row_size, 'x', old_number)

print('gesamt:')
for zahl in range(6):
    for reihe in range(1,10):
        print(reihe+1, 'x', zahl+1, ':', rows[zahl][reihe])

numbers = [0]*6
numbers[0] = [0] * count
numbers[1] = [0] * count
numbers[2] = [0] * count
numbers[3] = [0] * count
numbers[4] = [0] * count
numbers[5] = [0] * count

size = 0



for line in open('raw_numbers', 'r'):
    if int(line) == 1:
        numbers[0][size] = numbers[1][size-1] + 1
        numbers[1][size] = numbers[1][size-1]
        numbers[2][size] = numbers[2][size-1]
        numbers[3][size] = numbers[3][size-1]
        numbers[4][size] = numbers[4][size-1]
        numbers[5][size] = numbers[5][size-1]
    if int(line) == 2:
        numbers[0][size] = numbers[0][size-1]
        numbers[1][size] = numbers[1][size-1] + 1
        numbers[2][size] = numbers[2][size-1]
        numbers[3][size] = numbers[3][size-1]
        numbers[4][size] = numbers[4][size-1]
        numbers[5][size] = numbers[5][size-1]
    if int(line) == 3:
        numbers[0][size] = numbers[0][size-1]
        numbers[1][size] = numbers[1][size-1]
        numbers[2][size] = numbers[2][size-1] + 1
        numbers[3][size] = numbers[3][size-1]
        numbers[4][size] = numbers[4][size-1]
        numbers[5][size] = numbers[5][size-1]
    if int(line) == 4:
        numbers[0][size] = numbers[0][size-1]
        numbers[1][size] = numbers[1][size-1]
        numbers[2][size] = numbers[2][size-1]
        numbers[3][size] = numbers[3][size-1] + 1
        numbers[4][size] = numbers[4][size-1]
        numbers[5][size] = numbers[5][size-1]
    if int(line) == 5:
        numbers[0][size] = numbers[0][size-1]
        numbers[1][size] = numbers[1][size-1]
        numbers[2][size] = numbers[2][size-1]
        numbers[3][size] = numbers[3][size-1]
        numbers[4][size] = numbers[4][size-1] + 1
        numbers[5][size] = numbers[5][size-1]
    if int(line) == 6:
        numbers[0][size] = numbers[0][size-1]
        numbers[1][size] = numbers[1][size-1]
        numbers[2][size] = numbers[2][size-1]
        numbers[3][size] = numbers[3][size-1]
        numbers[4][size] = numbers[4][size-1]
        numbers[5][size] = numbers[5][size-1] + 1
    size += 1

file.close()

print(numbers)


X = np.arange(0,size)

plt.plot(X, numbers[0])
plt.plot(X, numbers[1])
plt.plot(X, numbers[2])
plt.plot(X, numbers[3])
plt.plot(X, numbers[4])
plt.plot(X, numbers[5])
plt.show()


