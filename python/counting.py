import random
import numpy as np
import csv

import matplotlib.pyplot as plt

name = 'seite2' # Dateiname, "raw" wird automatisch davor gesetzt
datei_name = 'raw_'+name

print(datei_name)

i = 0
rows = [0] * 6
rows[0] = [0] * 10
rows[1] = [0] * 10
rows[2] = [0] * 10
rows[3] = [0] * 10
rows[4] = [0] * 10
rows[5] = [0] * 10

file = open(datei_name, 'r')
count = len(file.readlines())
print(count)
file.close()

file = open(datei_name, 'r')
old_number = 0
next_number = 0
row_size = 1


# Anzahl an aufeinanderfolgenden gleichen Zahlen wird erfasst. Aktuell werden nicht die Korrekten werte Ausgegeben, da in den Raw-Dateien die Messfehler nicht erfasst werden. 
# So wird fälschlicherweise eine Reihe erkannt, obwohl dazwischen Messfehler gewesen sein könnten. (geht bis Zeile 120)
while i < count:
    i = i+1
    old_number = next_number
    next_number = int(file.readline())
    if next_number is old_number:
        row_size += 1
    elif row_size > 1:
        rows[int(old_number)-1][row_size-1] +=1
        #print('reihe gefunden:' ,row_size,'x', old_number)
        row_size = 1
    else:
        row_size = 1
file.close()



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

all_numbers = [0]*6

for line in open(datei_name, 'r'):
    if int(line) == 1:
        all_numbers[0] += 1
        numbers[0][size] = numbers[0][size-1] + 1
        numbers[1][size] = numbers[1][size-1]
        numbers[2][size] = numbers[2][size-1]
        numbers[3][size] = numbers[3][size-1]
        numbers[4][size] = numbers[4][size-1]
        numbers[5][size] = numbers[5][size-1]
    if int(line) == 2:
        all_numbers[1] += 1
        numbers[0][size] = numbers[0][size-1]
        numbers[1][size] = numbers[1][size-1] + 1
        numbers[2][size] = numbers[2][size-1]
        numbers[3][size] = numbers[3][size-1]
        numbers[4][size] = numbers[4][size-1]
        numbers[5][size] = numbers[5][size-1]
    if int(line) == 3:
        all_numbers[2] += 1
        numbers[0][size] = numbers[0][size-1]
        numbers[1][size] = numbers[1][size-1]
        numbers[2][size] = numbers[2][size-1] + 1
        numbers[3][size] = numbers[3][size-1]
        numbers[4][size] = numbers[4][size-1]
        numbers[5][size] = numbers[5][size-1]
    if int(line) == 4:
        all_numbers[3] += 1
        numbers[0][size] = numbers[0][size-1]
        numbers[1][size] = numbers[1][size-1]
        numbers[2][size] = numbers[2][size-1]
        numbers[3][size] = numbers[3][size-1] + 1
        numbers[4][size] = numbers[4][size-1]
        numbers[5][size] = numbers[5][size-1]
    if int(line) == 5:
        all_numbers[4] += 1
        numbers[0][size] = numbers[0][size-1]
        numbers[1][size] = numbers[1][size-1]
        numbers[2][size] = numbers[2][size-1]
        numbers[3][size] = numbers[3][size-1]
        numbers[4][size] = numbers[4][size-1] + 1
        numbers[5][size] = numbers[5][size-1]
    if int(line) == 6:
        all_numbers[5] += 1
        numbers[0][size] = numbers[0][size-1]
        numbers[1][size] = numbers[1][size-1]
        numbers[2][size] = numbers[2][size-1]
        numbers[3][size] = numbers[3][size-1]
        numbers[4][size] = numbers[4][size-1]
        numbers[5][size] = numbers[5][size-1] + 1
    size += 1

file.close()

# Ende der Reihenerfassung



print(all_numbers)

print('Mittelwert:')
print((all_numbers[0]+2*all_numbers[1]+3*all_numbers[2]+4*all_numbers[3]+5*all_numbers[4]+6*all_numbers[5])/size)

X = np.arange(0,size)

plt.plot(X, numbers[0], label='1')
plt.plot(X, numbers[1], label='2')
plt.plot(X, numbers[2], label='3')
plt.plot(X, numbers[3], label='4')
plt.plot(X, numbers[4], label='5')
plt.plot(X, numbers[5], label='6')
plt.legend(loc='upper left', frameon=False)
#plt.savefig('holz1.png')
plt.show()



with open(name + '.csv', mode='w', newline='') as csv_file:
    fieldnames = ['plot1', 'plot2', 'plot3', 'plot4', 'plot5', 'plot6']
    writer = csv.DictWriter(csv_file, fieldnames=fieldnames)

    writer.writeheader()
    writer.writerow({'plot1': 0, 'plot2': 0, 'plot3': 0, 'plot4': 0, 'plot5': 0,'plot6': 0})
    for i in range(0,size): #size
        writer.writerow({'plot1': numbers[0][i], 'plot2': numbers[1][i], 'plot3': numbers[2][i], 'plot4': numbers[3][i], 'plot5': numbers[4][i], 'plot6': numbers[5][i]})
    writer.writerow({'plot1': 0, 'plot2': 0, 'plot3': 0, 'plot4': 0, 'plot5': 0,'plot6': 0})
