import csv
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point

x = []
y = []
pontos = []
pontos_tmp = []
x_offset = 50
y_offset = 50
resolution = 5

with open('data_tst.txt', 'rb') as file1:
    reader1 = csv.reader(file1, delimiter=',')
    #reader1 = list(reader1)
    #reader1 = [list(x) for x in zip(*reader1)]

    itercars = iter(reader1)
    next(itercars)

    for row in itercars:
    	#pontos_tmp.append([row[5], row[6]])
    	pontos_tmp.append([(float(row[5])-x_offset)/5, (float(row[6])-y_offset)/5])
    	#x = float(row[5])-x_offset
    	#y = float(row[6])-y_offset
    	#pontos_tmp.append([x, y])

with open('data_out.csv', 'w') as writeFile:
    writer = csv.writer(writeFile)
    writer.writerows(pontos_tmp)
    
#print(pontos_tmp)