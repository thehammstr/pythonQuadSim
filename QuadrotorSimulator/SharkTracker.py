import csv

filename = 'Data_steady_Locomotion.csv'

with open(filename, 'rb') as csvfile:
     reader = csv.reader(csvfile, delimiter=' ', quotechar='|')
     for row in spamreader:
         print ', '.join(row)

