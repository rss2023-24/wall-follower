import csv
  
x = []
y = []

csv_filename = '../data/csv/left_slow.csv'
num = 0
with open(csv_filename,'r') as csvfile:
    lines = csv.reader(csvfile, delimiter=',')
    # print(type(lines))
    for ind, row in enumerate(lines):
        # print(row)
        if not ind:
            continue
        x.append(float(row[0]))
        y.append(float(row[1]))
        num += 1

alpha = 0.1
y_val = [abs(y_n-0.5) for y_n in y]
loss = (1/num)*sum(y_val)
score = 1/(1+alpha*loss)
print("this is your score")
print(score)
