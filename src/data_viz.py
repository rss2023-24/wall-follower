import matplotlib.pyplot as plt
import csv
  
x = []
y = []

csv_filename = '../data/csv/data.csv'
with open(csv_filename,'r') as csvfile:
    lines = csv.reader(csvfile, delimiter=',')
    # print(type(lines))
    for ind, row in enumerate(lines):
        # print(row)
        if not ind:
            continue
        x.append(float(row[0]))
        y.append(float(row[1]))
  
plt.plot(x, y, color = 'orange', linestyle = 'solid',
         marker = 'o')
  
plt.xticks(rotation = 25)
# plt.xlabel('Dates')
# plt.ylabel('Temperature(°C)')
# plt.title('Weather Report', fontsize = 20)

ax = plt.gca()
ax.set_xlim([None, 1678328678401520289])
ax.set_ylim([0.40, 0.60])
# ax.set_xlim([None, None])
# ax.set_ylim([None, None])

plt.grid()
# plt.legend()
plt.show()