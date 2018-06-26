import matplotlib.pyplot as plt

X1,X2,X3,X4,X5,X6,X7,X8,X9,X10 = [], [], [], [], [], [], [], [], [], []
for line in open('PPC_data.dat', 'r'):
  values = [float(s) for s in line.split()]
  X1.append(values[0])
  X2.append(values[1])
  X3.append(values[2])
  X4.append(values[3])
  X5.append(values[4])
  X6.append(values[5])
  X7.append(values[6])
  X8.append(values[7])
  X9.append(values[8])
  X10.append(values[9])

plt.figure(1)
plt.plot(X1, X2,linewidth=1)
plt.plot(X1, X3,linewidth=1)
#plt.xlim(0,5)




plt.figure(2)
plt.plot(X1, X4,linewidth=1)
plt.plot(X1, X10,linewidth=1)
#plt.xlim(0,5)



plt.figure(3)
#plt.plot(X1, X5)
plt.plot(X1, X6,'g+')
plt.plot(X1, X7,'r+')
#plt.xlim(0,5)

plt.show()
