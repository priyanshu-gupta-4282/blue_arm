import numpy as np

a1=90
a2=139
a4=139
a5=140

x=int(input("Enter an integer: "))
y=int(input("Enter an integer: "))
z=int(input("Enter an integer: "))

theta1=np.arctan2(y,x)

planer_dist=np.sqrt(x**2+y**2)
print(planer_dist)

d=np.sqrt(planer_dist**2+z**2)
print(d)

phi1=np.atan2(z,planer_dist)
phi2=np.arccos((a2*a2 + d*d - a4*a4) / (2 * a2 * d))

theta2=phi1+phi2
print(phi1,phi2)

phi3=np.arccos((a2**2 + a4**2 - d**2) / (2 * a2 * a4))
theta3 = (3.14 - (phi3)) 
print(np.degrees(theta1),np.degrees(theta2), np.degrees(theta3))




