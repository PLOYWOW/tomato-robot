from scipy.interpolate import InterpolatedUnivariateSpline
import numpy as np
import matplotlib.pyplot as plt

# z = np.array([0.11,
#             0.12,
#             0.13,
#             0.14,
#             0.15,
#             0.16,
#             0.17,
#             0.18,
#             0.19,
#             0.20,
#             0.21,
#             0.22,
#             0.23,
#             0.24,
#             0.25,
#             0.26,
#             0.27,
#             0.28,
#             0.29,
#             0.30])

################ Output
### Sand tomato
z = np.array([0.11,0.12,0.13,0.14,0.15,0.16,0.17,0.18,0.19,0.20,0.21,0.22,0.23,0.24,0.25,0.26,0.27,0.28,0.29,0.30]) 
### Real tomato
# z = np.array([0.145,0.15,0.155,0.16,0.165,0.17,0.175,0.18,0.185,0.19,0.195,0.2,0.205,0.21,0.215,0.22,0.225,0.23,0.235,0.24,0.245,0.25,0.255,0.26,0.265,0.27,0.275,0.28,0.285,0.29,0.295,0.3]) 

z = np.sort(z)
z = z[::-1]
print("Z == >> ",z)

################ Input
### Sand tomato
# area (px2)
area = np.array([424032,336736,315525,280000,262752,229270,202776,186126,160763,149328,130419,127088,116675,110664,98936,91126,81928,75301,71250,67344]) #Sand tomato

### Real tomato Incub Room
#area (px2)
# area = np.array([250228,241808,227304,213796,199688,185680,182624,168420,155904,149669,141620,132022,127716,121720,111150,103290,98838,96657,93572,87535,85768,83142,80592,76396,73656,70470,67584,65511,61976,61008,58280,55912]) #Real tomato
#height (px)
# area = np.array([484,476,462,452,436,422,416,401,384,377,365,353,348,340,325,313,306,303,298,287,284,279,276,269,264,261,256,251,244,246,235,232])
#width (px)
# area = np.array([517,508,492,473,458,440,439,420,406,397,388,374,367,358,342,330,323,319,314,305,302,298,292,284,279,270,264,261,254,249,248,241])

area = np.sort(area)
print(type(area))
print("AREA == >> ",area)

plt.subplot(1, 4, 1)
plt.plot(area,z,"*",ms=5)

print(z.shape)
print(area.shape)
# f_interp = InterpolatedUnivariateSpline(area,z,k=5,check_finite=False)

spl = InterpolatedUnivariateSpline(area, z, k=1, check_finite=False)

####### Within range
plt.subplot(1, 4, 2)
plt.plot(area, z, 'ro', ms=5)
xs = np.linspace(68000, 420000, 1000) 
print("XS ==>> ",xs)
plt.subplot(1, 4, 3)
plt.plot(xs, spl(xs), 'go', lw=3, alpha=0.7)
plt.title("INPUT WITHIN RANGE")
#print("sql XS == >> ",spl(xs))


####### Out of range
xs = np.linspace(0, 900000, 900000) 
plt.subplot(1, 4, 4)
plt.plot(xs, spl(xs), 'bo', lw=3, alpha=0.7)
plt.title("INPUT OUT OF RANGE")
plt.show()

test_z = spl(900000)
print("Test_z = ",end="")
print(test_z)