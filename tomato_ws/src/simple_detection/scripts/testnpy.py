import numpy as np
import glob

while (True):
    if(len(glob.glob('./RS_SETTINGdist*.npy')) == 2):
        dist_min = np.load("RS_SETTINGdist[cm]min.npy")
        dist_max = np.load("RS_SETTINGdist[cm]max.npy")

    print("dist_min: ",end="")
    print(dist_min)
    print("dist_max: ",end="")
    print(dist_max)

    print("\n")