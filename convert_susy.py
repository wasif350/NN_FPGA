import numpy as np, struct, os

data = np.loadtxt('susy.txt', delimiter=',', dtype=np.float32)
nd = data.shape[1]
np_pts = data.shape[0]

with open('susy.bin', 'wb') as f:
    f.write(struct.pack('ii', nd, np_pts))
    f.write(data.tobytes())

with open('susy_query.bin', 'wb') as f:
    f.write(struct.pack('ii', nd, 50000))
    f.write(data[:50000].tobytes())