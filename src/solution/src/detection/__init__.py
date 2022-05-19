import numpy as np

h_max = int(13/360*180)
h_min = int(240/360*180)

s_min = int(20/360*355)
v_min = int(50/360*255)

def in_range(frame):
    print(frame.shape)
    height, width, _ = frame.shape
    mask = np.zeros((height, width), np.ubyte)
    for i, row in enumerate(frame):
      for j, cell in enumerate(row):
        h, s, v = cell[0], cell[1], cell[2]
        h_check = h < h_max or h > h_min
        s_check = s > s_min
        v_check = v > v_min
        mask[i][j] = 255 * (h_check * s_check * v_check)
    print('mask')
    return mask

