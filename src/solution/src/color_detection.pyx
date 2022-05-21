import numpy as np
import cython

cdef int hue_range = 8

cdef int s_min = (20*255)/360
cdef int v_min = (50*255)/360

@cython.boundscheck(False)
@cython.wraparound(False)
def in_range(const unsigned char [:, :, :] frame, int target_hue):
  cdef int height, width
  cdef unsigned char h, s, v

  cdef int HUE = int(target_hue * (180.0/360))
  cdef int HUE_NEG = HUE - 180
  
  height = frame.shape[0]
  width = frame.shape[1]

  mask = np.zeros((height, width), np.ubyte)

  cdef Py_ssize_t i, j
  for i in range(height):
    for j in range(width):
      h, s, v = frame[i][j][0], frame[i][j][1], frame[i][j][2]
      h_check = ((HUE - h) < hue_range) or ((h - HUE_NEG) < hue_range)
      s_check = s > s_min
      v_check = v > v_min
      if(h_check and s_check and v_check):
        mask[i][j] = 255
  
  return mask