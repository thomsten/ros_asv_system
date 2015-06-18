from asv_state_estimator import StateEstimator
import numpy as np

if __name__ == "__main__":
    se = StateEstimator(59.88764*np.pi/180., 10.56593*np.pi/180.)
    coords = [[59.88123, 10.58068],
              [59.88708, 10.57219],
              [59.88385, 10.57834],
              [59.88103, 10.56656],
              [59.88094, 10.55706],
              [59.88600, 10.57295]]

    for c in coords:
        c2 =  se.geod2enu(c[0]*np.pi/180, c[1]*np.pi/180, 0.0)
        print "%.2f\t %.2f"%(c2[0], c2[1])

