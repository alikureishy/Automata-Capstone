#!/usr/bin/env python

import matplotlib.pyplot as plt
import numpy as np

MAX_DECEL = 0.5

def main():
    x = np.linspace(10, 0, 100)

    plt.plot(x, np.sqrt(2 * MAX_DECEL * x))

    plt.xlabel('distance')
    plt.ylabel('velocity')

    plt.xlim(10, 0)

    plt.grid(True)

    plt.show()

if __name__ == '__main__':
    main()