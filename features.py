import numpy as numpy
import math
from fractions import Fraction
from scipy.odr import *

class featuresDetection:
    def __init__(self):
        self.EPSILON = 10
        self.DELTA = 501
        self.SNUM = 6
        self.PMIN = 20
        self.GMAX = 20
        self.SEED_SEGMENTS = []
        self.LINE_SEGMENTS = []
        self.LASERPOINTS = []
        self.LINE_PARAMS = None
        self.NP = len(self.LASERPOINTS) - 1
        self.LMIN = 20
        self.LR = 0
        self.PR = 0
        
def dist_P2P(self, p1, p2):
    px = (p1[0] - p2[0])**2
    py = (p1[1] - p2[1])**2
    return math.sqrt(px+py)

def dist_P2line(self, params, pt):
    A, B, C = params
    distance = abs(A * pt[0] + B * pt[1] + C / math.sqrt(A ** 2 + B ** 2))
    return distance

def line_2points(self, m, b):
    x = 5
    y = m * x + b
    x2 = 2000
    y2 = m* x2 + b
    return [(x,y), (x2,y2)]

def lineForm_G2SI(self, A, B, C):
    m = -A/B
    B = -C/B
    return m, B

def lineForm_Si2G(self, m, B):
    A, B, C = -m, 1, -B
    if A < 0:
        A, B, C = -A, -B, -C
    den_a = Fraction(A).limit_denominator(1000).as_integer_ratio()[1]
    den_c = Fraction(C).limit_denominator(1000).as_integer_ratio()[1]
    
    gcd = np.gcd(den_a, den_c)
    