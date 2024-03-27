#!/usr/bin/env python3

NR_ITERATIONS = 10

def ebf(M, d):
    x = M**(1/d)
    for i in range(NR_ITERATIONS):
        f = (x**(d+1)-1)/(x-1) - (M+1)
        df = ((d*x-d-1)*x**d + 1) / ((x-1)*(x-1))
        x -= f/df
    return x


print(ebf(5200000, 5))


