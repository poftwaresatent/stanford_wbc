#!/usr/bin/env python

import numpy

omega = numpy.array([1.0, 2.5])
phi0 = numpy.array([2.3, -0.7])
amp = numpy.array([1.5, 0.3])

for tt in xrange(2000):
    foo = 1e-3 * tt * omega + phi0
    for pp in amp * numpy.sin(foo):
        print pp,
    for vv in amp * numpy.cos(foo):
        print vv,
    print
