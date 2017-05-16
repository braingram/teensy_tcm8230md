#!/usr/bin/env python

import glob
import sys

import numpy
import pygame
import serial


port = None
if len(sys.argv) > 1:
    port = sys.argv[1]
else:
    ports = glob.glob('/dev/ttyACM*')
    if len(ports) == 1:
        port = ports[0]


p = serial.Serial(port, 9600)
fw = 128
fh = 96
fb = fw * fh * 2


def read_frame():
    b = ""
    f = None
    started = False
    # read 1 frame
    while f is None:
        if p.inWaiting():
            b += p.read(p.inWaiting())
            if '====' in b:
                if not started:
                    b = b[b.index('====') + 4:]
                    started = True
                else:
                    f = b[:b.index('====')]
                    if len(f) == fb:
                        break
                    f = None
                    b = b[b.index('====') + 4:]
    print("Read frame with %s bytes" % len(f))
    print("Expected %s" % fb)
    return f

a = read_frame()

pygame.init()
screen = pygame.display.set_mode(
    (fw, fh), pygame.HWSURFACE | pygame.DOUBLEBUF)

quit = False
while not quit:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            quit = True
            continue
    f = read_frame()
    #a = numpy.fromstring(f, 'u1', (fw * fh)).reshape((fh, fw))
    a = numpy.fromstring(f, 'u2', (fw * fh)).reshape((fh, fw))
    a = a.swapaxes(0, 1)
    # RGB
    b = numpy.dstack(
        (
            (a >> 11).astype('u1') * 8,
            #numpy.zeros_like(a).astype('u1'),
            ((a & 0x7E0) >> 5).astype('u1') * 4,
            #numpy.zeros_like(a).astype('u1'),
            (a & 0x1f).astype('u1') * 8,
            #numpy.zeros_like(a).astype('u1'),
        )).astype('u1')
    #m = numpy.mean(b, axis=2).astype('u1')
    #b = numpy.dstack((m, m, m))
    #b = numpy.dstack((a, a, a))
    print(b.shape, b.min(), b.max())
    pygame.surfarray.blit_array(screen, b)
    pygame.display.flip()

sys.exit()
