import pygame as pg
import numpy as np

WIDTH, HEIGHT = 800, 600
FPS = 60
WIN=pg.display.set_mode((WIDTH, HEIGHT))
draw_mesh = False
ITERS_SELF_COLITION = 1
WALL_WIDTH=26
MIN_L=20 #min rest length

BLACK=(0,0,0)
WHITE=(255,255,255)
RED=(255,0,0)
GREEN=(0,255,0)
BLUE=(0,0,255)

DT=0.001
G=982
C=10
K=2500
RHO=.2