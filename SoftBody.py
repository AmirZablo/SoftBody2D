import numpy as np
import pygame as pg
from classes import *
from constants import *

pg.display.set_caption("Soft Body Simulation")

bodies=[]
blocks=[]

def out_of_window(body):
    if body.x<-150 or body.x>WIDTH+150 or body.y>HEIGHT+150:
        bodies.remove(body)

def is_overlapped(points, body_list):
    for p in points:
        for i in body_list:
            copy_border=i.border.copy()
            #remove repeated values, without altering the order
            border_ids=list(dict.fromkeys(copy_border))
            border_vetices=[(i.points[j].x,i.points[j].y) for j in border_ids]
            polygon=pl(border_vetices)
            point=pt(p)
            if polygon.contains(point):
                return True
    return False

def overlap_wall(points, body_list):
    for p in points:
        for i in body_list:
            polygon=pl(i.vertices)
            if polygon.contains(pt(p)):
                return True
    return False
            

def main():
    clock=pg.time.Clock()
    run=True

    top_left_wall=(0,0)
    top_right_wall=(0,0)
    right_mouse_pressed=False

    p1=(0,0)
    p2=(0,0)
    left_mouse_pressed=False

    while run:
        for event in pg.event.get():
            if event.type == pg.QUIT:
                run=False
            if event.type == pg.MOUSEBUTTONDOWN: # VALIDAR QUE NO SE ESTÉ CREANDO SOBRE OTRO OBJETO
                if event.button == 1:
                    left_mouse_pressed=True
                    p1=pg.mouse.get_pos()
                if event.button == 3:
                    top_left_wall=(event.pos[0],event.pos[1])
                    right_mouse_pressed=True
            if event.type==pg.MOUSEBUTTONUP:
                if event.button == 1:
                    p2=pg.mouse.get_pos()
                    dx=p2[0]-p1[0]
                    dy=p2[1]-p1[1]
                    if not is_overlapped([p1,(p1[0]+dx,p1[1]),p2,(p1[0],p1[1]+dy)],bodies) and not overlap_wall([p1,(p1[0]+dx,p1[1]),p2,(p1[0],p1[1]+dy)],blocks):
                        if dx>MIN_L and dy>MIN_L:
                            bodies.append(Rectangle(p1[0], p1[1], dx, dy, int(dx/MIN_L)+1, int(dy/MIN_L)+1, 10))
                        elif dx>MIN_L and dy<-MIN_L:
                            bodies.append(Rectangle(p1[0], p2[1], dx, np.abs(dy), int(dx/MIN_L)+1, int(np.abs(dy)/MIN_L), 10))
                        elif dx<-MIN_L and dy>MIN_L:
                            bodies.append(Rectangle(p2[0], p1[1], np.abs(dx), dy, int(np.abs(dx)/MIN_L), int(dy/MIN_L)+1, 10))
                        elif dx<-MIN_L and dy<-MIN_L:
                            bodies.append(Rectangle(p2[0], p2[1], np.abs(dx), np.abs(dy), int(np.abs(dx)/MIN_L), int(np.abs(dy)/MIN_L), 10))
                    left_mouse_pressed=False
                if event.button==3:
                    top_right_wall=(event.pos[0],event.pos[1])
                    if np.linalg.norm(np.array(top_left_wall)-np.array(top_right_wall))>5:
                        blocks.append(Block(top_left_wall,top_right_wall))
                    right_mouse_pressed=False




        WIN.fill(WHITE)
        for b in bodies:
            b.update(DT,blocks, bodies)
            out_of_window(b)
            b.draw(WIN)
        for b in blocks:
            b.draw(WIN)
       
        if right_mouse_pressed:
            pg.draw.line(WIN, BLACK, top_left_wall, pg.mouse.get_pos(), 1)
        if left_mouse_pressed:
            if np.abs(pg.mouse.get_pos()[0]-p1[0])<MIN_L or np.abs(pg.mouse.get_pos()[1]-p1[1])<MIN_L or is_overlapped([p1,(pg.mouse.get_pos()[0],p1[1]),pg.mouse.get_pos(),(p1[0],pg.mouse.get_pos()[1])],bodies) or overlap_wall([p1,(pg.mouse.get_pos()[0],p1[1]),pg.mouse.get_pos(),(p1[0],pg.mouse.get_pos()[1])],blocks):
                pg.draw.rect(WIN, RED, (p1[0],p1[1],pg.mouse.get_pos()[0]-p1[0],pg.mouse.get_pos()[1]-p1[1]), 1)
            else:
                pg.draw.rect(WIN, GREEN, (p1[0], p1[1], pg.mouse.get_pos()[0]-p1[0], pg.mouse.get_pos()[1]-p1[1]), 1)
        
        
        np.random.shuffle(bodies)

        pg.display.update()
        #clock.tick(FPS)

    pg.quit()

if __name__ == '__main__':
    main()