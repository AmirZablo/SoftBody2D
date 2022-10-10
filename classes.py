import numpy as np 
import pygame as pg
from constants import *
from shapely.geometry import Point as pt
from shapely.geometry.polygon import Polygon as pl

class Point:
    """
    Point class
    x: x position
    y: y position
    mass: mass of the point
    """
    def __init__(self, x, y, mass):
        self.x = x
        self.y = y
        self.mass = mass
        self.vx = 0
        self.vy = 0
        self.fx = 0
        self.fy = 0

    def gravity(self, g):
        """
        Get the gravitational force acting on the point
        """
        self.fy += self.mass * g

    def collision_floor(self,blocks):
        """
        Check if the point is colliding with any block
        """
        point=pt(self.x,self.y)
        for block in blocks:
            polygon=pl(block.vertices)
            if polygon.contains(point):
                nearest_point=polygon.exterior.interpolate(polygon.exterior.project(point))
                nearest_point=(nearest_point.x,nearest_point.y)
                if np.linalg.norm((nearest_point[0]-self.x,nearest_point[1]-self.y)) >0:
                    normal_vector=(nearest_point[0]-self.x,nearest_point[1]-self.y)/np.linalg.norm((nearest_point[0]-self.x,nearest_point[1]-self.y))
                    mod=np.sqrt(self.vx**2+self.vy**2)
                    self.vx=mod*normal_vector[0]
                    self.vy=mod*normal_vector[1]
                    self.x=nearest_point[0]
                    self.y=nearest_point[1]

    def colission_bodies(self, bodies):
        """
        Check if the point is colliding with any body
        """
        point=pt(self.x,self.y)
        for b in bodies:
            if self not in b.points:
                copy_border=b.border.copy()
                border_ids=list(dict.fromkeys(copy_border))
                border_vetices=[(b.points[i].x,b.points[i].y) for i in border_ids]
                polygon=pl(border_vetices)
                if polygon.contains(point):
                    nearest_point=polygon.exterior.interpolate(polygon.exterior.project(point))
                    nearest_point=(nearest_point.x,nearest_point.y)
                    normal_vector=(nearest_point[0]-self.x,nearest_point[1]-self.y)/np.linalg.norm((nearest_point[0]-self.x,nearest_point[1]-self.y))
                    mod=np.sqrt(self.vx**2+self.vy**2)
                    self.vx=mod*normal_vector[0]
                    self.vy=mod*normal_vector[1]
                    self.x=nearest_point[0]
                    self.y=nearest_point[1]

    def self_colission(self,points):
        """
        Check if the point is colliding with another point
        """
        for i in range(ITERS_SELF_COLITION):
            for point in points:
                if point!=self:
                    if np.sqrt((self.x-point.x)**2+(self.y-point.y)**2)<2 and np.sqrt((self.x-point.x)**2+(self.y-point.y)**2)!=0:
                        normal_vector=(point.x-self.x,point.y-self.y)/np.linalg.norm((point.x-self.x,point.y-self.y))
                        mod=np.sqrt(self.vx**2+self.vy**2)
                        self.vx=mod*normal_vector[0]
                        self.vy=mod*normal_vector[1]
                        self.x=point.x+2*normal_vector[0]
                        self.y=point.y+2*normal_vector[1]

    def update(self, dt):
        """
        Update the position of the point based on the forces acting on it
        """
        self.x += self.vx * dt
        self.y += self.vy * dt
        self.vx += self.fx * dt / self.mass
        self.vy += self.fy * dt / self.mass

    def draw(self, screen):
        """
        Draw the point on the screen
        """
        pg.draw.circle(screen, BLACK, (int(self.x), int(self.y)), 2)
    

class Spring:
    """
    Spring class
    p1: first point
    p2: second point
    k: spring constant
    l: rest length
    """
    def __init__(self, p1, p2, k, l):
        self.p1 = p1
        self.p2 = p2
        self.k = k
        self.l = l

    def update(self):
        """
        Apply the spring force to the points
        """
        dx = self.p2.x - self.p1.x
        dy = self.p2.y - self.p1.y
        d = np.sqrt(dx * dx + dy * dy)
        f = self.k * (d - self.l) + C * (self.p2.vx - self.p1.vx) * (dx / d) + C * (self.p2.vy - self.p1.vy) * (dy / d)
        self.p1.fx += f * dx / d
        self.p1.fy += f * dy / d
        self.p2.fx -= f * dx / d
        self.p2.fy -= f * dy / d

    def draw(self, screen):
        """
        Draw the spring
        """
        pg.draw.line(screen, BLACK, (int(self.p1.x), int(self.p1.y)), (int(self.p2.x), int(self.p2.y)), 1)

class SoftBody:
    """
    Parent class for all soft-body objects in the simulation
    points: list of Point objects
    springs: list of Spring objects
    """
    def __init__(self, points, springs):
        self.points = points
        self.springs = springs
        self.x=points[0].x
        self.y=points[0].y
        self.border=[]
        self.color=(np.random.randint(0,255),np.random.randint(0,255),np.random.randint(0,255))
        self.cm_x = sum([p.x * p.mass for p in self.points]) / sum([p.mass for p in self.points])
        self.cm_y = sum([p.y * p.mass for p in self.points]) / sum([p.mass for p in self.points])

    def update(self, dt, blocks, bodies):
        """
        Update the position of all points in the body
        """
        for p in self.points:
            p.fx = 0
            p.fy = 0
        for s in self.springs:
            s.update()
        for p in self.points:
            p.gravity(G)
            p.collision_floor(blocks)
            p.colission_bodies(bodies)
            p.self_colission(self.points)
            p.update(dt)
        self.x=self.points[0].x
        self.y=self.points[0].y
        self.cm_x = sum([p.x * p.mass for p in self.points]) / sum([p.mass for p in self.points])
        self.cm_y = sum([p.y * p.mass for p in self.points]) / sum([p.mass for p in self.points])

    def draw(self, screen):
        """
        Draw the body to the screen
        """
        pg.draw.polygon(screen, self.color, [(self.points[b].x, self.points[b].y) for b in self.border])
        if draw_mesh:
            for s in self.springs:
                s.draw(screen)
            for p in self.points:
                p.draw(screen)
            pg.draw.circle(screen, BLUE, (int(self.cm_x), int(self.cm_y)), 5)

class Rectangle(SoftBody):
    """
    x: x coordinate of the top left node
    y: y coordinate of the top left node
    length: length of the rectangle [m]
    height: height of the rectangle [m]
    num_points_length: number of points lengthwise
    num_points_height: number of points heightwise
    mass: mass of the rectangle [kg]
    """
    def __init__(self, x, y, length, height, num_points_length, num_points_height, mass):
        self.length = length
        self.height = height
        self.num_points_length = num_points_length
        self.num_points_height = num_points_height
        self.mass = mass
        points = []
        springs = []
        for i in range(num_points_length):
            for j in range(num_points_height):
                points.append(Point(x+(length/(num_points_length-1))*i, y+(height/(num_points_height-1))*j, mass/(num_points_length*num_points_height)))
        for i in range(num_points_length):
            for j in range(num_points_height):
                if i > 0: 
                    springs.append(Spring(points[i*num_points_height+j], points[(i-1)*num_points_height+j], K, length/(num_points_length-1)))
                if i < num_points_length-1:
                    springs.append(Spring(points[i*num_points_height+j], points[(i+1)*num_points_height+j], K, length/(num_points_length-1)))
                if j > 0:
                    springs.append(Spring(points[i*num_points_height+j], points[i*num_points_height+j-1], K, height/(num_points_height-1)))
                if j < num_points_height-1:
                    springs.append(Spring(points[i*num_points_height+j], points[i*num_points_height+j+1], K, height/(num_points_height-1)))
                if i > 0 and j > 0: #TODO
                    springs.append(Spring(points[i*num_points_height+j], points[(i-1)*num_points_height+j-1], K, np.sqrt((length/(num_points_length-1))**2+(height/(num_points_height-1))**2)))
                if i > 0 and j < num_points_height-1:
                    springs.append(Spring(points[i*num_points_height+j], points[(i-1)*num_points_height+j+1], K, np.sqrt((length/(num_points_length-1))**2+(height/(num_points_height-1))**2)))
                if i < num_points_length-1 and j > 0:
                    springs.append(Spring(points[i*num_points_height+j], points[(i+1)*num_points_height+j-1], K, np.sqrt((length/(num_points_length-1))**2+(height/(num_points_height-1))**2)))
                if i < num_points_length-1 and j < num_points_height-1:
                    springs.append(Spring(points[i*num_points_height+j], points[(i+1)*num_points_height+j+1], K, np.sqrt((length/(num_points_length-1))**2+(height/(num_points_height-1))**2)))
        super().__init__(points, springs)
        for i in range(num_points_height):
            self.border.append(i)
        for i in range(num_points_length):
            self.border.append((num_points_height-1)+i*num_points_height)
        for i in range(num_points_height):
            self.border.append((num_points_length*num_points_height-1)-i)
        for i in range(num_points_length):
            self.border.append((num_points_height*(num_points_length-1))-i*num_points_height)


class Square(Rectangle):
    """
    x: x coordinate of the top left node
    y: y coordinate of the top left node
    length: length of the square's side [m]
    num_points: number of points per side
    mass: mass of the square [kg]
    """
    def __init__(self, x, y, length, num_points, mass): #num_points is the number of points per side
        super().__init__(x,y,length,length,num_points,num_points,mass)

class Block():
    """
    x: x coordinate of the top left node
    y: y coordinate of the top left node
    theta: angle of the block
    length: length of the block [m]
    height: height of the block [m]
    """
    def __init__(self, top_left, top_right):
        self.x = top_left[0]
        self.y = top_left[1]
        vertices = []
        self.theta = np.arctan((top_right[1]-top_left[1])/((top_right[0]-top_left[0])+0.00001))

        vertices.append((top_left[0], top_left[1]))
        vertices.append((top_right[0], top_right[1]))
        vertices.append((top_right[0]-WALL_WIDTH*np.sin(self.theta), top_right[1]+WALL_WIDTH*np.cos(self.theta)))
        vertices.append((top_left[0]-WALL_WIDTH*np.sin(self.theta), top_left[1]+WALL_WIDTH*np.cos(self.theta)))
        self.vertices=vertices


    def draw(self, screen):
        """
        Draw the block to the screen
        """
        pg.draw.polygon(screen, (0,0,0), self.vertices)
