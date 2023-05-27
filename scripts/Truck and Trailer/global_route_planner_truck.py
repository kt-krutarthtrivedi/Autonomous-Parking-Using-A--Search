"""
| **@created on:** 03/15/22
| **@author:** Krutarth Trivedi | ktrivedi@wpi.edu
| **@version:** v1.0 | Initial Release
|
| **Description:**
|   RBE550 Motion Planning Coursework - HW3 - Valet
|   Truck with a trailer - Navigation using Ackerman Geometry for truck and trailer(s) geometry
"""

import math
from matplotlib.patches import Rectangle
import matplotlib.pyplot as plt
from numpy import pi, sqrt
import numpy as np

#parameters for displaying output
obstacle_posx = 75  
obstacle_posy = 75  
obstacle_width = 50
obstacle_height = 50
car1_posx = 15  
car1_posy = 20  

agent_posx = 50
agent_posy = 180
agent_trailer_posx = 0      
agent_trailer_posy = agent_posy
agent_theta = 0
car_width = 30  
car_height = 17.5 
agent_height = 17.5
agent_width = 30
padding = 2

#parameters for collision checking matrix for each boundary
agent_bound = [[agent_posx,agent_posy,1],[agent_posx+agent_width,agent_posy,1],[agent_posx+agent_width,agent_posy+agent_height,1],[agent_posx,agent_posy+agent_height,1]]
obstacle = [[obstacle_posx-padding,obstacle_posy-padding],[obstacle_posx+obstacle_width+padding,obstacle_posy-padding],[obstacle_posx+obstacle_width+padding,obstacle_posy+obstacle_height+padding],[obstacle_posx-padding,obstacle_posy+obstacle_height+padding]]
car1 = [[car1_posx-padding,car1_posy-padding],[car1_posx+agent_width+padding,car1_posy-padding],[car1_posx+agent_width+padding,car1_posy+agent_height+padding],[car1_posx-padding,car1_posy+agent_height+padding]]

#parameters for motion planning
agent_start = [agent_posx, agent_posy + car_height/2,0]
agent_trailer_start = [agent_trailer_posx, agent_trailer_posy + car_height/2,0]
agent_goal = [car1_posx+car_width+car_width+20,20+car_height/2,180]

# kinematic prameters
wheelbase = 30
hitch_length = 50
steering_angle = 30
vel = 1

# use to display the truck and trailer at different points 
agent_bound_T = [[-1,31,31,-1],[-8.75,-8.75,8.75,8.75],[1,1,1,1]]       #[[-1,21,21,-1],[-7.5,-7.5,7.5,7.5],[1,1,1,1]] 
agent_trailer_bound_T = [[-1,11,11,-1],[-8.75,-8.75,8.75,8.75],[1,1,1,1]]

#Environment setup.
def environment_position():
    obstacle_center_loc = Rectangle((obstacle_posx, obstacle_posy),obstacle_width,obstacle_height,color ='black')
    car1_loc = Rectangle((car1_posx, car1_posy),car_width, car_height,color ='red')
    goal = Rectangle((car1_posx+car_width+15, (car1_posy/2)),car_width+car_width+10+20, car_height+car_height,fc ='none',ec ='g',lw = 2)
    agent_loc = Rectangle((agent_posx, agent_posy),car_width, car_height,color ='green')
    agent_trailer_loc = Rectangle((agent_trailer_posx,agent_trailer_posy),10,car_height,color = 'green')
    return obstacle_center_loc,car1_loc,agent_loc, agent_trailer_loc, goal

#find the path with shortest distance that will be selected from the priority queue
def priority(queue): 
    min = math.inf
    index = 0
    for check in range(len(queue)):
        _,value,_,_ = queue[check]
        if value<min:
            min = value
            index = check #index of the shortest path
    return index

# to check for visited nodes for the A* algorithm
def check_visited(current,visited):
    for x,y,th,xt,yt,th_t in visited:
        if current[0]== x and current[1]== y and current[2]==th and current[3]== xt and current[4]== yt and current[5]==th_t:
            return True
    return False

# to check if collision free straight path exists between the robot and the goal
def straight_available(x,y,xt,yt):
    boundary_line = [[x,y],[agent_goal[0],agent_goal[1]],[agent_goal[0]+1,agent_goal[1]],[x+1,y]]
    boundary_line_t = [[xt,yt],[agent_goal[0]+50,agent_goal[1]],[agent_goal[0]+1+50,agent_goal[1]],[xt+1,yt]]
    if do_polygons_intersect(boundary_line,obstacle) and do_polygons_intersect(boundary_line_t,obstacle):
        return False
    if do_polygons_intersect(boundary_line,car1) and do_polygons_intersect(boundary_line_t,car1):
        return False
    return True

# to check if two pollygons intersect to check for collision
def do_polygons_intersect(a, b):
    polygons = [a, b]
    minA, maxA, projected, i, i1, j, minB, maxB = None, None, None, None, None, None, None, None

    for i in range(len(polygons)):

        # for each polygon, look at each edge of the polygon, and determine if it separates
        # the two shapes
        polygon = polygons[i]
        for i1 in range(len(polygon)):

            # grab 2 vertices to create an edge
            i2 = (i1 + 1) % len(polygon)
            p1 = polygon[i1]
            p2 = polygon[i2]

            # find the line perpendicular to this edge
            normal = { 'x': p2[1] - p1[1], 'y': p1[0] - p2[0] }

            minA, maxA = None, None
            # for each vertex in the first shape, project it onto the line perpendicular to the edge
            # and keep track of the min and max of these values
            for j in range(len(a)):
                projected = normal['x'] * a[j][0] + normal['y'] * a[j][1]
                if (minA is None) or (projected < minA): 
                    minA = projected

                if (maxA is None) or (projected > maxA):
                    maxA = projected

            # for each vertex in the second shape, project it onto the line perpendicular to the edge
            # and keep track of the min and max of these values
            minB, maxB = None, None
            for j in range(len(b)): 
                projected = normal['x'] * b[j][0] + normal['y'] * b[j][1]
                if (minB is None) or (projected < minB):
                    minB = projected

                if (maxB is None) or (projected > maxB):
                    maxB = projected

            # if there is no overlap between the projects, the edge we are looking at separates the two
            # polygons, and we know there is no overlap
            if (maxA < minB) or (maxB < minA):
                #print("polygons don't intersect!")
                return False
    return True

# to get the boundary of the robot from the axil position using homogeneous transformations 
def get_boundary(x,y,theta,car_type="car"):
    tx = x 
    ty = y 
    th = theta-agent_start[2]
    homogeneous_matrix = [[math.cos(th*(pi/180)),-math.sin(th*(pi/180)),tx],[math.sin(th*(pi/180)),math.cos(th*(pi/180)),ty]]
    if car_type == "car":
        mat_mul = np.dot(homogeneous_matrix,agent_bound_T)
    else:
        mat_mul = np.dot(homogeneous_matrix,agent_trailer_bound_T)
    new_boundary = [[mat_mul[0][0],mat_mul[1][0]],[mat_mul[0][1],mat_mul[1][1]],[mat_mul[0][2],mat_mul[1][2]],[mat_mul[0][3],mat_mul[1][3]]]
    return new_boundary

# to check if the position of the robot is valid or not
def valid_point(x,y,theta,xt,yt,theta_t): #collision checking current_pos(x,y,theta,vel)
    boundary = get_boundary(x,y,theta)
    boundary_t = get_boundary(xt,yt,theta_t,"trailer")

    if x < 1 or y < car_height/2.0 or x > 200-car_width or y > 200-car_height/2.0:
        return False 
    if do_polygons_intersect(boundary,obstacle):# or do_polygons_intersect(boundary_t,obstacle):

        return False
    if do_polygons_intersect(boundary,car1) or do_polygons_intersect(boundary_t,car1):
        return False    
    return True

#Cost function
def cost_function(x1,y1,x2,y2,theta1,theta2,vel): 
    cost = 0
    reverse_penalty = 0
    turn_penalty = 0
    distance = sqrt((pow(x1-x2,2)+pow(y1-y2,2)))
    cost = distance+reverse_penalty+turn_penalty
    return cost


#Heuristic function
def heuristic_function(x,y,theta,xt,yt,theta_t,vel,turn):
    theta_ = 0
    theta = (theta+360)%360
    theta_t = (theta_t+360)%360
    if theta_t == 0:
        theta_t=360
    reverse_penalty = 0 
    turn_penalty = 0
    obstacle_penalty = 0 
    distance = sqrt((pow(agent_goal[0]-x,2)+pow(agent_goal[1]-y,2))) # for x,y
    distance += sqrt(((pow((agent_goal[0]-car_width)-(x+car_width*math.cos(theta*(pi/180))),2)+pow((agent_goal[1]+car_height)-(y+car_width*math.sin(theta*(pi/180))),2))))
    if straight_available(x,y,xt,yt) and not(x>agent_goal[0]-5 and y>agent_goal[1]-5 and x <agent_goal[0]+5 and y <agent_goal[1]+15):
        theta_ = abs((360 + (math.atan2(y-agent_goal[1],x-agent_goal[0]))*(180/pi))%360 - theta+180) # for theta
    else:
        theta_ = 180
    if do_polygons_intersect([[x-15,y],[x+200*math.cos(theta*(pi/180))-15,y+200*math.sin(theta*(pi/180))],[15+x+200*math.cos(theta*(pi/180)),y+200*math.sin(theta*(pi/180))],[x+15,y]],obstacle):
        obstacle_penalty +=10
    if do_polygons_intersect([[x-15,y],[x+200*math.cos(theta*(pi/180))-15,y+200*math.sin(theta*(pi/180))],[15+x+200*math.cos(theta*(pi/180)),y+200*math.sin(theta*(pi/180))],[x+15,y]],obstacle):
        obstacle_penalty +=10
    if vel < 0:
        reverse_penalty = 1
    if abs(theta-theta_t)>15 and not(x>agent_goal[0]-5 and y>agent_goal[1]-5 and x <agent_goal[0]+5 and y <agent_goal[1]+15):
        turn_penalty = 5
    hurestic = distance+theta_+reverse_penalty+turn_penalty+obstacle_penalty
    return hurestic

#defining the neighbour points using the kinematic equation
def get_neighbours(x,y,theta,xt,yt,theta_t):
    neighbour = []
    for i in range(-steering_angle,steering_angle+1,5):
        x_dot = vel*math.cos(theta*(pi/180))
        y_dot = vel*math.sin(theta*(pi/180))
        theta_dot = (vel*math.tan(i*(pi/180))/wheelbase)*(180/pi)
        xt_dot = vel*math.cos(theta_t*(pi/180))
        yt_dot = vel*math.sin(theta_t*(pi/180))
        theta_t_dot = (vel*math.sin((theta-theta_t)*(pi/180))/hitch_length)*(180/pi)
        #theta_dot = (360 + theta_dot*(180/pi))%360
        #print(x,y,theta)
        #print(x_dot,y_dot,theta_dot)
        if(valid_point(x+x_dot,y+y_dot,theta+theta_dot,xt+xt_dot,yt+yt_dot,theta_t+theta_t_dot)):
            neighbour.append([round(x+x_dot,2),round(y+y_dot,2),(round(theta+theta_dot,2))%360,round(xt+xt_dot,2),round(yt+yt_dot,2),(round(theta_t+theta_t_dot,2)+360)%360,1,i])
        if(valid_point(x-x_dot,y-y_dot,theta-theta_dot,xt-xt_dot,yt-yt_dot,theta_t-theta_t_dot)):
            neighbour.append([round(x-x_dot,2),round(y-y_dot,2),(round(theta-theta_dot,2)+360)%360,round(xt-xt_dot,2),round(yt-yt_dot,2),(round(theta_t-theta_t_dot,2)+360)%360,-1,i])
    return neighbour

#A* algorithm to find the shortest path from the start orientation to goal orientation
def A_star():
    open_set = []
    visited = []
    start = [agent_start[0],agent_start[1],agent_start[2],agent_trailer_start[0],agent_trailer_start[1],agent_trailer_start[2]]
    tcost = 0
    gcost = 0
    path = [start]
    open_set.append((start,tcost,gcost,path))
    itr =0

    while len(open_set)>0:
        itr+=1
        index = priority(open_set)
        (shortest,_,gvalue,path) = open_set[index] #select the node with lowest distance
        
        open_set.pop(index)
        if not (check_visited([round(shortest[0]),round(shortest[1]),round(shortest[2]),round(shortest[3]),round(shortest[4]),round(shortest[5])],visited)):
            visited.append([round(shortest[0]),round(shortest[1]),round(shortest[2]),round(shortest[3]),round(shortest[4]),round(shortest[5])])
            if round(shortest[0]) <= agent_goal[0]+5 and round(shortest[0]) >= agent_goal[0]-5 and \
                round(shortest[1]) <= agent_goal[1]+5 and round(shortest[1]) >= agent_goal[1]-5 and shortest[2] <= agent_goal[2]+5 and shortest[2] >= agent_goal[2]-5:
                return path
            neighbours= get_neighbours(shortest[0],shortest[1],shortest[2],shortest[3],shortest[4],shortest[5])
            for neighbour in neighbours:
                vel = neighbour[6]
                turn = neighbour[7]
                temp_gcost = gvalue+(0.1*cost_function(shortest[0],shortest[1],neighbour[0],neighbour[1],shortest[2],neighbour[2],vel))
                temp_tcost = temp_gcost+(0.9*heuristic_function(neighbour[0],neighbour[1],neighbour[2],neighbour[3],neighbour[4],neighbour[5],vel,turn))
                open_set.append((neighbour,temp_tcost,temp_gcost,path+ [neighbour]))
    print("not working")      
    return path