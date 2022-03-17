#!/usr/bin/env python

import time
import os
import pygame
import sys
import math
import numpy as np

class Node:
    def __init__(self, state, parent, cost_to_come, cost_to_go, cost, ul, ur):
        
        self.state = state                                                     
        self.parent = parent                                                   
        self.cost_to_come = cost_to_come                                       
        self.cost_to_go = cost_to_go                                          
        self.cost = cost                                                       
        self.ul = ul                      
        self.ur = ur                          
    
    def __repr__(self):
        return str(self.state)
    
    # def switch_(self, fn):
    #     registry = dict()
    #     registry['default'] = fn

    #     def register(case):
    #         def inner(fn):
    #             registry[case] = fn
    #             return fn
    #         return inner
        
    #     def decorator(case):
    #         fn = registry.get(case, registry['default'])
    #         return fn()
        
    #     decorator.register = register
    #     return decorator                                                
    
    def check_obstacle_space(self, pot_node):
        """Defining obstacle space constraints using half plane equations."""

        x, y = pot_node[0], pot_node[1]

        if (x < 0) or (x > 400) or (y < 0) or (y > 300):
            return True
        elif (x-90)**2 + (y-70)**2 - 2500 <= 0: 
            return True
        elif (y- 150.727 + 1.4377*x >= 0) and (y - 117.176 - 0.6960*x <= 0) and (y - 466.181 + 1.4419*x <= 0) and (y - 56.308 - 0.6959*x >= 0):
            return True
        elif (x >= 185 and x <= 225 and y <= 295 and y >= 215) or (x >= 225 and x <= 245 and y <= 295 and y >= 255) or (x >= 225 and x <= 245 and y <=245 and y >= 215):    
            return True
        elif ((x-246)**2)/75**2 + ((y-145)**2)/45**2 - 1 <= 0:
            return True
        else:
            return False 
    
    def approximate_node(self, pot_node):
        x, y, theta_rad = pot_node[0], pot_node[1], pot_node[2]
        theta_deg = (theta_rad*180)/math.pi
        dec_x, dec_y = math.modf(x)[0], math.modf(y)[0]
        if dec_x < 0.5:
            x = math.floor(x)
        else:
            x = math.ceil(x)
        
        if dec_y < 0.5:
            y = math.floor(y)
        else:
            y = math.ceil(y)

        theta_deg_norm = theta_deg/30
        theta_dec, theta_int = math.modf(theta_deg_norm)[0], math.modf(theta_deg_norm)[1]

        if theta_dec < 0.5:
            theta_deg = theta_int*30
        else:
            theta_deg = (theta_int+1)*30
            
        theta_deg = theta_deg % 360                                         
        theta_rad = math.radians(theta_deg)
        return [x, y, theta_rad]
    
    def goal_threshold(self):
        x_goal = goal.state[0]
        y_goal = goal.state[1]
        
        if (self.state[0]-x_goal)**2 + (self.state[1]-y_goal)**2 - 25 <= 0:   # radius = 25
            return True
        else:
            return False

    # def common_move(self, potential_node, direction_val, direction):
    #     if not self.check_obstacle_space(potential_node):
    #         move_node = Node(copy.deepcopy(self.state), Node(self.state, self.parent, self.cost, self.distance), self.cost + direction_val, direction_val)
    #         if direction == 'up':
    #             move_node.state[1] = move_node.state[1] + 1
    #         elif direction == 'down':
    #             move_node.state[1] = move_node.state[1] - 1
    #         elif direction == 'left':
    #             move_node.state[0] = move_node.state[0] - 1
    #         elif direction == 'right':
    #             move_node.state[0] = move_node.state[0] + 1
    #         elif direction == 'up_left':
    #             move_node.state[0], move_node.state[1]  = move_node.state[0] - 1, move_node.state[1] + 1
    #         elif direction == 'up_right':
    #             move_node.state[0], move_node.state[1]  = move_node.state[0] + 1, move_node.state[1] + 1
    #         elif direction == 'down_left':
    #             move_node.state[0], move_node.state[1]  = move_node.state[0] - 1, move_node.state[1] - 1
    #         elif direction == 'down_right':
    #             move_node.state[0], move_node.state[1]  = move_node.state[0] - 1, move_node.state[1] - 1
    #         return move_node
    #     return None

    # def up(self):
    #     potential_node = (self.state[0], self.state[1] + 1)
    #     up_node = self.common_move(potential_node, 1, "up")
    #     return up_node

    # def down(self):
    #     potential_node = (self.state[0], self.state[1] - 1)
    #     down_node = self.common_move(potential_node, 1, "down")
    #     return down_node

    # def left(self):
    #     potential_node = (self.state[0] - 1, self.state[1])
    #     left_node = self.common_move(potential_node, 1, "left")
    #     return left_node

    # def right(self):
    #     potential_node = (self.state[0] + 1, self.state[1])
    #     right_node = self.common_move(potential_node, 1, "right")
    #     return right_node
    
    # def up_left(self):
    #     potential_node = (self.state[0] - 1, self.state[1] + 1)
    #     up_left = self.common_move(potential_node, 1.414, "up_left")
    #     return up_left
    
    # def up_right(self):
    #     potential_node = (self.state[0] + 1, self.state[1] + 1)
    #     up_right = self.common_move(potential_node, 1.414, "up_right")
    #     return up_right

    # def down_left(self):
    #     potential_node = (self.state[0] - 1, self.state[1] - 1)
    #     down_left = self.common_move(potential_node, 1.414, "down_left")
    #     return down_left

    # def down_right(self):
    #     potential_node = (self.state[0] + 1, self.state[1] - 1)
    #     down_right = self.common_move(potential_node, 1.414, "down_right")
    #     return down_right

    # def generate_children(self):
    #     children = []
    #     @self.switch_
    #     def move():
    #         return "Invalid move"
        
    #     move.register("up")(lambda: children.append(self.up()))
    #     move.register("down")(lambda: children.append(self.down()))
    #     move.register("left")(lambda: children.append(self.left()))
    #     move.register("right")(lambda: children.append(self.right()))  
    #     move.register("up_left")(lambda: children.append(self.up_left()))       
    #     move.register("up_right")(lambda: children.append(self.up_right()))       
    #     move.register("down_left")(lambda: children.append(self.down_left()))       
    #     move.register("down_right")(lambda: children.append(self.down_right()))  

    #     if self.up():
    #         move("up")
    #     if self.down():
    #         move("down")
    #     if self.left():
    #         move("left")
    #     if self.right():
    #         move("right")
    #     if self.up_left():
    #         move("up_left")
    #     if self.up_right():
    #         move("up_right")
    #     if self.down_left():
    #         move("down_left")
    #     if self.down_right():
    #         move("down_right")
        
    #     return children
    
    
    def generate_children(self):
        
        children = []
        r = 3.8 
        u1 = (2*math.pi*r*10)/60 
        u2 = (2*math.pi*r*10)/60   
        
        actions = [[u1,u1], [u2,u2],[u1,0],[0,u2],[u1,u2],[u2,u1],[u2,0],[0,u1]]
        
        for action in actions:
            child = self.generate_child(action[0], action[1])
            if child:
                children.append(child)
                
        return children
    
    def generate_child(self, ul, ur):
        t, x, y = 0, 0, 0     
        r = 3.8    
        L = 3.54    
        dt = 0.05
        theta = self.state[2]
        
        D=0
        start = self.state
        
        while t<1:    

            t = t + dt
            dx = 0.5*r * (ul + ur) * math.cos(theta) * dt
            x += dx        
            dy = 0.5*r * (ul + ur) * math.sin(theta) * dt        
            y += dy
            theta+= (r / L) * (ur - ul) * dt        
            D = D + (dx**2 + dy**2)**0.5
            pot_node = [self.state[0] + x, self.state[1] + y, theta]
            
            if self.check_obstacle_space(pot_node):
                return None
        pot_node = self.approximate_node(pot_node)
        cost_to_come = self.cost_to_come + D
        cost_to_go = ((pot_node[0] - goal.state[0])**2 + (pot_node[1] - goal.state[1])**2)**0.5
        cost = cost_to_come + cost_to_go
        new_node = Node(pot_node, self, cost_to_come, cost_to_go, cost, ul, ur)

        return new_node
    
    def find_path(self, goal_node):
        print("Shortest Path: ")
        current_node = goal_node
        path = []

        while(current_node.state != self.state):
            
            path.append(current_node)
            current_node = current_node.parent
        return path

if __name__== "__main__":

    global goal
    global clear

    while True:
        x1, y1, theta_s = map(float, input("Please input the X, Y, theta(in degrees) coordinates of the start node:\n").split())
        x1 , y1 = x1*100, y1*100
        x2, y2, theta_g = map(float, input("Please input the X, Y, theta(in degrees) coordinates of the goal node:\n").split())
        x2 , y2 = x2*100, y2*100 
        print("Chosen wheel RPM: L:50, R:60")
        R1, R2 = 50, 60
        clear = float(input("Please input the clearance:\n"))
        clear = clear*100
        input_node = Node([x1, y1, math.radians(theta_s)] , None, 0, ((x1 - x2)**2 + (y1 - y2)**2)**0.5, ((x1 - x2)**2 + (y1 - y2)**2)**0.5, 0, 0)
        goal =  Node([x2, y2, math.radians(theta_g)] , None, 9999999, 0, 9999999, 0 , 0)
        if goal.check_obstacle_space(goal.state) or input_node.check_obstacle_space(input_node.state):
            print("Input Coordinates are in obstacle space!")
        else:
            break    

    queue = []
    queue.append(input_node)
    visited_states_cost = np.zeros((401,301,12))
    visited_states_parent = [[[None for ori in range(12)]for col in range(301)] for row in range(401)]
    visited_states = []
    visited_states.append(input_node)
    
    t = time.time()
    while queue:
        queue.sort(key = lambda x: x.cost)
        current_node = queue.pop(0)
        if current_node.goal_threshold():
            print("Goal Found\n")      
            shortest = input_node.find_path(current_node)
            break
        children = current_node.generate_children() 
        for child in children:
            r = int(child.state[1])          
            c = int(child.state[0])      
            ang = int(math.degrees(child.state[2])/30)
     
            if visited_states_cost[r][c][ang] == 0:
                visited_states_cost[r][c][ang] = child.cost                                 
                visited_states_parent[r][c][ang] = child.parent.state
                visited_states.append(child)
                queue.append(child)
                print(r,c,ang, "->", child.state[1], child.state[0], child.state[2],)
        
            elif visited_states_cost[r][c][ang] > child.cost:
                print('update alert:',visited_states_cost[r][c][ang] , child.cost)
                visited_states_cost[r][c][ang] = child.cost
                visited_states_parent[r][c][ang] = child.parent.state
                                        
    print("Execution time", time.time()-t)

    pygame.init()

    counter = 0
    screen = pygame.display.set_mode((400,300))
    
    while True:

        # Map Generation in pygame
        screen.fill((0,0,0))
        
        pygame.draw.circle(screen, (255,0,0), (90, 300-70), 50)
        pygame.draw.polygon(screen, (255,0,0), ((44.250, 300-87.107), (15.725, 300-128.12),(163.24 ,300-230.793), (191.715, 300-189.734)))
        pygame.draw.polygon(screen, (255,0,0), ((185,5), (245,5), (245, 85), (185, 85)))
        pygame.draw.ellipse(screen, (255,0,0), ((171,110, 150, 90)))
        
        pygame.draw.circle(screen, (0,0, 255), (90, 300-70), 35)
        pygame.draw.polygon(screen, (0,0, 255), ((48, 300-108), (36.53, 300-124.38),(159.4 ,300-210.416), (170.87, 300-194.036)))
        pygame.draw.polygon(screen, (0,0, 255), ((200,20),(230,20),(230,30),(210,30),(210,60), (230,60), (230, 70), (200, 70)))
        pygame.draw.ellipse(screen, (0,0, 255), ((186,125, 120, 60)))

        # Goal threshold
        pygame.draw.circle(screen, (0,255,0), (goal.state[0], 300-goal.state[1]), 25)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
        if counter == 0:

            for node in visited_states:
                print(node)
                child = node.state
                parent = node.parent
                ul = node.ul
                ur = node.ur
                t, x, y = 0, 0, 0     
                r = 3.8    
                L = 3.54    
                dt = 0.05
                D = 0
                theta = child[2]
                previous_intermediate_node = node.state

                if parent != None:  
                    
                    while t<1:    
                        t = t + dt
                        dx = 0.5*r * (ul + ur) * math.cos(theta) * dt
                        x += dx        
                        dy = 0.5*r * (ul + ur) * math.sin(theta) * dt        
                        y += dy
                        theta+= (r / L) * (ur - ul) * dt        
                        D = D + (dx**2 + dy**2)**0.5
                        
                        # Pot node in not an instance of node class, it is just a list of x,y,theta values (i.e similar to state of the node)
                        pot_node = [child[0] + x, child[1] + y, theta]
                        
                        if node.check_obstacle_space(pot_node):
                            continue

                        pygame.draw.line(screen, (255,255,255), (pot_node[0], 300-pot_node[1]), (previous_intermediate_node[0], 300-previous_intermediate_node[1]))      
                        previous_intermediate_node = pot_node


                # pygame.draw.circle(screen, (255,255,255), (state[0]*2, 300-state[1]*2), 1) 
                pygame.display.update()
            # for node in shortest:
            #     print(node)
            #     child = node.state
            #     parent = node.parent
            #     ul = node.ul
            #     ur = node.ur
            #     t, x, y = 0, 0, 0     
            #     r = 3.8    
            #     L = 3.54    
            #     dt = 0.05
            #     D = 0
            #     theta = child[2]
            #     previous_intermediate_node = node.state

            #     if parent != None:  
                    
            #         while t<1:    
            #             t = t + dt
            #             dx = 0.5*r * (ul + ur) * math.cos(theta) * dt
            #             x += dx        
            #             dy = 0.5*r * (ul + ur) * math.sin(theta) * dt        
            #             y += dy
            #             theta+= (r / L) * (ur - ul) * dt        
            #             D = D + (dx**2 + dy**2)**0.5
                        
            #             # Pot node in not an instance of node class, it is just a list of x,y,theta values (i.e similar to state of the node)
            #             pot_node = [child[0] + x, child[1] + y, theta]
                        
            #             if node.check_obstacle_space(pot_node):
            #                 continue

            #             pygame.draw.line(screen, (255,255,255), (pot_node[0], 300-pot_node[1]), (previous_intermediate_node[0], 300-previous_intermediate_node[1]))      
            #             previous_intermediate_node = pot_node


            #     # pygame.draw.circle(screen, (255,255,255), (state[0]*2, 300-state[1]*2), 1) 
            #     pygame.display.update()
                    
            shortest = shortest[::-1]
            for node in shortest:
                    # # print(node)
                    child, parent, ul, ur = node.state, node.parent, node.ul, node.ur
                
                    pygame.draw.circle(screen, (255, 200, 100), (node.state[0], 300-node.state[1]), 3)
                   
                    child[0], child[1] = child[0]/100, child[1]/100
                    state = str({"child": child, "parent": parent, "ul": ul, "ur": ur})

                    pygame.display.update()
            
        counter +=1    

