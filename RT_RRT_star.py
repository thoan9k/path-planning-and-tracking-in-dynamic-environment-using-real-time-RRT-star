import time
import random
import pygame
import sim
import os
import sys
from shapely.geometry import Point
from shapely.affinity import scale, rotate
from tkinter import Tk, Canvas
# info = time.get_clock_info('process_time')
# print(info)
import math
from typing import List, Optional
from collections import namedtuple
import python_vrep_main as vrep


# print("Bắt đầu chương trình...")
# sim.simxFinish(-1)
# 
# clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)  # Connect to CoppeliaSim
# 
# if clientID != -1:
    # print("Đã kết nối với remote API server")
# else:
    # print("Kết nối không thành công!")
    # sys.exit("Không thể kết nối")
# 
# Main Objects
#errorcode, robot_position_handle = sim.simxGetObjectHandle(clientID, 'mecanum_robot', sim.simx_opmode_blocking)
# _, wheel1_handle = sim.simxGetObjectHandle(clientID, '/mecanum_robot/wheel1', sim.simx_opmode_blocking)
# _, wheel3_handle = sim.simxGetObjectHandle(clientID, '/mecanum_robot/wheel3', sim.simx_opmode_blocking)
# _, wheel4_handle = sim.simxGetObjectHandle(clientID, '/mecanum_robot/wheel4', sim.simx_opmode_blocking)
# _, wheel2_handle = sim.simxGetObjectHandle(clientID, '/mecanum_robot/wheel2', sim.simx_opmode_blocking)
# 
# _, obstacles_handle = sim.simxGetObjectHandle(clientID, 'obstacles', sim.simx_opmode_blocking)
# _, goal_handle = sim.simxGetObjectHandle(clientID, 'Goal', sim.simx_opmode_blocking)
# Tạo kiểu tương đương với ofVec2f và ofColor
Vec2f = namedtuple('Vec2f', ['x', 'y'])
Color = namedtuple('Color', ['r', 'g', 'b'])


def distance_point_to_segment(px, py, x1, y1, x2, y2):
    """Hàm tính khoảng cách từ một điểm (px, py) đến đoạn thẳng (x1, y1)-(x2, y2)."""
    # Tính vector đoạn thẳng và vector từ đầu đoạn đến điểm
    dx, dy = x2 - x1, y2 - y1
    if dx == 0 and dy == 0:
        # Nếu hai điểm của đoạn thẳng trùng nhau
        return math.hypot(px - x1, py - y1)
    
    # Tính hệ số t cho vị trí gần nhất trên đoạn thẳng
    t = ((px - x1) * dx + (py - y1) * dy) / (dx**2 + dy**2)
    t = max(0, min(1, t))
    
    # Tọa độ điểm gần nhất trên đoạn thẳng
    closest_x = x1 + t * dx
    closest_y = y1 + t * dy

    # Khoảng cách từ điểm đến điểm gần nhất trên đoạn thẳng
    return math.hypot(px - closest_x, py - closest_y)

def is_point_on_path(point, path, epsilon=0.1):
    """
    Kiểm tra xem điểm có nằm trên đường dẫn không.
    point: tuple (px, py) - Tọa độ của điểm cần kiểm tra.
    path: list of tuples [(x1, y1), (x2, y2), ...] - Các tọa độ của đường dẫn.
    epsilon: float - Ngưỡng khoảng cách để xác định điểm nằm trên đường.
    """
    px, py = point
    for i in range(len(path) - 1):
        # Lấy hai điểm liên tiếp trên đường dẫn
        x1, y1 = path[i]
        x2, y2 = path[i + 1]
        
        # Tính khoảng cách từ điểm đến đoạn thẳng giữa x1,y1 và x2,y2
        distance = distance_point_to_segment(px, py, x1, y1, x2, y2)
        
        if distance <= epsilon:
            return True  # Điểm nằm trên đường dẫn
    return False  # Điểm không nằm trên đường dẫn
class RT_RRT_star:
    def __init__(self, agent, goal, obstacles, mapdimentions, kmax=5, k=100, rs = 10, rb=30, ro=200, rg=36):
        self.agent = agent
        self.goal = goal
        self.old_goal = None
        self.obstacles = obstacles
        self.dynamic_obstacles = []
        self.Qr = []
        self.Qs = []
        self.mapdimensions = mapdimentions
        self.maph, self.mapw = self.mapdimensions
        pygame.display.set_caption('RT-RRT* path planning for mecanum on Vrep')
        self.surface = pygame.display.set_mode((self.mapw, self.maph)) #kích thước map
        self.surface.fill((255, 255, 255))
        self.x = []
        self.y = []
        self.parent = []
        self.child  = []
        self.cost = []
        self.blocked_nodes = []
        self.is_blocked = []
        # initialize the tree
        (x, y) = agent
        self.x.append(x)
        self.y.append(y)
        self.parent.append(0)
        self.cost.append(0)
        self.is_blocked.append(False)
        self.k = k        # limit the number of nodes to cover the environment
        self.kmax = kmax  # maximum neighbors around any one node
        self.rs = rs
        self.rb = rb
        self.ro = ro
        self.rg = rg
        self.root = 0
        self.d = 50
        # path
        self.goalstate = None
        self.goalFlag = False
        self.path = []
        self.minimal_path = None
        self.path_nodes = None
        self.old_path = None
        self.restarting = False
        #  grid area
        self.grid = []
        self.grid_pos = []
        self.num_grid = 0

    def add_node2grid_pos(self, node, Flag='None'):
        x = self.x[node]
        y = self.y[node]
        for i in range(0,self.num_grid):
            for j in range(0, self.num_grid):  
                if self.grid[i][j].collidepoint(x, y):
                    if Flag=='None':  
                        self.grid_pos.append((i,j))  
                    else:
                        self.grid_pos[node]=(i,j)
                    break 
        # print(f"grid pos: {self.grid_pos}") 
    def add_node(self, n, x, y):
        self.x.insert(n, x)
        self.y.append(y)
        self.add_node2grid_pos(n)

    def goal_grid(self,Xu=[]):
        x = None
        y = None
        if self.goal:
            x = self.goal[0]
            y = self.goal[1]
        if Xu:
            x = Xu[0]
            y = Xu[1]
        for i in range(0,self.num_grid):
            for j in range(0, self.num_grid):  
                if self.grid[i][j].collidepoint((x, y)):  
                     return (i,j)
                    
    def node_in_grid(self, pos, Xu=None):  # Trả về các node trong một vị trí grid
        nodes = [i for i in range(self.number_of_nodes()) if self.grid_pos[i] == pos and (Xu is None or i != Xu)]
        #print(f' Các nodes trong grid{pos} là: {nodes}')
        return nodes

    def get_adjacent_positions(self, pos):
        i, j = pos
        n = self.num_grid
        adjacent_positions = []
        temp = [pos]
        m = 1
        dem = 0
        while dem < 1: # tìm cho đến khi thấy ít nhất 1 node khác với 
            for x in range(i - m, i + m + 1):
                for y in range(j - m, j + m + 1):
                    if 0 <= x < n and 0 <= y < n and (x,y) != pos:
                        temp.append((x, y))
                        if self.node_in_grid((x,y)):
                            dem+=1
                        else:
                            l = len(temp)-1
                            temp.pop(l)
            if dem >=1:
                adjacent_positions = temp
            temp = []           
            m += 1
        # print(f'Các grid liền kề {pos}: {adjacent_positions}')
        return adjacent_positions

    def get_XSI(self, Xu): # Xu là 1 node bất kỳ
        m = None
        p = None
        is_int = isinstance(Xu, (int, float))
        if not is_int: # Nếu Xu là toạ độ
            p = self.goal_grid(Xu)
        elif is_int:
            m = Xu
            p = self.grid_pos[m]
        nodes = []
        # Lấy các vị trí liền kề từ vị trí của Xu
        adjacent_positions = self.get_adjacent_positions(p)
        if not adjacent_positions:
            return
        # Duyệt qua từng vị trí liền kề và tìm các nút ở vị trí đó
        for pos in adjacent_positions:
            if is_int:
                nodes.extend(self.node_in_grid(pos,Xu))
            else:
                nodes.extend(self.node_in_grid(pos))
                if Xu == self.goal and self.goalstate:
                    for i in range(len(nodes)):
                        if nodes[i] == self.goalstate:
                            nodes.pop(i)
                            break
        if isinstance(Xu, (int, float)) and Xu == self.root:
            for node in nodes:
                pygame.draw.circle(self.surface, (255,0,0), (self.x[node], self.y[node]), 2)
                pygame.display.update()
        
        
        return nodes
    
    def FindNodesNear(self, Xrand):  # Xrand là 1 node đã được lấy mẫu ngẫu nhiên hoặc là toạ độ 
        is_int = isinstance(Xrand, (int, float))
        XSI = self.get_XSI(Xrand)
        neighbors = []
        for node in XSI:
            if is_int:
                if self.distance(node, Xrand)<2*self.d:
                    neighbors.append(node)
            else:
                if math.dist((self.x[node], self.y[node]), Xrand)<2*self.d:
                    neighbors.append(node)
                
            #     if math.dist((self.x[node], self.y[node]), Xrand)<self.d:
            #         neighbors.append(node)
        # if is_int:
            
        #     for i in range(len(XSI)):
        #         if XSI[i] == Xrand:
        #             XSI.pop(i)
        #             break
        # elif self.goalstate: # loại bỏ node đó là goal
        #     if Xrand == self.goal:
        #         for i in range(len(XSI)):
        #             if XSI[i] == self.goalstate:
        #                 XSI.pop(i)
        #                 break
        return neighbors
        
    # def Node_closest(self, Xrand):
    #     xnear = self.FindNodesNear(Xrand)
    #     # print(f'các node lân cận :{xnear}')
    #     if not xnear:  # Kiểm tra xnear không rỗng
    #         return None  # Trả về None nếu không có node gần

    #     cmin = float('inf')
    #     Xclosest = None
    #     for node in xnear:
    #         if node == Xrand:
    #             continue
    #         c = self.distance(Xrand, node)
    #         if c < cmin:
    #             cmin = c
    #             Xclosest = node
    #     return Xclosest

    def remove_node(self, n):
        self.x.pop(n)
        self.y.pop(n)
        self.grid_pos.pop(n)

    def add_edge(self, parent, child):
        self.parent.insert(child, parent)
    def remove_edge(self, n):
        self.parent.pop(n)
    def number_of_nodes(self):
        return len(self.x)
    def distance(self, n1, n2):
        (x1, y1) = (self.x[n1], self.y[n1])
        (x2, y2) = (self.x[n2], self.y[n2])
        dx = (float(x1) - float(x2)) ** 2
        dy = (float(y1) - float(y2)) ** 2
        return (dx + dy) ** 0.5
    def sample_envir(self):
        x = int(random.uniform(0, self.mapw))
        y = int(random.uniform(0, self.maph))

        return x, y
    def divide_grid(self, numberofsquares): # map có kích thước vuông
        self.num_grid = int(numberofsquares**0.5)
        size = int(self.mapw / self.num_grid)
        self.grid = [[[] for _ in range(self.num_grid)] for _ in range(self.num_grid)]
        for i in range(0, self.num_grid):
            for j in range(0, self.num_grid):
                upper = (size*i, size*j)
                square = pygame.Rect(upper, (size,      size))
                
                self.grid[i][j] = square

        return self.grid
    def crossObstacle(self, x1, x2, y1, y2):
        obs_ = self.obstacles.copy()
        while len(obs_) > 0:
            rect = obs_.pop(0)
            for i in range(0, 101):
                u = i / 100
                x_ = x1 * u + x2 * (1 - u)
                y_ = y1 * u + y2 * (1 - u)
                if rect.collidepoint(x_, y_):
                    return True
        return False

    def connect(self, n1, n2):
        x1, y1 = self.x[n1], self.y[n1]
        x2, y2 = self.x[n2], self.y[n2]
        if self.crossObstacle_forstep(x1, x2, y1, y2):
            self.remove_node(n2)
            return False
        else:
            self.add_edge(n1, n2)
            cost_ = self.cost[n1] + self.distance(n1, n2)
            self.cost.append(cost_)
            return True  
    # đầu vào có thể là 1 node bất kỳ hoặc 1 danh sách các toạ độ
    def cost_(self, n_current):  # tính chi phí hay khoảng cách từ điểm hiện tại(xk) tới self.root(x0)
        #blocknodes = self.update_blocked_nodes()
        #print(f'Các nodes bị chăn: {blocknodes}')    
        c = 0
        if isinstance(n_current, int): # nếu n_current là 1 node bất kỳ
            c = 0
            while n_current != self.root:
                if n_current in self.blocked_nodes:
                    return float('inf')
                
                c += self.distance(n_current, self.parent[n_current])
                n_current = self.parent[n_current]
        elif isinstance(n_current, (list, tuple)): # nếu n_current là danh sách các toạ độ 
            for i in range(len(n_current)-1):
                c += math.dist(n_current[i], n_current[i+1])    
                #print("toạ độ")
        return c


    def Rewire_random_node(self): # node có sẵn trong hàng đợi Qr (xrand hoặc xclosest )
        start_time = time.time()
        elapse_time = 0
        while(elapse_time<0.03 and self.Qr):
            print(f'Qr:{self.Qr}')
            xr = self.Qr.pop(0)
            Xnear = self.FindNodesNear(xr)
            for x_near in Xnear:
                if self.is_blocked[x_near]:
                    continue
                c_old = self.cost_(x_near)
                c_new = self.cost_(xr)+ self.distance(xr, x_near)
                if c_new<c_old and not self.crossObstacle(self.x[xr], self.x[x_near], self.y[xr], self.y[x_near]):
                    pygame.draw.line(self.surface, (255, 255, 255),(self.x[x_near], self.y[x_near]),(self.x[self.parent[x_near]], self.y[self.parent[x_near]]),1)
                    
                    #self.parent[x_near] = xr
                    self.set_parent(x_near, xr)
                    pygame.draw.line(self.surface, (0, 0, 255),(self.x[x_near], self.y[x_near]),(self.x[xr], self.y[xr]),1)
                    if x_near not in self.Qr:
                        self.Qr.append(x_near)

            end_time = time.time()
            elapse_time = end_time - start_time

    def set_parent(self, child, new_parent):
        # Kiểm tra chu trình bằng cách duyệt ngược từ child đến root
        current_node = new_parent
        while current_node != self.root:
            if current_node == child:
                #print(f"Lỗi: Không thể gán {new_parent} làm parent của {child} vì sẽ tạo chu trình.")
                return  # Dừng lại và không cập nhật parent để tránh chu trình
            current_node = self.parent[current_node]

        # Nếu không có chu trình, cập nhật parent
        self.parent[child] = new_parent

    def Rewire_Tree_root(self):
        start_time = time.time()
        elapse_time = 0
        if self.restarting:
            self.Qs = []
            self.restarting = False
            self.Qs.append(self.root)
            # print(f'Đã thêm gốc mới vào hàng đợi Qs: {self.root}')
        while(elapse_time<0.01 and self.Qs):
            # print(f'Qs{self.Qs}')
            xs = self.Qs.pop(0)
            Xnear = self.FindNodesNear(xs)
            if not Xnear:
                break
            for x_near in Xnear:
                if self.is_blocked[x_near]:
                    continue
                c_old = self.cost_(x_near)
                c_new = self.cost_(xs)+self.distance(xs, x_near)
                if c_new<c_old and not self.crossObstacle(self.x[xs], self.x[x_near], self.y[xs], self.y[x_near]):
                    pygame.draw.line(self.surface, (255, 255, 255),(self.x[x_near], self.y[x_near]),(self.x[self.parent[x_near]], self.y[self.parent[x_near]]),1)
                    
                    #self.parent[x_near] = xs
                    self.set_parent(x_near, xs)
                    pygame.draw.line(self.surface, (0, 0, 255),(self.x[x_near], self.y[x_near]),(self.x[xs], self.y[xs]),1)
                    if x_near not in self.Qs:
                        self.Qs.append(x_near)
            end_time = time.time()
            elapse_time = end_time - start_time

    def get_random_point_in_ellipse(self):
        focus1 = [self.x[self.root], self.y[self.root]]
        focus2 = self.goal
    
        # Tính trung điểm giữa hai tiêu điểm (tâm của ellipse)
        center_x = (focus1[0] + focus2[0]) / 2
        center_y = (focus1[1] + focus2[1]) / 2
        center = Point(center_x, center_y)
        # Tính khoảng cách giữa hai tiêu điểm
        focal_distance = math.dist(focus1, focus2)
        major_axis_length = focal_distance + 30
        # if self.goalstate:
        #     major_axis_length =  self.cost_(self.goalstate)
        # Kiểm tra điều kiện tạo ellipse
        if major_axis_length < focal_distance:
            raise ValueError("Chiều dài trục lớn phải lớn hơn hoặc bằng khoảng cách giữa hai tiêu điểm.")
        # Tính trục nhỏ dựa trên trục lớn và khoảng cách giữa hai tiêu điểm
        minor_axis_length = math.sqrt(major_axis_length**2 - focal_distance**2)
        # Tạo ellipse bằng cách tạo một đường tròn tại tâm và biến đổi thành ellipse
        
        ellipse = center.buffer(1)  # Tạo đường tròn bán kính 1 tại tâm
        ellipse = scale(ellipse, major_axis_length / 2, minor_axis_length / 2)  # Biến đổi thành ellipse
        # Xác định góc giữa đường nối hai tiêu điểm và trục ngang
        angle = math.degrees(math.atan2(focus2[1] - focus1[1], focus2[0] - focus1[0]))
        # Xoay ellipse theo góc này
        ellipse = rotate(ellipse, angle, origin=center)
        # Lấy bounding box của ellipse
        min_x, min_y, max_x, max_y = ellipse.bounds
        if max_y >self.maph:
            max_y =self.maph
        if  max_x>self.mapw:
            max_x =self.mapw
        while True:
            # Tạo một điểm ngẫu nhiên trong bounding box của ellipse
            random_x = max(0, int(random.uniform(min_x, max_x)))
            random_y = max(0, int(random.uniform(min_y, max_y)))
            random_point = Point(random_x, random_y)
            # Kiểm tra xem điểm này có nằm trong ellipse không
            if ellipse.contains(random_point):
                return random_point.x, random_point.y  
     
    def sample(self, alpha=0.1, beta=2): # trả về node 
        Pr = float(random.uniform(0, 1))
        # uniform X
        #x_uniform, y_uniform = self.sample_envir()
        # line to goal
        #x_goal, y_goal = self.goal
        xrand = None
        if not self.goal:
            xrand = self.sample_envir()
            if self.path:
                while is_point_on_path((xrand[0], xrand[1]), self.path):
                    xrand = self.sample_envir()
        elif Pr > 1 - alpha:  #  random in the line to goal 
            xnear = self.get_XSI(self.goal)
            xclosest = xnear[0]
            x = int(random.uniform(min(self.x[xclosest], self.goal[0]), max(self.x[xclosest], self.goal[0])))
            y = int(random.uniform(min(self.y[xclosest], self.goal[1]), max(self.y[xclosest], self.goal[1])))
            path = []
            numpoints = 10
            for i in range(0, numpoints):
                u = i / numpoints
                x = int(self.goal[0] * u + self.x[xclosest] * (1 - u))
                y = int(self.goal[1] * u + self.y[xclosest] * (1 - u))
                path.append((x, y))
            ran = int(random.uniform(1, numpoints-1)) 
            xrand = path[ran]
            # print("pr1")
        elif Pr <= (1-alpha)/beta :
            xrand = self.sample_envir()
            if self.path:
                while is_point_on_path((xrand[0], xrand[1]), self.path):
                    xrand = self.sample_envir()
            # print("pr2")
        # Ellipsis
        else: 
            xrand = self.get_random_point_in_ellipse()
            # print("pr3")
        
        n = self.number_of_nodes()
        self.add_node(n, int(xrand[0]),int(xrand[1]))
        # while n in self.update_blocked_nodes():
        #     if self.goal:
        #         xrand = self.get_random_point_in_ellipse()
        #     else:
        #         xrand = self.sample_envir()
        #     self.x[n]= int(xrand[0])
        #     self.y[n]= int(xrand[1])
        """Tìm node gần nhất"""
        XSI = self.get_XSI(n)
        xclosest = min(XSI, key=lambda x: self.distance(x, n))
        # print(f'xclosest :{xclosest, self.x[xclosest], self.y[xclosest]}')
        #d = self.distance(n, xclosest)
                
        # print(f' Đã thêm node {n} : {self.x[n], self.y[n]}')
        number = self.number_of_nodes()
        U_X = self.maph*self.mapw
        d = math.sqrt(U_X*self.kmax/(math.pi*number))

        if d < self.rs:
            d = self.rs
        self.d = d
        # if self.distance(n, xclosest)> d:
        #     dy = self.y[n] - self.y[xclosest]
        #     dx = self.x[n] - self.x[xclosest]
        #     theta = math.atan2(dy, dx)
        #     self.x[n] = int(self.x[xclosest] + d * math.cos(theta))
        #     self.y[n] = int(self.y[xclosest] + d * math.sin(theta))
        #     self.add_node2grid_pos(n, 'Replace')
        # # 
            # print(f' Đã thêm lại node {n} : {self.x[n], self.y[n]}')
        
        return n, xclosest


    def Add_node_to_Tree(self, Xnew, xclosest, Xnear):
        self.add_edge(xclosest, Xnew)
        cmin = self.cost_(xclosest) + self.distance(xclosest, Xnew)  
        for x_near in Xnear:
            cnew = self.cost_(x_near) + self.distance(x_near, Xnew)  
            if cnew < cmin and not self.crossObstacle(self.x[x_near], self.x[Xnew], self.y[x_near],self.y[Xnew]):  
                cmin = cnew
                xclosest = x_near  
        # thêm cạnh và node(đã thêm trc đó)
        pygame.draw.circle(self.surface, (10,10,10), (self.x[Xnew], self.y[Xnew]), 2, 0) # vẽ node
        pygame.draw.line(self.surface, (0,0,255),(self.x[xclosest], self.y[xclosest]) ,(self.x[Xnew], self.y[Xnew]),1) # vẽ nhánh
        pygame.display.update()
        # self.add_edge(xclosest, Xnew)
        
        self.parent[Xnew] = xclosest
        self.is_blocked.append(False)
        # print(f'Đã thêm nhánh {xclosest, Xnew}')
        #if not self.goal:
        #    return
        #if self.goalstate:
        #    C_new = self.cost_(self.goalstate)
        #if math.dist((self.x[Xnew], self.y[Xnew]), (self.goal[0], self.goal[1])) < self.rg and not self.crossObstacle(self.x[Xnew] , self.goal[0], self.y[Xnew], self.goal[1]):
        #    if not self.goalFlag:
        #        self.goalFlag = True
        #        self.goalstate = Xnew + 1
        #        self.x.insert(self.goalstate,self.goal[0])
        #        self.y.insert(self.goalstate,self.goal[1])
        #        self.parent.append(Xnew)
        #        self.add_node2grid_pos(self.goalstate)
        #        self.minimal_path = self.path_to_goal()
        #    else:
        #        #xoá cạnh cũ và nối cạnh mới
        #        C_old = self.cost_(self.minimal_path)
        #        
        #        if C_new<C_old:
        #            pygame.draw.line(self.surface, (255, 255, 255), (self.x[self.goalstate], self.y[self.goalstate]),
        #                             (self.x[self.parent[self.goalstate]], self.y[self.parent[self.goalstate]]),1 )
        #            self.parent[self.goalstate] = Xnew
#
        #            pygame.draw.line(self.surface, (0, 0, 255), (self.x[self.goalstate], self.y[self.goalstate]),
        #                            (self.x[Xnew], self.y[Xnew]),1 )
        #            self.minimal_path = self.path_to_goal

    def Nodes_in_goalarea(self):
        near_node = self.get_XSI(self.goal)
        # if self.goalstate:
        #     for i in range(len(near_node)):
        #         if near_node[i] == self.goalstate:
        #             near_node.pop(i)
        #             break
            # for i in range(len(near_node)):
            #     if near_node[i] == self.root:
            #         near_node.pop(i)
            #         break
        # print(f'Các nodes trong vùng mục tiêu: {near_node}')
        nodes_in_goalarea = []
        for node in near_node:
            if math.dist((self.x[node], self.y[node]), self.goal)< self.rg and not self.crossObstacle(self.x[node] , self.goal[0], self.y[node], self.goal[1]):
                nodes_in_goalarea.append(node)
        return nodes_in_goalarea
    
    def update_blocked_nodes(self):
        Obst = self.get_dynamic_obs_inside()
        if not Obst:
            return []
        # nếu có vật cản
        # reset bộ đệm chứa các node cản
        self.blocked_nodes = []
        for ob in Obst:
            xnear = self.get_XSI((ob[1], ob[0]))
            #print(f'toạ độ ob: {ob}')
            for node in xnear:
                #pygame.draw.circle(self.surface, (255, 0, 0), (self.x[node], self.y[node]), 2, 0)
                if math.dist((self.x[node], self.y[node]), (ob[1], ob[0])) <= self.rb:
                    self.blocked_nodes.append(node)
                    self.is_blocked[node]=True
                    self.blocked_nodes.append(self.parent[node])
                    self.is_blocked[self.parent[node]]=True
                    childs = self.get_child(node)
                    self.blocked_nodes.extend(childs)
                    for child in childs:
                        self.is_blocked[child]=True
                    # pygame.draw.circle(self.surface, (0, 255, 255), (self.x[node], self.y[node]), 3, 0)
                    # pygame.display.update()
                else:
                    self.is_blocked[node]=False
                    self.is_blocked[self.parent[node]]=False
                    childs = self.get_child(node)
                    for child in childs:
                        self.is_blocked[child]=False
                    
        # if Obst:
            # print(f'Các node bị block:{blocked_nodes}')
        return self.blocked_nodes
    
    def update_path(self):
        # self.update_blocked_nodes()
        # thiết lập chi phí cho node bị chặn trong vùng
        # Dưới đây chỉ xét chi phí trong goal area 
        nodes_in_goalarea = self.Nodes_in_goalarea()
        if not self.old_goal or self.goal != self.old_goal:
            self.goalstate = self.number_of_nodes()
            self.add_node(self.goalstate, self.goal[0], self.goal[1])
            #print(f"Đã thêm vùng grid mới của goal: {self.grid_pos[self.goalstate]}") 
            self.add_edge(nodes_in_goalarea[0],self.goalstate)
            self.is_blocked.append(False)
            self.old_goal = self.goal
        
        if len(nodes_in_goalarea)>=1:
            cmin = float('inf')
            best_node = nodes_in_goalarea[0]
            for node in nodes_in_goalarea:
                """ NHỚ BỎ QUA NODE GOAL """
                if node == self.goalstate :
                    continue
                cnew = self.cost_(node) + self.distance(node, self.goalstate)
                # print(f'c_new = {cnew}')
                if cnew<cmin:
                    cmin = cnew
                    best_node = node
            #self.parent[self.goalstate] = best_node
            self.set_parent(self.goalstate, best_node)
            if self.old_path:
                for i in range(len(self.old_path)-1):
                    pygame.draw.line(self.surface, (255, 255, 255),self.old_path[i], self.old_path[i+1], 5)
            if self.path_to_goal():
                self.minimal_path = self.path_to_goal()
            else:
                return False
            
            for i in range(len(self.minimal_path)-1):
                pygame.draw.line(self.surface, (255, 255, 0),self.minimal_path[i], self.minimal_path[i+1], 5)
            return True
                     
    def Expansion_and_Rewiring(self):
        xrand, xclosest  = self.sample() # đã thêm node vào danh sách 
        #xclosest = self.Node_closest(xrand) # node gần nhất 

        if not self.crossObstacle(self.x[xrand], self.x[xclosest], self.y[xrand], self.y[xclosest]):
            Xnear = self.FindNodesNear(xrand)
            #print(f'Xnear :{Xnear}')
            if len(Xnear) < self.kmax or self.distance(xrand, xclosest) > self.rs:
                self.Add_node_to_Tree(xrand, xclosest, Xnear)
                #  Push xrand to the first of Qr
                self.Qr.insert(0, xrand)
            else:
                self.remove_node(xrand)
                #self.add_edge(xclosest, xrand)
                # print(f'Đã xoá node {xrand}')
                self.Qr.insert(0, xclosest)
            self.Rewire_random_node()  # RewireRandomNode(Qr T )
        else:
             # xoá nếu node đó k tìm được liên kết tới node gần nhất  
            # print(f' Đã xoá node {xrand}: {self.x[xrand], self.y[xrand]}')  
            self.remove_node(xrand)
        
        if self.number_of_nodes()>1:

            self.Rewire_Tree_root()  # RewireFromRoot(Qs T )
    
    def Update_tree(self):
        blocked_nodes = self.update_blocked_nodes()
        for node in blocked_nodes:
            pygame.draw.circle(self.surface, (0,255,255),(self.x[node], self.y[node]),2,0)
        #self.get_dynamic_obs_inside()
        #self.surface.fill((255, 255, 255))
         # Sao chép lưới để duyệt và vẽ
        # grid_copy = self.grid.copy()
        # j=0
        # for row in grid_copy:
            # j+=1
            # i=j%2
            # for squares in row:
                # pygame.draw.rect(self.surface, (220,220,220), squares, i%2)  # Vẽ hình chữ nhật lên bề mặt
                # i+=1
        # self.agent = vrep.map_value((vrep.get_position(robot_position_handle)[1], vrep.get_position(robot_position_handle)[0]), flag='Vrep2Py')
        #pygame.draw.circle(self.surface, (255, 0.618*255, 0.266*255), self.agent, 36, 1)
        #pygame.draw.circle(self.surface, (255, 0.618*255, 0.266*255), self.agent, 15, 0)
        #pygame.draw.circle(self.surface, (0,0, 255), self.agent, self.ro, 2)
        
        # self.goal = vrep.map_value((vrep.get_position(goal_handle)[1], vrep.get_position(goal_handle)[0]), flag='Vrep2Py')
        if self.goal:
            pygame.draw.circle(self.surface, (255, 0, 0), self.goal, self.rg, 1)
            pygame.draw.circle(self.surface, (255, 0, 0), self.goal, 18, 0)
        pygame.draw.circle(self.surface, (0,255, 0), (self.x[self.root], self.y[self.root]), 5, 0)
        obs_list = self.obstacles.copy()
        #print(f'obs = {obs_list}')
        while obs_list:
            ob = obs_list.pop(0)
            pygame.draw.rect(self.surface, (10, 10, 10), ob, 0)
        #self.dynamic_obstacles = get_dynamic_obstacles()
        #print(self.dynamic_obstacles)
        #self.print_tree()
        # Vẽ chướng ngại vật
        obs_ = self.dynamic_obstacles

        obs_[0][1]+=speed
        obs_[1][1]+=speed
        obs_[1][0]+=-speed
        obs_[3][1]+=-speed
        obs_[4][0]+=speed
        obs_[5][0]+=-2*speed
        for d_ob in obs_:
           pygame.draw.circle(self. surface, (0,255,0),(d_ob[1], d_ob[0]), self.rb,1)
           pygame.draw.circle(self. surface, (255,125,0),(d_ob[1], d_ob[0]), self.rb/2,0)
           pygame.display.update()
        pygame.display.update()
        self.old_path = self.minimal_path
    def path_to_goal(self): # đầu ra là chuỗi các toạ độ trên path
        current_node = self.goalstate
        path = []
        self.path_nodes = []
        path.append(self.goal)
        self.path_nodes.append(self.goalstate)
        visited = set()  # Tập hợp các node đã duyệt qua
        while current_node != self.root:
            if current_node in visited:
                print("Lỗi: Phát hiện chu trình trong cây!")
                return []  # Dừng lại nếu phát hiện chu trình
            visited.add(current_node)
            # if current_node == self.parent[current_node]:
            #     break
            current_node = self.parent[current_node]
            # Tiếp tục tìm parent của node hiện tại
            # current_node = self.parent.get(current_node, None)
            node_pos = self.x[current_node], self.y[current_node]
            path.append(node_pos)
            self.path_nodes.append(current_node)
        self.path_nodes.reverse()
        path.reverse()
        return path
    
    def node_in_path(self, current_node):
        nodes = []
        nodes.append(current_node)
        while current_node is not self.root:
            current_node = self.parent[current_node]
            nodes.append(current_node)
        return nodes 
    
    def get_child(self, parentNode):
        child = []
        for i in range(0, self.number_of_nodes()):
            if self.parent[i]== parentNode:
                child.append(i)
        return child

    def Path_planning(self):
        path = []
        current_node = self.root
        next_node = None
        if self.agent == self.goal:
            return
        # if not self.Nodes_in_goalarea() : # if Tree has reached xgoal
        #     return
    
        if self.Nodes_in_goalarea():
            while True:
                if self.update_path():
                    break
            return self.path_nodes
        else:
            # print("ok1")
            return []
            for i in range(self.k):
                min_cost = float('inf') 
                for child in self.get_child(current_node):
                    fc = self.cost_(child) + self.Heuristic(child)
                    if fc<min_cost:
                        min_cost = fc
                        next_node = child
                if next_node is None:
                    break
                
                # di chuyển đến node tiếp theo  và thêm vào đường đi
                path.append(next_node)
                current_node = next_node

            return path
    def get_dynamic_obs_inside(self, Flag='None'):
        Obst = self.dynamic_obstacles
        temp = []
        for ob in Obst:
            if math.dist(self.agent, (ob[1], ob[0])) < self.ro:
                temp.append(ob)
        Obst = temp
        # if Obst: # and Flag=='Update':
        #     print(f'Các vật cản động trong phạm vi ro: {Obst}')
        return Obst
    
    def Heuristic(self, node):
        #Obst = get_dynamic_Obsttacles()
        # chỉ xét vật cản động bên trong ro
        Obst = self.get_dynamic_obs_inside()
        if not Obst:
            math.dist((self.x[node], self.y[node]), self.goal)
        
        for ob in Obst:
            if math.dist(ob, (self.x[node], self.y[node])) < self.rb:
                return float('inf')
        return math.dist((self.x[node], self.y[node]), self.goal)

    def Move(self,target, speed):
        xt = self.x[target]
        yt = self.y[target]
        dy = yt - self.agent[1]
        dx = xt - self.agent[0]
        distance = math.dist(self.agent, (xt, yt))
        if distance == 0:
            return
        movex =self.agent[0] + dx/distance*speed #*math.sqrt(2)
        movey = self.agent[1] + dy/distance*speed  #*math.sqrt(2)
        # print(f'move {movex, movey}')
        self.agent = [int(movex), int(movey)]
        #pygame.draw.circle(self.surface, (255, 0.618*255, 0.266*255), self.agent, 36, 1)
        pygame.draw.circle(self.surface, (255, 0.618*255, 0.266*255), self.agent, 5, 0)
        #pygame.draw.circle(self.surface, (0,0, 255), self.agent, self.ro, 2)
        pygame.display.update()
        
    def close(self, agent, goal):
        distance = math.dist(agent, goal)
        if distance<=self.rs:
            self.restarting = True
            return True
        else:
            return False
    
    def print_tree(self):# not good 
        for i in range(1,self.number_of_nodes()):
            if i in self.update_blocked_nodes(): # không vẽ các node trong vùng vật cản
                continue
            pos1=self.x[i], self.y[i]
            pos2 = self.x[self.parent[i]], self.y[self.parent[i]]
            pygame.draw.line(self.surface, (0,0,255),pos1, pos2, 1)
            pygame.draw.circle(self.surface, (10,10,10), pos1, 2, 0)
        pygame.display.update()

def get_obstacles():
    obsdim=35
    #return vrep.get_obstacles_positions()
    Obst = []
    rect = None
    for obstacle in vrep.get_obstacles_positions():
        centerx = vrep.map_value(obstacle, flag='Vrep2Py')[0] - obsdim / 2
        centery = vrep.map_value(obstacle, flag='Vrep2Py')[1] - obsdim / 2
        obstacle = (centerx, centery)
        rect = pygame.Rect(obstacle, (obsdim, obsdim))
        Obst.append(rect)
    return Obst
def get_dynamic_obstacles():  # đã điều chỉnh hệ toạ độ đúng 
    dynamic_pos = vrep.get_dynamic_obstacles_position()
    pos = []
    for obs in dynamic_pos:
        centerx = vrep.map_value(obs, flag='Vrep2Py')[1] 
        centery = vrep.map_value(obs, flag='Vrep2Py')[0]
        pos.append((centerx, centery))
    return pos
# Assume:
#  Initialize T with xa, Qr, Qs
def auto_make_goal(rt_rrt_star, limit_time=5, init_time=0):
    if time.time() - init_time > limit_time and time.time() - init_time < limit_time+5 and rt_rrt_star.close(rt_rrt_star.agent, tree_root):
        rt_rrt_star.goal = (525,140)
        speed=-1
    elif time.time() - init_time > limit_time+5 and time.time() - init_time < limit_time+10 and rt_rrt_star.close(rt_rrt_star.agent, tree_root):
        rt_rrt_star.goalFlag = False
        #rt_rrt_star.restarting = True
        rt_rrt_star.goal = (100,140)
        speed =1
    elif time.time() - init_time > limit_time+10 and time.time() - init_time < limit_time+15 and rt_rrt_star.close(rt_rrt_star.agent, tree_root):
        rt_rrt_star.goalFlag = False
        #rt_rrt_star.restarting = True
        rt_rrt_star.goal = (100,525)
        speed = -1
    elif time.time() - init_time >limit_time+15 and rt_rrt_star.close(rt_rrt_star.agent, tree_root):
        rt_rrt_star.goalFlag = False
        #rt_rrt_star.restarting = True
        rt_rrt_star.goal = (525,525)
        init_time = time.time()
        speed = 1
    return speed
if __name__ == '__main__':
    # print("Bắt đầu chương trình...")
    # sim.simxFinish(-1)
    # if clientID != -1:
        # print("Đã kết nối với remote API server")
    # else:
        # print("Kết nối không thành công!")
        # sys.exit("Không thể kết nối")
    # start = vrep.map_value((vrep.get_position(robot_position_handle)[1], vrep.get_position(robot_position_handle)[0]), flag='Vrep2Py')
    # goal  = vrep.map_value((vrep.get_position(goal_handle)[1], vrep.get_position(goal_handle)[0]), flag='Vrep2Py')
    # print(f"\nVị trí tác nhân: {start} -- Vị trí đích: {goal}\n")
    # obs = get_obstacles() # đối tượng là hình chữ nhật
    # dynamic_obs = get_dynamic_obstacles() # đối tượng là toạ độ chưa mapvalue
    OBS =  [
    pygame.Rect(89, 198, 35, 35),
    pygame.Rect(194, 62, 35, 35),
    pygame.Rect(177, 327, 35, 35),
    pygame.Rect(233, 330, 35, 35),
    pygame.Rect(222, 204, 35, 35),
    pygame.Rect(548, 338, 35, 35),
    pygame.Rect(393, 338, 35, 35),
    pygame.Rect(315, 524, 35, 35),
    pygame.Rect(398, 97, 35, 35),
    pygame.Rect(399, 169, 35, 35),
    pygame.Rect(390, 230, 35, 35),
    pygame.Rect(318, 471, 35, 35),
    pygame.Rect(132, 563, 35, 35),
    pygame.Rect(110, 324, 35, 35),
    pygame.Rect(21, 194, 35, 35),
    pygame.Rect(134, 21, 35, 35),
    pygame.Rect(36, 557, 35, 35),
    pygame.Rect(393, 281, 35, 35),
    pygame.Rect(252, 21, 35, 35),
    pygame.Rect(194, 561, 35, 35),
    pygame.Rect(149, 201, 35, 35),
    ]
    start = (300,300)#(56,80)
    goal = None # (525,140)

    mapdimenstions = (600,600)
    obsdim = 30
    # Thiết lập biến môi trường SDL để đặt vị trí của cửa sổ
    os.environ['SDL_VIDEO_WINDOW_POS'] = f"{700},{100}"
    pygame.init()
    rt_rrt_star = RT_RRT_star(agent=start, goal=goal, obstacles=OBS, mapdimentions=mapdimenstions)
    rt_rrt_star.divide_grid(numberofsquares=225)
    rt_rrt_star.add_node2grid_pos(0)
    # *
    rt_rrt_star.dynamic_obstacles = [[366, 346], [468, 130], [504, 460], [300,150], [300,500], [170,300]]
    speed = 1
    #*
    
    def draw_grid(object):

        grid_copy = object.grid.copy()
        j=0
        for row in grid_copy:
            j+=1
            i=j%2
            for squares in row:
                pygame.draw.rect(object.surface, (220,220,220), squares, i%2)  # Vẽ hình chữ nhật lên bề mặt
                i+=1
    draw_grid(rt_rrt_star)
    init_time = time.time()
    pre_second=-1
    while(True):
        print('Updating ...')
        rt_rrt_star.Update_tree() # Update xgoal, xa, Xfree and Xobs
        start_time = time.time() 
        ellapsed_time = 0
        while ellapsed_time<0.1:
            rt_rrt_star.Expansion_and_Rewiring()
            end_time = time.time()
            ellapsed_time = end_time - start_time
        tree_root = rt_rrt_star.x[rt_rrt_star.root], rt_rrt_star.y[rt_rrt_star.root]
        for event in pygame.event.get():
            if event.type == pygame.MOUSEBUTTONDOWN:
                rt_rrt_star.goal = event.pos  # Lưu tọa độ chuột vào goal khi nhấn chuột
                rt_rrt_star.goalFlag = False
                #speed = -speed
        current_second = int(time.time())
        if current_second%15==0 and current_second!=pre_second:
            speed=-speed 
            pre_second = current_second   
            #rt_rrt_star.restarting = True
        if not rt_rrt_star.goal:
            continue
        path_nodes = rt_rrt_star.Path_planning()
        print(f"path_nodes :{path_nodes}")
        tree_root = rt_rrt_star.x[rt_rrt_star.root], rt_rrt_star.y[rt_rrt_star.root]
        #print(f'Gốc cây: {tree_root}')
        if not path_nodes:
            continue
        if len(path_nodes)>1 and rt_rrt_star.close(rt_rrt_star.agent, tree_root):
            #print("1")
            rt_rrt_star.restarting = True
            rt_rrt_star.parent[rt_rrt_star.root] = path_nodes[1] # cập nhập parent của gốc cũ là gốc mới
            rt_rrt_star.root = path_nodes[1] # cập nhập gốc mới
            rt_rrt_star.parent[rt_rrt_star.root] = path_nodes[1] # cập nhập parent của gốc mới là gốc mới
        #Move the agent toward x0 for a limited time  
        time_move = time.time()
        #print(f'neighbor radius {rt_rrt_star.d}')
        while time.time()-time_move < 0.001:
            rt_rrt_star.Move(rt_rrt_star.root,1.5)
            # print("moving...")
