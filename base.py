import random
import math
import time
from shapely.geometry import Point
from shapely.affinity import scale, rotate
import pygame
dem = 0

class RRTmap:
    def __init__(self, start, goal, mapdimensions, obsdim, obsnum, Mapwindowname='RRT path planning'):
        self.start = start
        self.goal = goal
        self.mapdimensions = mapdimensions
        self.maph, self.mapw = self.mapdimensions

        self.Mapwindowname = Mapwindowname
        pygame.display.set_caption(self.Mapwindowname)
        self.map = pygame.display.set_mode((self.mapw, self.maph)) #kích thước map
        self.map.fill((255, 255, 255))
        self.nodeRadian = 2
        self.nodeThickness = 0
        self.edgeThickness = 1

        self.obstacles = []
        self.obsdim = obsdim
        self.obsnum = obsnum

        self.grey = (70, 70, 70)
        self.blue = (0, 0, 255)
        self.green = (0, 255, 0)
        self.red = (255, 0, 0)
        self.white = (255, 255, 255)

    def drawmap(self, obstacles):
        # vẽ điểm bắt đầu và điểm đích
        pygame.draw.circle(self.map, self.green, self.start, self.nodeRadian + 10, 0)
        pygame.draw.circle(self.map, self.red, self.goal, self.nodeRadian + 20, 1)
        # Vẽ các trướng ngại vật hình vuông
        self.drawObs(obstacles)

    #  Vẽ các điểm đỏ(node) đại diện cho đường đi từ đích về điểm bắt đầu
    def drawpath(self, coords):
        for coord in coords:
            pygame.draw.circle(self.map, self.red, coord, self.nodeRadian + 3, self.nodeThickness)

    def drawObs(self, obstacles):
        obstacleslist = obstacles.copy()
        while len(obstacleslist) > 0:  # vẽ các hình chữ nhật bên trong map đã tạo(nền trắng)
            obstacle = obstacleslist.pop(0)
            pygame.draw.rect(self.map, self.grey, obstacle)


class RRTgraph:
    def __init__(self, start, goal, mapdimensions, obsdim, obsnum, speed=0):
        
        self.start = start
        self.goal = goal
        self.mapdimensions = mapdimensions
        self.maph, self.mapw = self.mapdimensions
        self.x = []
        self.y = []
        self.parent = []
        self.cost = []
        self.cost_2 = 0
        self.speed = speed
        # initialize the tree
        (x, y) = start
        self.x.append(x)
        self.y.append(y)
        self.parent.append(0)
        self.cost.append(0)
        # the square obstacles
        self.obstacles = []
        self.obsdim = obsdim
        self.obsnum = obsnum
        self.ellip = None
        self.angle = 0
        self.surface = None
        self.r1 = 0
        self.r2 = 0
        self.center = None
        self.pre_cost = float('inf')
        # path
        self.goalstate = None
        self.goalFlag = False
        self.path = []
        self.error = False

    def makerandomRect(self):
        uppercornerx = int(random.uniform(0, self.mapw - self.obsdim))
        uppercornery = int(random.uniform(0, self.maph - self.obsdim))
        return uppercornerx, uppercornery

    def getTrueObs(self, obs, size=-30):
        TOBS = []
        for ob in obs:
            TOBS.append(ob.inflate(size, size))
        return TOBS

    def reset(self, all_=False):
        if not all_:
            n = len(self.x) - 1
            self.x.pop(n)
            self.y.pop(n)
            self.parent.pop(n)
            self.cost.pop(n)
        else:
            self.x = []
            self.y = []
            self.parent = []
            self.cost = []
            (x, y) = self.start
            # initialize the tree
            self.x.append(x)
            self.y.append(y)
            self.parent.append(0)
            self.cost.append(0)


    def makeObs(self):

        obs = []
        for i in range(0, self.obsnum):
            rect = None
            startgoalcol = True
            while startgoalcol:
                upper = self.makerandomRect()
                rect = pygame.Rect(upper, (self.obsdim,     self.obsdim))
                if rect.collidepoint(self.start) or rect.collidepoint(self.goal):
                    startgoalcol = True
                else:
                    startgoalcol = False
            obs.append(rect)
        self.obstacles = obs.copy()
        return obs

    def add_node(self, n, x, y):
        self.x.insert(n, x)
        self.y.append(y)

    def remove_node(self, n):
        self.x.pop(n)
        self.y.pop(n)

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

    def waypoints2path(self, num=5):
        oldpath = self.getPathcoords()
        path = []
        for i in range(0, len(self.path) - 1):
            # print(i)
            if i >= len(self.path):
                break
            x1, y1 = oldpath[i]
            x2, y2 = oldpath[i + 1]
            # print('---------')
            # print((x1, y1), (x2, y2))
            for i in range(0, num):
                u = i / num
                x = int(x2 * u + x1 * (1 - u))
                y = int(y2 * u + y1 * (1 - u))
                path.append((x, y))
                # print((x, y))

        return path
    
    #  tạo ngẫu nhiên các node
    def sample_envir(self):
        x = int(random.uniform(0, self.mapw))
        y = int(random.uniform(0, self.maph))
        return x, y

    def nearest(self, n):  # tìm node gần nhất
        dmin = self.distance(0, n)
        nnear = 0  # Lần đầu tiên là điểm thứ 0 -điểm start
        for i in range(0, n):
            if self.distance(i, n) < dmin:
                dmin = self.distance(i, n)
                nnear = i
        return nnear
    def nearest2(self, point):
        dmin = math.dist((self.x[0], self.y[0]), point)
        nnear = 0  # Lần đầu tiên là điểm thứ 0 -điểm start
        for i in range(0, self.number_of_nodes()):
            d_new = math.dist((self.x[i], self.y[i]), point)
            if  d_new < dmin:
                dmin = d_new
                nnear = i
        return nnear
    
    def isFree(self):  # kiểm tra xem điểm ngẫu nhiên đó có va vào khối vật cản không?
        n = self.number_of_nodes() - 1
        (x, y) = (self.x[n], self.y[n])
        obstacles = self.obstacles.copy()
        while len(obstacles) > 0:
            rect = obstacles.pop(0)
            if rect.collidepoint(x, y):
                self.remove_node(n)
                return False
        return True

    def crossObstacle_forstep(self, x1, x2, y1, y2, step_max=35):

        obs = self.obstacles.copy()
        dx = float(x2) - float(x1)
        dy = float(y2) - float(y1)
        theta = math.atan2(dy, dx)
        x, y = int(x1 + step_max * math.cos(theta)), int(y1 + step_max * math.sin(theta))

        while len(obs) > 0:
            rect = obs.pop(0)
            for i in range(0, 101):
                u = i / 100
                x_ = x1 * u + x * (1 - u)
                y_ = y1 * u + y * (1 - u)
                if rect.collidepoint(x_, y_):  #  kiểm tra điểm có chạm hay nằm trong hình chữ nhật hay k?
                    return True
        return False

    def crossObstacle(self, x1, x2, y1, y2):
        obs = self.obstacles.copy()

        while len(obs) > 0:
            rect = obs.pop(0)
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
        
    def get_random_point_in_ellipse(self, cbest):
        focus1 = self.start
        focus2 = self.goal
       

        # Tính trung điểm giữa hai tiêu điểm (tâm của ellipse)
        center_x = (focus1[0] + focus2[0]) / 2
        center_y = (focus1[1] + focus2[1]) / 2
        center = Point(center_x, center_y)
        self.center = (center_x, center_y)

        # Tính khoảng cách giữa hai tiêu điểm
        focal_distance = math.dist(focus1, focus2)  # cmin
        major_axis_length =  cbest  # cbest
        self.r1 = cbest/2
        # Kiểm tra điều kiện tạo ellipse
        if major_axis_length < focal_distance:
            raise ValueError("Chiều dài trục lớn phải lớn hơn hoặc bằng khoảng cách giữa hai tiêu điểm.")

        # Tính trục nhỏ dựa trên trục lớn và khoảng cách giữa hai tiêu điểm
        minor_axis_length = math.sqrt(major_axis_length**2 - focal_distance**2)
        self.r2 = minor_axis_length/2
        # Tạo ellipse bằng cách tạo một đường tròn tại tâm và biến đổi thành ellipse
        ellipse = center.buffer(1)  # Tạo đường tròn bán kính 1 tại tâm
        ellipse = scale(ellipse, major_axis_length / 2, minor_axis_length / 2)  # Biến đổi thành ellipse
        
        # Xác định góc giữa đường nối hai tiêu điểm và trục ngang
        angle = math.degrees(math.atan2(focus2[1] - focus1[1], focus2[0] - focus1[0]))
        self.angle = angle
        # Xoay ellipse theo góc này
        ellipse = rotate(ellipse, angle, origin=center)
        self.ellip = ellipse
        # Lấy bounding box của ellipse
        min_x, min_y, max_x, max_y = ellipse.bounds

        while True:
            # Tạo một điểm ngẫu nhiên trong bounding box của ellipse
            random_x = max(0, random.uniform(min_x, max_x))
            random_y = max(0, random.uniform(min_y, max_y))
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
        if Pr > 1 - alpha:  #  random in the line to goal 
            xclosest = self.nearest2(self.goal)
            x = float(random.uniform(min(self.x[xclosest], self.goal[0]), max(self.x[xclosest], self.goal[0])))
            y = float(random.uniform(min(self.y[xclosest], self.goal[1]), max(self.y[xclosest], self.goal[0])))
            path = []
            numpoints = 5
            for i in range(0, numpoints):
                u = i / 5
                x = int(self.goal[0] * u + self.x[xclosest] * (1 - u))
                y = int(self.goal[1] * u + self.y[xclosest] * (1 - u))
                path.append((x, y))
            ran = int(random.uniform(1, 4))
            xrand = path[ran]
        else:
            #Pr <= (1-alpha)/beta:
            xrand = self.sample_envir()
    # Ellipsis
        return xrand
    
    def step(self, nnear, nrand, step_max=35):
        d = self.distance(nnear, nrand)
        x1, y1 = self.x[nnear], self.y[nnear]
        x2, y2 = self.x[nrand], self.y[nrand]
        if self.goalFlag and self.ellip is not None:
            self.obstacles = self.get_obs_in_ellipsis()
        c_new = float('inf')
        if d <= step_max:
            if not self.crossObstacle(x1, x2, y1, y2):
                self.add_edge(nnear, nrand)
                cost_ = self.cost[nnear] + self.distance(nnear, nrand)
                self.cost.append(cost_)
            else:
                self.remove_node(nrand)
            if abs(x2 - self.goal[0]) < step_max and abs(y2 - self.goal[1]) < step_max and not self.crossObstacle(x2, self.goal[0], y2, self.goal[1]):
                
                
                if not self.goalFlag:
                    self.add_node(nrand+1, self.goal[0], self.goal[1])
                    self.add_edge(nrand, nrand+1)
                    self.goalstate = nrand+1
                    self.goalFlag = True
                    cost_ = self.cost[nrand] + self.distance(nrand, nrand+1)  
                    self.cost.append(cost_)
                    
                else:
                    c_new = self.cost[nrand] + self.distance(nrand, self.goalstate)
                    if c_new<self.cost[self.goalstate]:
                        self.parent[self.goalstate] = nrand
                        self.cost[self.goalstate] = c_new
                       


        # nếu d > stepmax thì đã thêm node ở phía trước rồi
        else :

            dx = float(x2) - float(x1)
            dy = float(y2) - float(y1)
            theta = math.atan2(dy, dx)
            self.remove_node(nrand)  # xoá node này do lớn hơn step ( là xoá x và y)
            x, y = int(x1 + step_max * math.cos(theta)), int(y1 + step_max * math.sin(theta))
            if not self.crossObstacle(x1, x, y1, y):
                self.add_node(nrand, x, y)
                self.add_edge(nnear, nrand)
                cost_ = self.cost[nnear] + self.distance(nnear, nrand)
                self.cost.append(cost_)
            if abs(x - self.goal[0]) < step_max and abs(y - self.goal[1]) < step_max and not self.crossObstacle(x, self.goal[0], y, self.goal[1]):
                
                
                if not self.goalFlag:
                # return False
                # if self.goalstate is not None
                    self.add_node(nrand+1, self.goal[0], self.goal[1])
                    self.add_edge(nrand, nrand+1)
                    self.goalstate = nrand+1
                    self.goalFlag = True
                    cost_ = self.cost[nrand] + self.distance(nrand, nrand+1)
                    self.cost.append(cost_)
                    
                else:
                    c_new = self.cost[nrand] + self.distance(nrand, self.goalstate)
                    if c_new<self.cost[self.goalstate]:
                        self.parent[self.goalstate] = nrand
                        self.cost[self.goalstate] = c_new



    def path_to_goal(self): # trạng thái cờ true khi đến đích, và cập nhập các node trên path

        if self.goalFlag:
            self.path = []
            self.path.append(self.goalstate)
            # time.sleep(0.02)
            # tìm node cha của điểm goal
            newPos = self.parent[self.goalstate]  # vị trí node cha
            # time.sleep(0.02)
            # vòng lặp tìm node cha cho đến khi đến điểm start
            while newPos != 0:
                self.path.append(newPos)
                newPos = self.parent[newPos]
            self.path.append(0)
        return self.goalFlag

    def getPathcoords(self): # cập nhập toạ độ 
        pathcoords = []
        for node in self.path:
            # if len(self.x) < node:
            #     continue
            (x, y) = self.x[node], self.y[node]
            pathcoords.append((x, y))

        return pathcoords
    

    
    def getfinalcost(self):
        final_cost = 0.0
        for i in range(len(self.path) - 1):
            final_cost += self.distance(self.path[i], self.path[i + 1])
        if final_cost != 0:
            self.cost_2 = final_cost
        return final_cost

    def bias(self):
        # n = self.number_of_nodes()
        # self.add_node(n, ngoal[0], ngoal[1])  #  thêm x và y vào list x v
        # nnear = self.nearest(n)
        # self.step(nnear, n)
        # self.connect(nnear, n)
        xclosest = self.nearest2((self.goal[0], self.goal[1]))
        x = float(random.uniform(min(self.x[xclosest], self.goal[0]), max(self.x[xclosest], self.goal[0])))
        y = float(random.uniform(min(self.y[xclosest], self.goal[1]), max(self.y[xclosest], self.goal[0])))
        path = []
        numpoints = 10
        for i in range(0, numpoints):
            u = i / numpoints
            x = int(self.goal[0] * u + self.x[xclosest] * (1 - u))
            y = int(self.goal[1] * u + self.y[xclosest] * (1 - u))
            path.append((x, y))
        ran = int(random.uniform(1, 4))
        xrand = path[ran]
        n = self.number_of_nodes()
        self.add_node(n, xrand[0], xrand[1])  #  thêm x và y vào list x v
        nnear = self.nearest(n)
        self.step(nnear, n)
        return self.x, self.y, self.parent

    def expand(self):
        n = self.number_of_nodes()
        if self.goalFlag:
            cbest = self.getfinalcost()
            if cbest<self.pre_cost:
                x, y = self.get_random_point_in_ellipse(cbest=cbest)
            else :
                x, y = self.get_random_point_in_ellipse(cbest=self.pre_cost)
            self.pre_cost = cbest
        else :
            x, y = self.sample_envir()
            #x, y = self.sample()
        self.add_node(n, x, y)
        
        xnearest = self.nearest(n)
        self.step(xnearest, n)
        # self.connect(xnearest, n)

        return self.x, self.y, self.parent

    def rewire(self, map_, new_node, neighbors):
        if not neighbors:
            return

        # STEP 1: Tìm neighbor có chi phí nhỏ nhất đến new_node
        cost_min = float('inf')  # Khởi tạo với giá trị vô cực
        best_neighbor = None
        x_new, y_new = self.x[new_node], self.y[new_node]

        # Duyệt qua các neighbors để tìm đường đi có chi phí nhỏ nhất
        # for i in neighbors:
            # x_neighbor, y_neighbor = self.x[i], self.y[i]
            # if not self.crossObstacle(x_new, x_neighbor, y_new, y_neighbor):
                # new_cost = self.cost[i] + self.distance(new_node, i)
                # if new_cost < cost_min:
                    # cost_min = new_cost
                    # best_neighbor = i  # Lưu lại neighbor tốt nhất

        for i in neighbors:
            x_neighbor, y_neighbor = self.x[i], self.y[i]
            new_cost = self.cost[i] + self.distance(new_node, i)
            if new_cost < cost_min and not self.crossObstacle(x_new, x_neighbor, y_new, y_neighbor):
                    cost_min = new_cost
                    best_neighbor = i  # Lưu lại neighbor tốt nhất

        if best_neighbor is not None:
            # xoá đường line cũ tới nốt mới
            pygame.draw.line(map_, (255, 255, 255), (self.x[-1], self.y[-1]),
                             (self.x[self.parent[-1]], self.y[self.parent[-1]]), 1)
            time.sleep(self.speed)
            # Cập nhật parent của new_node
            self.parent[new_node] = best_neighbor
            self.cost[new_node] = cost_min
            # Vẽ đường nối từ best_neighbor đến new_node
            pygame.draw.line(map_, (0, 0, 255), (self.x[best_neighbor], self.y[best_neighbor]), (x_new, y_new), 1)
            time.sleep(self.speed)
            pygame.display.update()
        
        # STEP 2: Tái cấu trúc cây
        for i in neighbors:
            # Tính toán chi phí mới nếu đi qua new_node đến neighbor i
            if i == best_neighbor:
                continue
            x_neighbor, y_neighbor = self.x[i], self.y[i]
            if not self.crossObstacle(x_new, x_neighbor, y_new, y_neighbor):
                new_cost = self.cost[new_node] + self.distance(new_node, i)
                
                
                
                # Nếu chi phí mới nhỏ hơn chi phí hiện tại của các neighbor, cập nhật parent
                if new_cost < self.cost[i]:
                    # xoá đường kết nối neighbor cũ
                    pygame.draw.line(map_, (255, 255, 255), (x_neighbor, y_neighbor),
                                     (self.x[self.parent[i]], self.y[self.parent[i]]), 1)
                    time.sleep(self.speed)
                    # kết nối neighbor đó với new_node
                    self.parent[i] = new_node
                    self.cost[i] = new_cost
                    # Vẽ đường nối từ new_node đến neighbor i
                    pygame.draw.line(map_, (0, 0, 255), (x_new, y_new), (x_neighbor, y_neighbor), 1)
                    time.sleep(self.speed)
                    pygame.display.update()

    # Tìm kiếm các node lân cận
    def near_neighbors(self, new_node, dmax=50,kmax=5):
        # # Số chiều không gian (2D)
        # d = 2
        #
        # # Hằng số gamma ban đầu có thể thử là tỷ lệ với kích thước bản đồ
        # gamma = 100  # Có thể thử từ 50-200 tùy vào kết quả
        n = self.number_of_nodes()
        radius = math.sqrt(self.maph*self.mapw*kmax/(math.pi*n))
        if radius<=35 :
            radius = 35
        # if radius>=150:
            # radius = 150
        #
        # radius = min(gamma * (math.log(new_node) / new_node) ** (1 / d), dmax)
        neighbors = []  # tìm lại neighbor khi gọi lại
        for i in range(len(self.x)):  # quét tất cả các node
            if self.distance(new_node, i) < radius:
                neighbors.append(i)
                if len(neighbors)>kmax:
                    return neighbors
        return neighbors
    
    def get_obs_in_ellipsis(self):
        # Lấy bounding box của ellipse
        #ellipse = self.ellip
        if self.ellip is None:
            return
        obs = []
        for ob in self.obstacles:
            points = Point(ob.topleft), Point(ob.topright), Point(ob.bottomleft), Point(ob.bottomright)
            for point in points:
                if self.ellip.contains(point):
                    obs.append(ob)
                    break
        #print(f'\nHave {len(obs)} obstacles in ellipse\n')
        return obs