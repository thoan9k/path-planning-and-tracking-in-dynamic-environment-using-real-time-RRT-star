import time

import pygame
import math

# dt = 0
# lasttime = pygame.time.get_ticks()
lx = 0.5
ly = 0.5


def atan2_0_to_2pi(y, x):
    angle = math.atan2(y, x)  # Kết quả ban đầu trong khoảng [-π, π]
    if angle < 0:
        angle += 2 * math.pi  # Đưa kết quả về khoảng [0, 2π]
    return angle


def dist(pos1, pos2):
    (x1, y1) = pos1
    (x2, y2) = pos2
    dx = (float(x1) - float(x2)) ** 2
    dy = (float(y1) - float(y2)) ** 2
    return (dx + dy) ** 0.5


class Envir:
    def __init__(self, dimentions):
        # colors
        self.green = (0, 255, 0)
        self.blue = (0, 0, 255)
        self.red = (255, 0, 0)
        self.yelow = (255, 255, 0)
        self.black = (0, 0, 0)
        self.white = (255, 255, 255)
        # map dimentions
        self.width = dimentions[0]
        self.height = dimentions[1]
        # window display
        pygame.display.set_caption("Informed RRT*")
        self.map = pygame.display.set_mode((self.width, self.height))
        self.map.fill(self.white)
        # text
        self.font = pygame.font.Font('freesansbold.ttf', 30)
        self.text = self.font.render('default', True, self.white, self.black)
        self.textRect = self.text.get_rect()
        self.textRect.center = (dimentions[0] - 1130, dimentions[1] - 20)
        # trail
        self.trail_set = []

    def draw_trail(self, pos):
        for i in range(0, len(self.trail_set) - 1):
            pygame.draw.line(self.map, self.green, (self.trail_set[i][0], self.trail_set[i][1]),
                             (self.trail_set[i + 1][0], self.trail_set[i + 1][1]), 1)

        if self.trail_set.__sizeof__() > 30000:
            self.trail_set.pop(0)
        self.trail_set.append(pos)

    def write_info(self, vl, vr, theta):
        txt = f"v={(vl + vr) / 2}  vl={vl}  vr={vr}  theta={int(math.degrees(theta))}"
        self.text = self.font.render(txt, True, self.black, self.white)
        self.map.blit(self.text, self.textRect)

    def draw_XYaxis_frame(self, pos, rotation):
        n = 50
        (x_pos, y_pos) = pos
        # Added "-rotation" to hold the axis fixed on the mobile car
        x_axis = (x_pos + n * math.cos(rotation), y_pos + n * math.sin(rotation))
        y_axis = (x_pos + n * math.cos(rotation + math.pi / 2)), y_pos + n * math.sin(rotation + math.pi / 2)
        pygame.draw.line(self.map, self.yelow, pos, x_axis, 3)
        pygame.draw.line(self.map, self.blue, pos, y_axis, 3)


class PIDcontroller:
    # def __init__(self, start, goal, R_=0.02, L_=0.35,
    #              kP=1.0, kI=0.01, kD=0, kH=5, dt=0.1, v=1.0,
    #              arrive_distance=0.01):
    def __init__(self, start, goal, R=0.05, Lx=0.15, Ly=0.15,
                 kP=1.0, kI=0.01, kD=0, kH=5, dt=0.1, v=1.0,
                 arrive_distance=0.01):
        self.current = start
        self.goal = goal
        self.R = R # in meter 
        self.lx = Lx  # in meter
        self.ly = Ly
        self.E = [0, 0]  # Cummulative error
        self.old_e = [0, 0]  # Previous error

        self.Kp = kP
        self.Ki = kI
        self.Kd = kD
        self.kh = kH
        self.desiredV = v
        self.dt = dt  # in second
        self.arrive_distance = arrive_distance

    def PIDcalculate(self):
        # difference distance

        if len(self.goal) == 2:
            x_goal, y_goal = self.goal
        else:
            x_goal, y_goal, _ = self.goal
        x_cur, y_cur, theta_cur = self.current
        dx = x_goal - x_cur
        dy = y_goal - y_cur
        # desired angle
        if len(self.goal) == 2:
            goal_theta = math.atan2(dy, dx)
        else:
            goal_theta = self.goal[2]
        # Error between the goal and current position of robot
        errorx = dx
        errory = dy
        alpha = goal_theta - theta_cur
        
        errorxR = math.cos(theta_cur)*errorx +math.sin(theta_cur)*errory
        erroryR = -math.sin(theta_cur)*errorx +math.cos(theta_cur)*errory

        
        if abs(alpha) > math.pi:
            if alpha > 0:
                alpha = -(2*math.pi - alpha)
            else:
                alpha = 2*math.pi + alpha
        print(f"alpha = {alpha}")
        e_Px = errorxR
        e_Ix= self.E[0] + errorxR  # lỗi tích luỹ
        e_Dx = errorxR - self.old_e[0] # lỗi vận tốc

        e_Py = erroryR
        e_Iy = self.E[1] + erroryR
        e_Dy = erroryR - self.old_e[1]
        # this PID is obtained PID controller for the linear velocity and P controller for the angular velocity
        vx = self.Kp * e_Px + self.Ki * e_Ix + self.Kd * e_Dx
        vy = self.Kp * e_Py + self.Ki * e_Iy + self.Kd * e_Dy
        wz = self.kh * alpha
        # inverse kinematic equation
        w1 = (vx - vy -(self.lx +self.ly)*wz)/self.R
        w2 = (vx + vy +(self.lx+self.ly)*wz)/self.R
        w3 = (vx + vy -(self.lx+self.ly)*wz)/self.R
        w4 = (vx - vy +(self.lx+self.ly)*wz)/self.R
        v = (vx**2 + vy**2)**0.5
        print(f"V={v} Wz= {wz}")
        self.E[0] = self.E[0] + errorxR
        self.old_e[0] = errorxR

        self.E[1] = self.E[1] + erroryR
        self.old_e[1] = erroryR
        
        return w1, w2, w3, w4

    def isArrived(self, dstar=0.3):
        x_goal, y_goal = self.goal
        x_cur, y_cur, _ = self.current
        # print("Arrive check:", str(abs(x_cur - x_goal)),
        #       str(abs(y_cur - y_goal)))
        current_state = [x_cur, y_cur]
        goal_state = [x_goal, y_goal]
        self.arrive_distance = dstar
        distance_err = dist(goal_state, current_state)
        if distance_err < self.arrive_distance:
            return True
        else:
            return False


class Robot:
    def __init__(self, startpos, carimg, path=None):
        self.mm2pixel = 3779.52
        # robot dims
        self.x = startpos[0]
        self.y = startpos[1]
        # graphics
        self.img = pygame.image.load(carimg)
        # Resize image using pygame's transform.scale
        self.img = pygame.transform.scale(self.img, (self.img.get_width() / 30, self.img.get_height() / 30))
        self.rotated = self.img  # avoid damage img
        self.rect = self.rotated.get_rect(center=(self.x, self.y))

        self.b = self.img.get_height() / 30

        # self.vl = 0.001 * self.mm2pixel
        # self.vr = 0.001 * self.mm2pixel
        self.vl = 0
        self.vr = 0
        self.maxspeed = 0.02 * self.mm2pixel
        self.minspeed = -0.02 * self.mm2pixel
        self.a = self.img.get_width() / 50
        self.u = 1
        self.w = 0
        self.path = path
        self.waypoint = len(self.path) - 1
        self.theta = 0
        if self.theta < 0:
            self.theta += 2 * math.pi

    def draw(self, map_):
        map_.blit(self.rotated, self.rect)

    def move(self, dt):
        # forward kinematic equation
        self.x += (self.u * math.cos(self.theta) - self.a * math.sin(self.theta) * self.w) * dt
        self.y += (self.u * math.sin(self.theta) + self.a * math.cos(self.theta) * self.w) * dt
        self.theta += self.w * dt

        self.rotated = pygame.transform.rotozoom(self.img, math.degrees(-self.theta), 1)
        self.rect = self.rotated.get_rect(center=(self.x, self.y))

        self.follow_path()

    def follow_path(self):
        target = self.path[self.waypoint-1]
        delta_x = target[0] - self.x
        delta_y = target[1] - self.y

        if math.atan2(delta_y, delta_x) < 0:
            self.theta = math.atan2(delta_y, delta_x) + 2 * math.pi
        else:
            self.theta = math.atan2(delta_y, delta_x)

        # inverse kinematic equation
        self.u = delta_x * math.cos(self.theta) + delta_y * math.sin(self.theta)
        self.w = (-1 / self.a) * math.sin(self.theta) + (1 / self.a) * math.cos(self.theta)

        if dist((self.x, self.y), self.path[self.waypoint]) <= 35:
            self.waypoint -= 1  # move to next point

        if self.waypoint <= 0:
            self.waypoint = 0

    def caculate_velocity(self, target):

        x_target, y_target = target

        dx = x_target - self.x
        dy = y_target - self.y

        # # tính toán sai số khoảng cách
        # distance = self.dist(target, (self.x, self.y))

        # # tính toán góc mục tiêu
        # if math.atan2(dy, dx) < 0:
        #     theta_target = math.atan2(dy, dx) + 2 * math.pi
        # else:
        #     theta_target = math.atan2(dy, dx)

        # sai số góc
        self.theta = atan2_0_to_2pi(dy, dx)
        # theta_diff = theta_target - self.theta

        self.u = math.cos(self.theta) * dx + math.sin(self.theta) * dy + lx * math.sin(
            atan2_0_to_2pi(lx * (x_target - self.x), 1))
        self.w = (-1 / self.a) * math.sin(self.theta) * dx + (-1 / self.a) * math.cos(self.theta) * dy + lx * math.sin(
            atan2_0_to_2pi(ly * (y_target - self.y), 1))

#
# # initialization
# pygame.init()
#
# # running or not
# running = True
#
# # dimentions
# dims = (1200, 600)
#
# # map
# envir = Envir(dims)
#
# # mobile car
# startpos = (100, 200)
# mobile_car = Robot(startpos, "car_img.png", 0.01 * 3779.52)
# # simulation loop
# while running:
#     for event in pygame.event.get():
#         if event.type == pygame.QUIT:
#             running = False
#         mobile_car.move(event)
#     dt = (pygame.time.get_ticks() - lasttime) / 1000
#     lasttime = pygame.time.get_ticks()
#     pygame.display.update()
#     envir.map.fill(envir.white)
#     mobile_car.move()
#
#     envir.write_info(int(mobile_car.vl), int(mobile_car.vr), mobile_car.theta)
#     mobile_car.draw(envir.map)
#     envir.draw_trail((mobile_car.x, mobile_car.y))
#     envir.draw_XYaxis_frame((mobile_car.x, mobile_car.y), mobile_car.theta)
