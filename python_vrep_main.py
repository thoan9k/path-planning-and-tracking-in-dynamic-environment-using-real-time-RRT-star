import sys
import time
import base
import robot_follow_path
import sim
import math
import pygame
import os
# print("Bắt đầu chương trình...")
# sim.simxFinish(-1)

clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)  # Connect to CoppeliaSim
robot_position_handle = None
goal_handle = None
# if clientID != -1:
    # print("Đã kết nối với remote API server")
# else:
    # print("Kết nối không thành công!")
    # sys.exit("Không thể kết nối")
# 
# Main Objects
# errorcode, robot_position_handle = sim.simxGetObjectHandle(clientID, 'mecanum_robot', sim.simx_opmode_blocking)
# _, wheel1_handle = sim.simxGetObjectHandle(clientID, '/mecanum_robot/wheel1', sim.simx_opmode_blocking)
# _, wheel3_handle = sim.simxGetObjectHandle(clientID, '/mecanum_robot/wheel3', sim.simx_opmode_blocking)
# _, wheel4_handle = sim.simxGetObjectHandle(clientID, '/mecanum_robot/wheel4', sim.simx_opmode_blocking)
# _, wheel2_handle = sim.simxGetObjectHandle(clientID, '/mecanum_robot/wheel2', sim.simx_opmode_blocking)
# 
# _, obstacles_handle = sim.simxGetObjectHandle(clientID, 'obstacles', sim.simx_opmode_blocking)
# _, goal_handle = sim.simxGetObjectHandle(clientID, 'Goal', sim.simx_opmode_blocking)

#  =============================== control motor =================================================


def set_velocities(w1, w2, w3, w4):
    global wheel1_handle, wheel2_handle, wheel3_handle, wheel4_handle
    # if errorcode == sim.simx_return_ok:
    #     print("Lấy được handle của rightMotor")
    # else:
    #     print("Lỗi khi lấy handle của rightMotor:", errorcode)
    
    # Đặt vận tốc cho 4  bánh mecanum
    sim.simxSetJointTargetVelocity(clientID, wheel1_handle, w1, sim.simx_opmode_oneshot_wait)
    sim.simxSetJointTargetVelocity(clientID, wheel2_handle, w2, sim.simx_opmode_oneshot_wait)
    sim.simxSetJointTargetVelocity(clientID, wheel3_handle, w3, sim.simx_opmode_oneshot_wait)
    sim.simxSetJointTargetVelocity(clientID, wheel4_handle, w4, sim.simx_opmode_oneshot_wait)


def get_position(Object, index=2):
    _, position = sim.simxGetObjectPosition(clientID, Object, -1, sim.simx_opmode_blocking)
    # print(f'Vị trí hiện tại:{position} ')
    # # ========= Nhận và đặt hướng quay(chiều ngược chiều kim đồng hồ)=============================
    _, orientation = sim.simxGetObjectOrientation(clientID, Object, -1, sim.simx_opmode_blocking)
    # orientation[2] = orientation[2] / (2 * math.pi) * 360
    # print(f'Hướng hiện tại:{orientation[2] / (2 * math.pi) * 360}')
    if Object == robot_position_handle or Object == goal_handle:
        return position[0], position[1], orientation[index]
    else:
        return position[0], position[1]


def set_position(pos, theta_, object=robot_position_handle):
    if pos is not None and len(pos) == 2:
        pos.append(0)
    if pos is not None:
        sim.simxSetObjectPosition(clientID, object, -1, [pos[0], pos[1], pos[2]],
                                  sim.simx_opmode_oneshot)
        _, position = sim.simxGetObjectPosition(clientID, object, -1, sim.simx_opmode_blocking)
        print(f'Vị trí mới: {position} ')
    if theta_ is not None:
        sim.simxSetObjectOrientation(clientID, object, -1, [0, 0, theta_], sim.simx_opmode_oneshot)
        _, orientation = sim.simxGetObjectOrientation(clientID, object, -1, sim.simx_opmode_blocking)
        print(f'Hướng mới:{orientation[2] / (2 * math.pi) * 360}')


# ======================================== Draw and erase ====================================================
def create_line(pos1, pos2, color=(0, 1, 0)):
    # point1 = [0, 0, 0]  # Điểm bắt đầu
    # point2 = [2, 2, 0]  # Điểm kết thúc
    y1, x1 = pos1
    y2, x2 = pos2
    points = [x1, y1, 0, x2, y2, 0]
    # Gọi script để vẽ đường thẳng
    # Gọi script để vẽ đường thẳng
    emptyBuff = bytearray()
    sim.simxCallScriptFunction(
        clientID,
        "obstacles",  # Đảm bảo đối tượng này có script gắn với nó
        sim.sim_scripttype_childscript,  # Loại script là child script
        'createLine',  # Tên hàm trong script Lua
        color,  # Các tham số nguyên (không cần trong trường hợp này)
        points,  # Tham số kiểu float: bao gồm điểm bắt đầu và điểm kết thúc
        [],  # Tham số kiểu string (nếu có)
        emptyBuff,  # Tham số buffer (nếu có)
        sim.simx_opmode_blocking  # Chế độ blocking
    )


def create_point(pos1, color=(0, 1, 0), mode="normal"):
    y1, x1 = pos1
    point_ = [y1, x1, 0]
    emptyBuff = bytearray()
    sim.simxCallScriptFunction(
        clientID,
        "obstacles",  # Đảm bảo đối tượng này có script gắn với nó
        sim.sim_scripttype_childscript,  # Loại script là child script
        'createPoint',  # Tên hàm trong script Lua
        color,  # Các tham số nguyên (không cần trong trường hợp này)
        point_,  # Tham số kiểu float: bao gồm điểm bắt đầu và điểm kết thúc
        [mode],  # Tham số kiểu string (nếu có)
        emptyBuff,  # Tham số buffer (nếu có)
        sim.simx_opmode_blocking  # Chế độ blocking
    )


def create_circle(center, radius, num_points, color=(1, 0, 0)):
    angle_step = 2 * math.pi / num_points
    points = []

    for j in range(num_points + 1):
        angle = j * angle_step
        x = center[0] + radius * math.cos(angle)
        y = center[1] + radius * math.sin(angle)
        z = center[2]
        points.append([x, y, z])

    # Gửi các đoạn thẳng qua script của CoppeliaSim để vẽ đường tròn
    for j in range(len(points) - 1):
        x1, y1, z1 = points[j]
        x2, y2, z2 = points[j + 1]
        create_line((x1, y1), (x2, y2), color)


def remove_path():
    emptyBuff = bytearray()
    sim.simxCallScriptFunction(
        clientID,
        "obstacles",  # Đảm bảo đối tượng này có script gắn với nó
        sim.sim_scripttype_childscript,  # Loại script là child script
        'removeAllDrawings',  # Tên hàm trong script Lua
        [],  # Các tham số nguyên (không cần trong trường hợp này)
        [],  # Tham số kiểu float: bao gồm điểm bắt đầu và điểm kết thúc
        [],  # Tham số kiểu string (nếu có)
        emptyBuff,  # Tham số buffer (nếu có)
        sim.simx_opmode_blocking  # Chế độ blocking
    )


def remove_Allpoints():
    emptyBuff = bytearray()
    sim.simxCallScriptFunction(
        clientID,
        "obstacles",  # Đảm bảo đối tượng này có script gắn với nó
        sim.sim_scripttype_childscript,  # Loại script là child script
        'removeAllPoints',  # Tên hàm trong script Lua
        [],  # Các tham số nguyên (không cần trong trường hợp này)
        [],  # Tham số kiểu float: bao gồm điểm bắt đầu và điểm kết thúc
        [],  # Tham số kiểu string (nếu có)
        emptyBuff,  # Tham số buffer (nếu có)
        sim.simx_opmode_blocking  # Chế độ blocking
    )
#  ================================ Position of square obstacles ==========================================


def get_obstacles_positions():
    positions = []
    for j in range(4, 25):
        position = get_position(sim.simxGetObjectHandle(clientID, f'Cuboid{j}', sim.simx_opmode_blocking)[1])
        positions.append((position[1], position[0]))
        # print(position)
    return positions

def get_dynamic_obstacles_position():
    positions = []
    position = get_position(sim.simxGetObjectHandle(clientID, f'Dynamic_obstacles_', sim.simx_opmode_blocking)[1])
    positions.append((position[1], position[0]))
    position = get_position(sim.simxGetObjectHandle(clientID, f'Dynamic_obstacles_#0', sim.simx_opmode_blocking)[1])
    positions.append((position[1], position[0]))
    position = get_position(sim.simxGetObjectHandle(clientID, f'Dynamic_obstacles_#1', sim.simx_opmode_blocking)[1])
    positions.append((position[1], position[0]))
    return positions
#  ================================= Convert coordinates between Python and CoppeliaSim(Vrep)==========================
def map_value(x, in_min=(0, 0), in_max=(600, 600), out_min=(-2.5, -2.5), out_max=(2.5, 2.5), flag='Py2Vrep'):
    if flag == 'Vrep2Py':
        in_min = (-2.5, -2.5)
        in_max = (2.5, 2.5)
        out_min = (0, 0)
        out_max = (600, 600)
        return [int((x[0] - in_min[0]) * (out_max[0] - out_min[0]) / (in_max[0] - in_min[0]) + out_min[0]),
                int((x[1] - in_min[1]) * (out_max[1] - out_min[1]) / (in_max[1] - in_min[1]) + out_min[1])]
    if isinstance(x, int):
        return (x - in_min[0]) * (out_max[0] - out_min[0]) / (in_max[0] - in_min[0]) + out_min[0]
    return [(x[0] - in_min[0]) * (out_max[0] - out_min[0]) / (in_max[0] - in_min[0]) + out_min[0],
            (x[1] - in_min[1]) * (out_max[1] - out_min[1]) / (in_max[1] - in_min[1]) + out_min[1]]

# ======================================== Draw map or environment ====================================================
# that obtain obstacles, start point, goal point, nodes and branches of RRT* tree
# the INPUT include: position of start and goal, quantity and size of obstacles, maximum of node, dimension of map(2D)
# the OUTPUT are the optimal path from start to goal and the cost(distance) if necessary


def main(obs_, graph, map__):
    global start, goal, dimensions, obsdim, obsnum, iteration_max

    # tạo vật cản

    # vẽ vật cản
    map__.drawmap(obs_)
    iteration = 0
    t1 = time.time()
    oldcost = float('inf')
    oldpath = None

    # n_image = 0
    # # Lưu màn hình Pygame vào file BMP
    # pygame.image.save(map_.map, f"images/screenshot{n_image}.bmp")
    #
    # # Sử dụng Pillow để mở và chuyển file BMP sang PNG hoặc GIF
    # image = Image.open(f"images/screenshot{n_image}.bmp")
    # image.save(f"images/screenshot{n_image}.png")  # Lưu dưới dạng PNG
    # n_image += 1

    while iteration <= iteration_max:
        eslaped = time.time() - t1
        t1 = time.time()
        if eslaped > 10:
            raise
        if iteration % 10 == 0:
            n = graph.number_of_nodes()
            x, y, parent = graph.bias()
            pygame.draw.circle(map_.map, map_.red, (x[-1], y[-1]), map_.nodeRadian + 2, map_.nodeThickness)
            pygame.draw.line(map_.map, map_.blue, (x[-1], y[-1]),
                             (x[parent[-1]], y[parent[-1]]), map_.edgeThickness)
            # print(graph.number_of_nodes()-n)
            if graph.number_of_nodes() - n == 1:
                new_node = graph.number_of_nodes() - 1
                neighbors = graph.near_neighbors(new_node, 70)
                graph.rewire(map_.map, new_node, neighbors)

            # time.sleep(0.05)
        else:
            n = graph.number_of_nodes()
            x, y, parent = graph.expand()
            pygame.draw.circle(map_.map, map_.grey, (x[-1], y[-1]), map_.nodeRadian + 2, map_.nodeThickness)
            pygame.draw.line(map_.map, map_.blue, (x[-1], y[-1]),
                             (x[parent[-1]], y[parent[-1]]), map_.edgeThickness)
            if graph.number_of_nodes() - n == 1:
                new_node = graph.number_of_nodes() - 1
                neighbors = graph.near_neighbors(new_node, 70)
                graph.rewire(map_.map, new_node, neighbors)
                # pygame.draw.line(map_.map, map_.blue, (x[-1], y[-1]),
                #                  (x[parent[-1]], y[parent[-1]]), map_.edgeThickness)
            # time.sleep(0.05)
        # time.sleep(0.05)
        pygame.display.update()
        iteration += 1
        # map_.drawpath(graph.getPathcoords())
        if iteration % 100 == 0:
            print(iteration)
        if graph.path_to_goal():
            new_cost = graph.getfinalcost()
            if new_cost < oldcost:
                print("New cost :" + str(new_cost))
                if oldpath:
                    for j in range(0, len(oldpath) - 1):
                        pygame.draw.line(map_.map, (255, 255, 255), oldpath[j], oldpath[j + 1],
                                         map_.edgeThickness + 5)
                        # pos1 = map_value(oldpath[i], flag='Py2Vrep')
                        # pos2 = map_value(oldpath[i+1], flag='Py2Vrep')
                        # create_line(pos1, pos2, (1, 1, 1))
                        remove_path()
                        # time.sleep(0.03)
                        pygame.display.update()
                for j in range(0, len(graph.getPathcoords()) - 1):
                    pygame.draw.line(map_.map, (255, 255, 0), graph.getPathcoords()[j], graph.getPathcoords()[j + 1],
                                     map_.edgeThickness + 5)
                    pos1 = map_value(graph.getPathcoords()[j], flag='Py2Vrep')
                    pos2 = map_value(graph.getPathcoords()[j + 1], flag='Py2Vrep')
                    create_line(pos1, pos2, (0, 1, 0))

                    time.sleep(0.03)
                    pygame.display.update()
                oldcost = new_cost
                oldpath = graph.getPathcoords()
                pygame.draw.circle(map_.map, map_.green, map_.start, map_.nodeRadian + 10, 0)
                pygame.draw.circle(map_.map, map_.red, map_.goal, map_.nodeRadian + 20, 1)
        # if iteration % 10 == 0:
        #     # Lưu màn hình Pygame vào file BMP
        #     pygame.image.save(map_.map, f"images/screenshot{n_image}.bmp")
        #
        #     # Sử dụng Pillow để mở và chuyển file BMP sang PNG hoặc GIF
        #     image = Image.open(f"images/screenshot{n_image}.bmp")
        #     image.save(f"images/screenshot{n_image}.png")  # Lưu dưới dạng PNG
        #     # image.save("screenshot.gif")  # Lưu dưới dạng GIF
        #     n_image += 1

    if graph.getfinalcost() != 0:
        print("Optimal cost of " + str(iteration_max) + " iterations is " + str(graph.getfinalcost()))
    else:
        print("don't find any way to goal")
    pygame.display.update()
    # pygame.event.clear()
    # pygame.event.wait(0)

# ===========================================================================================================
if __name__ == '__main__':
    print("Bắt đầu chương trình...")
    sim.simxFinish(-1)

    clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)  # Connect to CoppeliaSim

    if clientID != -1:
        print("Đã kết nối với remote API server")
    else:
        print("Kết nối không thành công!")
        sys.exit("Không thể kết nối")

    # Main Objects
    errorcode, robot_position_handle = sim.simxGetObjectHandle(clientID, 'mecanum_robot', sim.simx_opmode_blocking)
    _, wheel1_handle = sim.simxGetObjectHandle(clientID, '/mecanum_robot/wheel1', sim.simx_opmode_blocking)
    _, wheel3_handle = sim.simxGetObjectHandle(clientID, '/mecanum_robot/wheel3', sim.simx_opmode_blocking)
    _, wheel4_handle = sim.simxGetObjectHandle(clientID, '/mecanum_robot/wheel4', sim.simx_opmode_blocking)
    _, wheel2_handle = sim.simxGetObjectHandle(clientID, '/mecanum_robot/wheel2', sim.simx_opmode_blocking) 

    _, obstacles_handle = sim.simxGetObjectHandle(clientID, 'obstacles', sim.simx_opmode_blocking)
    _, goal_handle = sim.simxGetObjectHandle(clientID, 'Goal', sim.simx_opmode_blocking)  
    pygame.init()
    # Thiết lập biến môi trường SDL để đặt vị trí của cửa sổ
    os.environ['SDL_VIDEO_WINDOW_POS'] = f"{700},{100}"
    remove_path()
    remove_Allpoints()
    # Đánh dấu điểm bắt đầu
    create_circle((get_position(robot_position_handle)[1], get_position(robot_position_handle)[0], 0), 0.3, 10, (0, 1, 0))

    dimensions = (600, 600)
    obsdim = map_value((0.3 - 2.5, 0), flag='Vrep2Py')[0]
    obsnum = 21  # Số vật cản trong môi trường
    iteration_max = 500  # số node tạo ngẫu nhiên để tìm đường tối ưu
    # set_position((-2, -2, 0.5), 0)
    set_velocities(0, 0, 0, 0)  # Thiết lập tốc độ ban đầu cho diff mobile car

    # Lấy toạ độ và hướng của điểm bắt đầu và điểm kết thúc
    init_start = get_position(robot_position_handle)
    start = map_value((init_start[1], init_start[0]), flag='Vrep2Py')
    print(f"Vị trí bắt đầu Start:{start, get_position(robot_position_handle)[2] * 360 / (2 * math.pi)}")
    finish_goal = get_position(goal_handle)
    y_goal, x_goal, finsish_theta_goal = finish_goal

    # Đánh dấu điểm kết thúc
    create_circle((x_goal, y_goal, 0), 0.3, 10)

    # Vrep coordinate can be switched x-axis into y-axis and inverse too
    goal = map_value((x_goal, y_goal), flag='Vrep2Py')
    print(f"Vị trí đích Goal:{[goal[0], goal[1], finsish_theta_goal * 360 / (2 * math.pi)]}")
    # set_velocities(-2, 2, -2, 2)
    obs = []
    rect = None
    for obstacle in get_obstacles_positions():
        centerx = map_value(obstacle, flag='Vrep2Py')[0] - obsdim / 2
        centery = map_value(obstacle, flag='Vrep2Py')[1] - obsdim / 2
        obstacle = (centerx, centery)
        rect = pygame.Rect(obstacle, (obsdim, obsdim))
        obs.append(rect)

    #print(obs)
    result = False
    

    map_ = base.RRTmap(start, goal, dimensions, obsdim, obsnum, 'Informed RRT* path planning')
    graph_ = base.RRTgraph(start, goal, dimensions, obsdim, obsnum)
    graph_.obstacles = obs.copy()

    map_.drawmap(obs)
    graph_.obstacles = graph_.getTrueObs(obs,60)
    obs = graph_.getTrueObs(obs, 60)
    pygame.display.update()
    time.sleep(3)
    i_ = 0
    while not result:
        if i_ > 0:
            graph_.reset()
        try:
            main(obs, graph_, map_)
            if graph_.goalFlag:
                result = True
            else:
                print("finding path again...")
        except IndexError:
            result = False
        i_ += 1
        if i_ >= 3 and graph_.goalFlag:
            print("size of final path: " + str(len(graph_.getPathcoords())))
            print("final cost is " + str(graph_.cost_2))
            print("final cost is " + str(graph_.getfinalcost()))
            break

    #========================Draw nurse curve========================================
    # Tạo một đường cong B-Spline
    from geomdl import BSpline
    from geomdl import utilities
    curve = BSpline.Curve()
    # Bậc của đường cong 
    curve.degree = 3
    # Thiết lập các điểm điều khiển
    curve.ctrlpts = graph_.getPathcoords()
    # Tạo vector nút (knot vector) dựa trên số lượng điểm điều khiển và bậc của đường cong
    curve.knotvector = utilities.generate_knot_vector(curve.degree, len(curve.ctrlpts))
    # Số lượng điểm cần vẽ trên đường cong
    curve.sample_size = 60
    # Tính toán các điểm trên đường NURBS
    curve.evaluate()
    # Vẽ đường NURBS
    # x_val, y_val = zip(*curve.evalpts)
    nurbs = curve.evalpts
    for i in range(0, len(nurbs)-1):
        pygame.draw.line(map_.map,map_.green, nurbs[i], nurbs[i+1],map_.edgeThickness+3)
        pygame.display.update()
        pos1 = map_value(nurbs[i], flag="Py2Vrep")
        pos2 = map_value(nurbs[i+1], flag="Py2Vrep")
        create_line(pos1, pos2, (0, 0, 1))
        


    create_circle((init_start[1], init_start[0], 0), 0.2, 10, color=(1, 1, 0))
    # Đánh dấu điểm kết thúc
    create_circle((x_goal, y_goal, 0), 0.2, 10, color=(1, 1, 0))
    path = []
    # chuyển chuối toạ độ sang Vrep
    for point in nurbs:
        path.append(map_value(point, flag="Py2Vrep"))
    # print(path)
    path.pop(len(nurbs)-1)
    i = len(path) - 1
    e = 0
    x_c, y_c, theta = get_position(robot_position_handle)
    x_d = 0
    y_d = 0
    # 
    # Ban đầu xoay về phía điểm mong muốn trước bằng P controller
    while True:
        x_c, y_c, theta_c = get_position(robot_position_handle)
        theta_d = math.atan2((path[i][0] - y_c), path[i][1] - x_c)
        controller = robot_follow_path.PIDcontroller(get_position(robot_position_handle), (x_c, y_c, theta_d), kP=0, kI=0,
                                                    kD=0, kH=2)
        while not abs(theta_c - theta_d) <= math.pi / 9:
            
            controller.current = get_position(robot_position_handle)
            theta_c = controller.current[2]
            #print(f"thetac ={theta_c} Thetad = {theta_d}")
            w1, w2, w3, w4 = controller.PIDcalculate()
            set_velocities(-w1, -w2, -w3, -w4)
        break
    # 
    # Theo path được chỉ định bằng PID controller
    while True:
        x_c, y_c, theta = get_position(robot_position_handle)
        y_d, x_d = path[i - 1]
    # 
        current = (x_c, y_c, theta)
        desired = (x_d, y_d)
    # 
        # Robot_follow_path.PIDcontroller(current, goal=desired,kP=0.12, kI=0.03, kD=0, kH=0.6, arrive_distance=d_star)
        controller = robot_follow_path.PIDcontroller(current, goal=desired, kP=0.8, kI=0.01, kD=0, kH=1)
                                                    
        # Create trail
        create_point((x_c, y_c), (0, 1, 1))
        while not controller.isArrived(0.3):
            controller.current = get_position(robot_position_handle)    
            w1, w2, w3, w4 = controller.PIDcalculate()
            set_velocities(-w1, -w2, -w3, -w4)  
        i -= 1
        # Xoay về hướng đích mong muốn P controller
        if i <= 0:
            i = 0
            while robot_follow_path.dist((x_c, y_c), (x_d, y_d))>= 0.1:
                x_c, y_c, theta = get_position(robot_position_handle)
                y_d, x_d = path[0]
            set_velocities(0, 0, 0, 0)
            
            while True:
                _, _, theta_c = get_position(robot_position_handle)
                controller = robot_follow_path.PIDcontroller((finish_goal[0], finish_goal[1], theta_c), finish_goal, kP=0,
                                                            kI=0, kD=0, kH=1)
                w1, w2, w3, w4 = controller.PIDcalculate()
                set_velocities(-w1, -w2, -w3, -w4)
                if abs(theta_c - finish_goal[2]) <= math.pi / 36:
                    break
            set_velocities(0, 0, 0, 0)
            break
    sim.simxFinish(clientID)
    sim.simxStopSimulation(clientID, sim.simx_opmode_blocking)  

    # print(get_position())
    # create_line((-2, -2), (2, 2), (1,1,0))
    # set_position((-2, -2), math.pi / 3)
    # # time.sleep(2)
    # print(obs)

    # set_velocities(0.5, 0.5)  
