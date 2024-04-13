import numpy as np
import math
import matplotlib
import matplotlib.pyplot as plt
import time
import heapq
import rospy
from geometry_msgs.msg import Twist

class Node:

    def __init__(self, x, y, parent, current_theta, delta_theta, Lrpm, Rrpm, g_cost, h_cost, f_cost):
        self.x = x
        self.y = y
        self.parent = parent
        self.current_theta = current_theta
        self.delta_theta = delta_theta
        self.Lrpm = Lrpm
        self.Rrpm = Rrpm
        self.g_cost = g_cost
        self.h_cost = h_cost
        self.f_cost = f_cost

        Node.__lt__ = lambda self, other: self.f_cost < other.f_cost

def isCircle(x, y, clearance):
    circle = ((np.square(x - 4)) + (np.square(y - 1.1)) <= np.square(0.5 + clearance))
    return circle

def isRectangle(x, y, clearance):
    # two obstacles
    box1 = (x >= 1.5 - clearance) and (x <= 1.65 + clearance) and (y >= 0.75 - clearance)
    box2 = (x >= 2.5 - clearance) and (x <= 2.65 + clearance) and (y <= 1.25 + clearance)

    #border clearances
    rect1 = (x >= 0) and (x <= clearance)
    rect2 = (x >= 6 - clearance)
    rect3 = (y <= clearance)
    rect4 = (y >= 2 - clearance)

    return box1 or box2 or rect1 or rect2 or rect3 or rect4

def isValid(x, y, clearance):
    return not (isCircle(x, y, clearance) or isRectangle(x, y, clearance))

def isGoal(current, goal, tolerance=0.2):
    return math.dist((current.x, current.y), (goal.x, goal.y)) < tolerance

def threshold(x, y, th, theta):
    return round(x, 1), round(y, 1), round(th / theta) * theta

def actions(rpm1, rpm2):
    action = [[rpm1, 0], [0, rpm1], [rpm1, rpm1], [0, rpm2], [rpm2, 0], [rpm2, rpm2], [rpm1, rpm2], [rpm2, rpm1]]
    return action

def cost(Xi, Yi, Thetai, UL, UR, clearance, return_path, N_list, S_list):
    t = 0
    r = 0.033
    cost = 0
    Xn = Xi
    Yn = Yi
    Thetan = 3.14 * Thetai / 180
    flag = True

    while t < 1 and flag:
        t += 0.1
        Xs, Ys = Xn, Yn
        UL_UR_avg = 0.5 * (UL + UR)
        cos_theta, sin_theta = math.cos(Thetan), math.sin(Thetan)
        Xn += r * UL_UR_avg * cos_theta * 0.1
        Yn += r * UL_UR_avg * sin_theta * 0.1
        Thetan += (r / 0.16) * (UR - UL) * 0.1
        if not isValid(Xn, Yn, clearance):
            flag = False
        elif return_path == 0:
            c2g = math.dist((Xs, Ys), (Xn, Yn))
            cost += c2g
            N_list.append((Xn, Yn))
            S_list.append((Xs, Ys))
        elif return_path == 1:
            plt.plot([Xs, Xn], [Ys, Yn], color="red")

    Thetan = 180 * Thetan / 3.14
    return [Xn, Yn, Thetan, cost, N_list, S_list]

def astar(start_node, goal_node, rpm1, rpm2, clearance):
    N_list = []
    S_list = []
    open_nodes = {}
    open_nodes[(start_node.x * 5000 + start_node.y)] = start_node
    closed_nodes = set()
    theta_threshold = 15
    open_nodes_list = [(start_node.f_cost, start_node)]
    dist_fn = math.dist

    while open_nodes_list:
        current_node = heapq.heappop(open_nodes_list)[1]
        current_id = current_node.x * 5000 + current_node.y

        if isGoal(current_node, goal_node):
            goal_node.parent = current_node.parent
            goal_node.f_cost = current_node.f_cost
            print("Goal Node found")
            return 1, N_list, S_list

        if current_id in closed_nodes:
            continue

        closed_nodes.add(current_id)
        del open_nodes[current_id]

        for action in actions(rpm1, rpm2):
            next_node_data = cost(current_node.x, current_node.y, current_node.current_theta, action[0], action[1], clearance, 0, N_list, S_list)
            if not next_node_data:
                continue

            angle = next_node_data[2]
            th = round(angle / theta_threshold) * theta_threshold % 360
            ct = current_node.delta_theta - th
            x, y, t = threshold(next_node_data[0], next_node_data[1], th, theta_threshold)
            c2g = dist_fn((x, y), (goal_node.x, goal_node.y))
            new_node = Node(x, y, current_node, t, ct, action[0], action[1], current_node.g_cost + next_node_data[3], c2g,
                            current_node.g_cost + next_node_data[3] + c2g)

            new_node_id = new_node.x * 5000 + new_node.y

            if isGoal(new_node, goal_node):
                goal_node.parent = new_node.parent
                goal_node.f_cost = new_node.f_cost
                print("Goal Node found")
                return 1, N_list, S_list

            if not isValid(new_node.x, new_node.y, clearance):
                continue

            if new_node_id in closed_nodes:
                continue

            if new_node_id in open_nodes:
                if new_node.f_cost < open_nodes[new_node_id].f_cost:
                    open_nodes[new_node_id].f_cost = new_node.f_cost
                    open_nodes[new_node_id].parent = new_node
            else:
                open_nodes[new_node_id] = new_node
                heapq.heappush(open_nodes_list, (new_node.f_cost, new_node))

    return 0, N_list, S_list

def Generate_Path(goal_node):
    path = []
    UL = []
    UR = []
    path.append((goal_node.x, goal_node.y, goal_node.current_theta))
    UL.append(goal_node.Lrpm)
    UR.append(goal_node.Rrpm)
    parent_node = goal_node.parent

    while parent_node != -1:
        path.append((parent_node.x, parent_node.y, parent_node.current_theta))
        UL.append(parent_node.Lrpm)
        UR.append(parent_node.Rrpm)
        parent_node = parent_node.parent

    UL_arr = np.array(UL[::-1])
    UR_arr = np.array(UR[::-1])
    path.reverse()
    return np.array(path)[:,0], np.array(path)[:,1], np.array(path)[:,2], UL_arr, UR_arr

def calculate_velocities(Xi, Yi, Thetai, UL, UR):
    r = 0.033
    L = 0.16
    dt = 0.1

    UL = UL * 2 * math.pi / 60
    UR = UR * 2 * math.pi / 60

    thetan = 3.14 * Thetai / 180

    theta_dot = (r / L) * (UR - UL)
    change_theta = theta_dot + thetan

    velocity = (r / 2) * (UL + UR)

    return velocity, theta_dot

def ros_pub(xpath, ypath, Lrpm, Rrpm, theta_path):
    rospy.init_node('velocity_publisher', anonymous=True)
    cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=100)
    twist = Twist()

    twist.linear.x = 0.0
    twist.linear.y = 0.0
    twist.linear.z = 0.0
    twist.angular.x = 0.0
    twist.angular.y = 0.0
    twist.angular.z = 0.0
    cmd_vel.publish(twist)

    c = 0
    rate = rospy.Rate(10)

    for i in range(len(xpath)):
        print(f"Lrpm:{Lrpm[i]}, Rrpm:{Rrpm[i]}")
        while not rospy.is_shutdown():
            if c == 111:
                twist.linear.x = 0
                twist.angular.z = 0
                cmd_vel.publish(twist)
                break
            else:
                velocity, angle = calculate_velocities(xpath[i], ypath[i], theta_path[i], Lrpm[i], Rrpm[i])
                twist.linear.x = velocity
                twist.angular.z = angle
                cmd_vel.publish(twist)
                c = c + 1
                rate.sleep()
        c = 0

if __name__ == '__main__':
    width = 6
    height = 2
    robot_radius = 0.105

    clearance = float(input("Enter the clearance of obstacles(in meter): ")) + robot_radius

    RPM1, RPM2 = map(int, input("Enter the left and right RPMs: ").split())

    x_start, y_start = map(float, input("Enter the start coordinates(in meter): ").split())
    x_start += 0.5
    y_start += 1

    if not isValid(x_start, y_start, clearance):
        print("Invalid start node or Node is in Obstacle space")
        exit(-1)

    start_theta = int(input("Enter the robot start angle: "))

    x_goal, y_goal = map(float, input("Enter the goal coordinates(in meter): ").split())
    x_goal += 0.5
    y_goal += 1

    if not isValid(x_goal, y_goal, clearance):
        print("Invalid goal node or Node is in Obstacle space")
        exit(-1)

    start_time = time.time()

    cost_to_go = ((x_goal - y_goal) ** 2 + (y_goal - x_start) ** 2) ** 0.5
    total_cost = cost_to_go
    start_node = Node(x_start, y_start, -1, start_theta, 0, 0, 0, 0, cost_to_go, total_cost)
    goal_node = Node(x_goal, y_goal, -1, 0, 0, 0, 0, cost_to_go, 0, total_cost)

    flag, N_list, S_list = astar(start_node, goal_node, RPM1, RPM2, clearance)
    total_time = time.time() - start_time
    print("Total time: ", total_time)

    if flag:
        xpath, ypath, theta_path, Lrpm, Rrpm = Generate_Path(goal_node)
        print(f"xpath: {xpath}\n")
        print(f"ypath: {ypath}\n")
        print(f"lrpm: {Lrpm}\n")
        print(f"Rrpm: {Rrpm}\n")
    else:
        print("Goal not reachable!")
        exit(-1)

    ros_pub(xpath, ypath, Lrpm, Rrpm, theta_path)