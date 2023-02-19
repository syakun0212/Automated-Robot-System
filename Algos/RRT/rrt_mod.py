# %% imports
import math
import random
import matplotlib.pyplot as plt
import numpy as np
import time

show_animation = True

# %%

class RRT:
    class Node:
        def __init__(self, x, y):
            self.x = x
            self.y = y
            self.path_x = []
            self.path_y = []
            self.parent = None

    class AreaBounds:
        def __init__(self, area):
            self.xmin = float(area[0])
            self.xmax = float(area[1])
            self.ymin = float(area[2])
            self.ymax = float(area[3])

    def __init__(self,
                 start, # Start position (x,y)
                 goal,  # Goal position (x,y)
                 obstacle_list,
                 rand_area, # Random sampling area
                 expand_dis=3.0,
                 path_resolution=0.5,
                 goal_sample_rate=5,
                 max_iter=500,
                 play_area=None # Stay inside this area
                 ):
        self.start = self.Node(start[0], start[1])
        self.end = self.Node(goal[0], goal[1])
        self.min_rand = rand_area[0]
        self.max_rand = rand_area[1]
        if play_area is not None:
            self.play_area = self.AreaBounds(play_area)
        else:
            self.play_area = None
        self.expand_dis = expand_dis
        self.path_resolution = path_resolution
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter
        self.obstacle_list = obstacle_list
        self.node_list = []

    def calc_dist_to_goal(self, x, y):
        dx = x - self.end.x
        dy = y - self.end.y
        return math.hypot(dx, dy)

    def get_random_node(self):
        if random.randint(0, 200) > self.goal_sample_rate:
            rnd = self.Node(
                random.uniform(self.min_rand, self.max_rand),
                random.uniform(self.min_rand, self.max_rand))
        else:  # goal point sampling
            rnd = self.Node(self.end.x, self.end.y)
        return rnd


    def steer(self, from_node, to_node, extend_length=float("inf")):

        new_node = self.Node(from_node.x, from_node.y)
        d, theta = self.calc_distance_and_angle(new_node, to_node)

        new_node.path_x = [new_node.x]
        new_node.path_y = [new_node.y]

        if extend_length > d:
            extend_length = d

        n_expand = math.floor(extend_length / self.path_resolution)

        for _ in range(n_expand):
            new_node.x += self.path_resolution * math.cos(theta)
            new_node.y += self.path_resolution * math.sin(theta)
            new_node.path_x.append(new_node.x)
            new_node.path_y.append(new_node.y)

        d, _ = self.calc_distance_and_angle(new_node, to_node)
        if d <= self.path_resolution:
            new_node.path_x.append(to_node.x)
            new_node.path_y.append(to_node.y)
            new_node.x = to_node.x
            new_node.y = to_node.y

        new_node.parent = from_node

        return new_node

    def generate_final_course(self, goal_ind):
        path = [[self.end.x, self.end.y]]
        node = self.node_list[goal_ind]
        while node.parent is not None:
            path.append([node.x, node.y])
            node = node.parent
        path.append([node.x, node.y])

        return path

    def draw_graph(self, rnd=None):
        plt.clf()
        # for stopping simulation with the esc key.
        plt.figure(figsize=(10,10))
        plt.gcf().canvas.mpl_connect(
            'key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])
        if rnd is not None:
            plt.plot(rnd.x, rnd.y, "^k")
        for node in self.node_list:
            if node.parent:
                plt.plot(node.path_x, node.path_y, "-g")

        # Plot starting point
        starting_rng = list(range(0,41,1))
        xl_b=[0 + a for a in starting_rng ]
        yl_b=[0 + a for a in starting_rng]
        plt.plot(xl_b,[0 for a in starting_rng], "grey", linewidth=5, linestyle="--")
        plt.plot(xl_b,[40 for a in starting_rng], "grey", linewidth=5, linestyle="--")
        plt.plot([0 for a in starting_rng], yl_b, "grey", linewidth=5, linestyle="--")
        plt.plot([40 for a in starting_rng], yl_b, "grey", linewidth=5, linestyle="--")


        for (ox, oy, image_dir) in self.obstacle_list:
            self.plot_obs(ox, oy, image_dir)

        if self.play_area is not None:
            plt.plot([self.play_area.xmin, self.play_area.xmax,
                      self.play_area.xmax, self.play_area.xmin,
                      self.play_area.xmin],
                     [self.play_area.ymin, self.play_area.ymin,
                      self.play_area.ymax, self.play_area.ymax,
                      self.play_area.ymin],
                     "-k")

        plt.plot(self.start.x, self.start.y, "og", markersize=12)
        plt.plot(self.end.x, self.end.y, "xr", markersize=12)
        plt.axis("equal")
        plt.axis([0, 200, 0, 200])
        plt.grid(True)
        plt.pause(0.01)


    def planning(self,animation=True):
        self.node_list = [self.start]

        for i in range(self.max_iter):
            rnd_node = self.get_random_node()
            nearest_ind = self.get_nearest_node_index(self.node_list, rnd_node)
            nearest_node = self.node_list[nearest_ind]

            new_node = self.steer(nearest_node, rnd_node, self.expand_dis)

            if self.check_if_outside_play_area(new_node, self.play_area) and \
                self.check_collision(new_node, self.obstacle_list):
                self.node_list.append(new_node)

            # Draw Graph
            if animation and i % 100 == 0:
                self.draw_graph(rnd_node)

            if self.calc_dist_to_goal(self.node_list[-1].x,
                                        self.node_list[-1].y) <= self.expand_dis:
                final_node = self.steer(self.node_list[-1], self.end,
                                        self.expand_dis)
                if self.check_collision(final_node, self.obstacle_list):
                    return self.generate_final_course(len(self.node_list) - 1)

            if animation and i % 5:
                self.draw_graph(rnd_node)
        return None

    @staticmethod
    def get_nearest_node_index(node_list, rnd_node):
        dlist = [(node.x - rnd_node.x)**2 + (node.y - rnd_node.y)**2
                 for node in node_list]
        minind = dlist.index(min(dlist))

        return minind

    @staticmethod
    def plot_obs(x, y, image_dir, color_obs="navy", color_bdry="cornflowerblue"):
        # plot obs
        obs_rng = list(range(-5,+6,1))
        xl=[x + a for a in obs_rng]
        yl=[y + a for a in obs_rng]
        plt.plot(xl,[y-5 for a in obs_rng], color_obs, linewidth=3)
        plt.plot(xl,[y+5 for a in obs_rng], color_obs, linewidth=3)
        plt.plot([x-5 for a in obs_rng],yl, color_obs, linewidth=3)
        plt.plot([x+5 for a in obs_rng], yl, color_obs, linewidth=3)

        # plot boundtary
        boundary_rng = list(range(-20,+21,1))
        xl_b=[x + a for a in boundary_rng ]
        yl_b=[y + a for a in boundary_rng ]
        plt.plot(xl_b,[y-20 for a in boundary_rng ], color_bdry, linewidth=1.5, linestyle="--")
        plt.plot(xl_b,[y+20 for a in boundary_rng ], color_bdry, linewidth=1.5, linestyle="--")
        plt.plot([x-20 for a in boundary_rng ], yl_b, color_bdry, linewidth=1.5, linestyle="--")
        plt.plot([x+20 for a in boundary_rng ], yl_b, color_bdry, linewidth=1.5, linestyle="--")

    @staticmethod
    # Function to change
    def check_collision(node, obstacleList):

        if node is None:
            return False

        for (ox, oy, image_dir) in obstacleList:
            dx_list = [ox - x for x in node.path_x]
            dy_list = [oy - y for y in node.path_y]
            d_list = [dx * dx + dy * dy for (dx, dy) in zip(dx_list, dy_list)]
            # Block area - 15cm top and bottom
            if min(d_list) <= 20**2:
                return False  # collision

        return True  # safe

    @staticmethod
    def calc_distance_and_angle(from_node, to_node):
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        d = math.hypot(dx, dy)
        theta = math.atan2(dy, dx)
        return d, theta

    @staticmethod
    def check_if_outside_play_area(node, play_area):

        if play_area is None:
            return True  # no play_area was defined, every pos should be ok

        if node.x < play_area.xmin or node.x > play_area.xmax or \
           node.y < play_area.ymin or node.y > play_area.ymax:
            return False  # outside - bad
        else:
            return True  # inside - ok

def main(gx=95.0, gy=106.0):

    start_time = time.time()
    print("start " + __file__)

    # ====Search Path with RRT====
    obstacleList = [(55, 65, 1), (95,85, 2), (115,155, 2), (75,165, 2), (155,185, 2)]
    start=[20, 20]
    goal=[gx, gy]
    # Set Initial parameters
    rrt = RRT(
        start=start,
        goal=goal,
        rand_area=[0, 200],
        expand_dis=20.0,
        path_resolution=1,
        max_iter=1000,
        obstacle_list=obstacleList,
        # goal_sample_rate=None,
        # play_area=[0, 10, 0, 14]
        )
    path = rrt.planning(animation=show_animation)

    if path is None:
        print("Cannot find path")
    else:
        print("found path!!")

    # show final path
    plt.gcf()
    plt.figure(figsize=(10,10))

    # Plot starting point
    starting_rng = list(range(0,41,1))
    xl_b=[0 + a for a in starting_rng ]
    yl_b=[0 + a for a in starting_rng]
    plt.plot(xl_b,[0 for a in starting_rng], "grey", linewidth=5, linestyle="--")
    plt.plot(xl_b,[40 for a in starting_rng], "grey", linewidth=5, linestyle="--")
    plt.plot([0 for a in starting_rng], yl_b, "grey", linewidth=5, linestyle="--")
    plt.plot([40 for a in starting_rng], yl_b, "grey", linewidth=5, linestyle="--")

    # plot obstacles
    for (ox, oy, image_dir) in obstacleList :
        RRT.plot_obs(ox, oy, image_dir)

    # plot path
    plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
    plt.grid(True)
    plt.plot(start[0], start[1], "og", markersize=12)
    plt.plot(goal[0], goal[1], "xr", markersize=12)
    plt.axis("equal")
    plt.axis([0, 200, 0, 200])
    plt.show()

    print("Execution time: ", round(time.time() - start_time, 2), " seconds")

    return path

    # Draw final path
    # if show_animation:
    #     rrt.draw_graph()
    #     plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
    #     plt.grid(True)
    #     plt.pause(0.01)  # Need for Mac
    #     plt.show()

# %%main()
__file__="rrt_mod.py"
if __name__ == '__main__':
    main()

# %%
