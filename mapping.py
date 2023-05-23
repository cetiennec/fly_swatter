import numpy as np
import matplotlib.pyplot as plt
import cv2

# use the following command for opencv : pip install opencv-contrib-python

class Map :
    def __init__(self):
        self.min_x, self.max_x = 0, 5.0  # meter
        self.min_y, self.max_y = 0, 3.0  # meter

        self.res_pos = 0.15  # meter, must be inferior to size of landing pad

        self.size_of_grown_margin = 1  # in number of cells
        self.use_diagonal_neighbors = True

        self.range_max = 1.1  # meter, maximum range of distance sensor
        self.conf = 0.2  # certainty given by each measurement

        self.bare_map = np.zeros(
            (
                int((self.max_y - self.min_y) / self.res_pos),
                int((self.max_x - self.min_x) / self.res_pos),
            )
        )  # 0 = unknown, 1 = free, -1 = occupied

        self.grown_map = np.zeros_like(self.bare_map)

        self.waypoint = None
        self.optimal_cell_path = None

        self.plot_ready = False


#%% Update Map

    def update_map(self, sensor_data):
        """Update map from sensor_data, will increase or decrease the certainty value on cells for which the sensors have a measurement"""
        pos_x, pos_y = [sensor_data["stateEstimate.x"] + 2.5, 1.5 + sensor_data["stateEstimate.y"]]
        yaw = sensor_data["stabilizer.yaw"]

        for j in range(4):  # 4 sensors
            yaw_sensor = j * np.pi / 2  # yaw positive is counter clockwise
            if j == 0:
                measurement = sensor_data["range.front"]
            elif j == 1:
                measurement = sensor_data["range.left"]
            elif j == 2:
                measurement = sensor_data["range.back"]
            elif j == 3:
                measurement = sensor_data["range.right"]

            measurement = measurement / 1000 #to convert it back to meters
            for i in range(int(self.range_max / self.res_pos)):  # range is 2 meters
                dist = i * self.res_pos
                idx_x, idx_y = self.cell_from_pos(
                    [
                        pos_x + dist * np.cos(yaw_sensor),
                        pos_y + dist * np.sin(yaw_sensor),
                    ]
                )

                # make sure the point is within the map
                if (
                    idx_x < 0
                    or idx_x >= self.grown_map.shape[0]
                    or idx_y < 0
                    or idx_y >= self.grown_map.shape[1]
                    or dist > self.range_max
                ):
                    break

                # update the map
                if dist < measurement:
                    self.bare_map[idx_x, idx_y] += self.conf
                else:
                    self.bare_map[idx_x, idx_y] -= self.conf
                    break

        # roll the bare measurement map, as to enforce safety margin around the obstacles
        self.bare_map = np.clip(self.bare_map, -1, 1)
        for i in range(-self.size_of_grown_margin, self.size_of_grown_margin + 1, 1):
            for j in range(
                -self.size_of_grown_margin, self.size_of_grown_margin + 1, 1
            ):
                if i == 0 and j == 0:
                    self.grown_map += np.roll(
                        np.clip(self.bare_map, -1, 1), (i, j), axis=(0, 1)
                    )
                else:
                    self.grown_map += np.roll(
                        np.clip(self.bare_map, -1, 0), (i, j), axis=(0, 1)
                    )

        self.grown_map = np.clip(self.grown_map, -1, 1)  # certainty can never be more than 100%

#%% Utils

    def pos_from_cell(self, cell):
        return (
            cell[1] * self.res_pos + self.min_x,
            self.max_y - cell[0] * self.res_pos,
        )

    def cell_from_pos(self, position):
        idx_x = int(np.round((self.max_y - position[1]) / self.res_pos, 0))
        idx_y = int(np.round((position[0] - self.min_x) / self.res_pos, 0))
        return (idx_x, idx_y)
    
    def merge_sorted_list(self, list1, list2, heuristics1, heuristics2):
        result_list = []
        heuristics = []
        while len(list1) > 0 and len(list2) > 0:
            if heuristics1[0] > heuristics2[0]:
                result_list.append(list2.pop(0))
                heuristics.append(heuristics2.pop(0))
            else:
                result_list.append(list1.pop(0))
                heuristics.append(heuristics1.pop(0))

        for element in list1:
            result_list.append(element)
            heuristics.append(heuristics1.pop(0))

        for element in list2:
            result_list.append(element)
            heuristics.append(heuristics2.pop(0))

        return result_list, heuristics
    
    def find_neighbors(self, cell):
        cell_neighbors = []
        distances = []

        if self.use_diagonal_neighbors :
            for i in [-1, 0, 1]:
                for j in [-1, 0, 1]:
                    if (
                        (i != 0 or j != 0)
                        and cell[0] + i >= 0
                        and cell[0] + i < self.grown_map.shape[0]
                        and cell[1] + j >= 0
                        and cell[1] + j < self.grown_map.shape[1]
                    ):
                        cell_neighbors.append((cell[0] + i, cell[1] + j))
                        distances.append(self.distance((0, 0), (i, j)))
        else :
            for i in [-1, 1]:
                if (
                    cell[0] + i >= 0
                    and cell[0] + i < self.grown_map.shape[0]
                ):
                    cell_neighbors.append((cell[0] + i, cell[1]))
                    distances.append(abs(i))
            for j in [-1, 1]:
                if (
                    cell[1] + j >= 0
                    and cell[1] + j < self.grown_map.shape[1]
                ):
                    cell_neighbors.append((cell[0], cell[1] + j))
                    distances.append(abs(j))
        return cell_neighbors, distances
    
    def distance(self,cell_a,cell_b) :
        return np.linalg.norm(np.array(cell_a) - np.array(cell_b))
    

#%% Display the map
    def display_cell_map(self, sensor_data=None):
        if not self.plot_ready:
            plt.ion()
            plt.subplot(131)
            self.cell_map_plot = plt.imshow(
                np.transpose(self.grown_map), vmin=-1, vmax=1, cmap="gray", origin="lower"
            )
            (self.optimal_cell_path_plot,) = plt.plot(0, 0, "go")
            (self.cell_pos_plot,) = plt.plot(0, 0, "r+")
            (self.waypoint_plot,) = plt.plot(0, 0, "bx")

            plt.title("Cell map and planned path")
            plt.xlabel("cX")
            plt.ylabel("cY")

            self.plot_ready = True

        # update map data and flip the map , so it appears with the same coordinate system as simulation
        self.cell_map_plot.set_data(np.transpose(self.grown_map))

        # get position
        if sensor_data is not None :
            idx_x, idx_y = self.cell_from_pos(
                [sensor_data["stateEstimate.x"] + 2.5, 1.5 + sensor_data["stateEstimate.y"]]
            )
            self.cell_pos_plot.set_xdata(idx_x)
            self.cell_pos_plot.set_ydata(idx_y)

        if self.waypoint is not None:
            self.waypoint_plot.set_xdata(self.waypoint[0])
            self.waypoint_plot.set_ydata(self.waypoint[1])

        if self.optimal_cell_path is not None:
            self.optimal_cell_path_plot.set_xdata(
                [cell[0] for cell in self.optimal_cell_path]
            )
            self.optimal_cell_path_plot.set_ydata(
                [cell[1] for cell in self.optimal_cell_path]
            )

        plt.pause(0.001)

    def display_map_using_cv(self, sensor_data = None):
        upscaling_factor = 20
        gray_image = np.transpose(self.grown_map)
        gray_image = (gray_image+1)/2
        


        (width, height) = np.shape(gray_image)

        gray_image = cv2.resize(gray_image,(upscaling_factor*height, upscaling_factor*width) , interpolation=cv2.INTER_NEAREST)
        
        map_image = cv2.cvtColor(gray_image.astype('float32'), cv2.COLOR_GRAY2BGR)
        if self.optimal_cell_path is not None:
            for cell in self.optimal_cell_path :
                cv2.circle(map_image, (upscaling_factor*cell[0],upscaling_factor*cell[1]), int(upscaling_factor*0.7),(0,255,0),-1)

        if sensor_data is not None :
            idx_x, idx_y = self.cell_from_pos(
                [2.5 + sensor_data["stateEstimate.x"], 1.5 + sensor_data["stateEstimate.y"]]
            )
            cv2.circle(map_image, (upscaling_factor*idx_x, upscaling_factor*idx_y), int(upscaling_factor*0.5),(255,0, 0),-1)

        map_image = np.flip(map_image, axis= 0)
        cv2.imshow('Cell map and planned path', map_image)
        cv2.waitKey(1)
    



    
#%% Path planning : A* algorithm
    def perform_a_star(self, start_cell, target_cell):
        self.grown_map_astar = np.zeros_like(self.grown_map)  # unmarked at 0
        self.grown_map_astar[self.grown_map < 0] = -1  # to keep convention

        cameFrom = np.zeros(self.grown_map_astar.shape, dtype=tuple)

        cells_to_explore = [start_cell]
        heuristics = [0]

        self.grown_map_astar[start_cell] = 1
        cameFrom[
            start_cell
        ] = start_cell  # the start cell is the only one that comes from itself
        # print(target_cell)
        if self.grown_map[target_cell] < 0:
            self.optimal_cell_path = None
            return

        # do the A* algorithm
        while (
            self.grown_map_astar[target_cell] == 0 and len(cells_to_explore) > 0
        ):  # while goal is unmarked and you still have cells to explore
            current_cell = cells_to_explore.pop(0)  # take first cell
            heuristics.pop(0)

            valid_neighbors = []
            valid_heuristics = []

            neighbors, distances = self.find_neighbors(current_cell)

            for neighbor, distance in zip(neighbors, distances):
                if self.grown_map_astar[neighbor] >= 0:  # neighbor is not a wall
                    if self.grown_map_astar[neighbor] == 0:  # path has not been marked yet
                        self.grown_map_astar[neighbor] = (
                            self.grown_map_astar[current_cell] + distance
                        )
                        cameFrom[neighbor] = current_cell

                        valid_neighbors.append(
                            neighbor
                        )  # neighbor should be (re-)considered
                        valid_heuristics.append(
                            self.grown_map_astar[neighbor]
                            + self.distance(neighbor, target_cell)
                        )  # definition of consistent heuristic

                    if (
                        self.grown_map_astar[neighbor]
                        > self.grown_map_astar[current_cell] + distance
                    ):  # we found a better path than previous one
                        cameFrom[neighbor] = current_cell
                        self.grown_map_astar[neighbor] = (
                            self.grown_map_astar[current_cell] + distance
                        )

            # sort neighbors according to heuristics
            order = np.array(
                valid_heuristics
            ).argsort()  # returns list of indexes that would sort array
            valid_neighbors = [valid_neighbors[i] for i in order]
            valid_heuristics = [valid_heuristics[i] for i in order]

            cells_to_explore, heuristics = self.merge_sorted_list(
                cells_to_explore, valid_neighbors, heuristics, valid_heuristics
            )

        # now find best path
        current = target_cell
        self.optimal_cell_path = [current]
        while current != cameFrom[current]:
            current = cameFrom[current]
            self.optimal_cell_path.append(current)

        self.optimal_cell_path.reverse()

# map = Map()
# map.grown_map[0,1] = 1
# states = dict()
# states["stateEstimate.x"] = 0.0
# states["stateEstimate.y"] = 3.0

# print(map.cell_from_pos(
#                 [states["stateEstimate.x"], states["stateEstimate.y"]]))

# map.display_map_using_cv(states)
# cv2.waitKey(2000)