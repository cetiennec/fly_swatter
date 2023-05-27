from mapping import Map
import cv2

map = Map()
map.grown_map[0,1] = 1

map.grown_map[0:14,17:] = -1
states = dict()
states["stateEstimate.x"] = 0.0
states["stateEstimate.y"] = 1.0

map.create_waypoints(False)
waypoint = map.get_next_waypoint()
print("Next waypoint : " + str(waypoint))

original_target = map.cell_from_pos((3.5 - map.x_start_pos, 1.5 - map.y_start_pos))

map.perform_a_star(map.cell_from_pos(
                [states["stateEstimate.x"], states["stateEstimate.y"]]), original_target)


print("Print pos in cell format : " + str(map.cell_from_pos(
                [states["stateEstimate.x"], states["stateEstimate.y"]])))
print("Expected :  (3,17)")

print("Print cell in pos format: " + str(map.pos_from_cell((10, 17))))
print("Expected :  (0,0)")

map.display_map_using_cv(states)
cv2.waitKey(10000) # close window after 10 seconds