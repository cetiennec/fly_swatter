from mapping import Map
import cv2

map = Map()
map.grown_map[0,1] = 1
states = dict()
states["stateEstimate.x"] = 0.0
states["stateEstimate.y"] = 0.0

print(map.cell_from_pos(
                [states["stateEstimate.x"], states["stateEstimate.y"]]))

map.display_map_using_cv(states)
cv2.waitKey(0) #press some key to close the window