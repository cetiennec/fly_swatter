import logging
import time
from threading import Timer

import cflib.crtp  # noqa
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.utils import uri_helper

import keyboard  # using module keyboard

from mapping import Map
import numpy as np


uri = uri_helper.uri_from_env(default='radio://0/70/2M/E7E7E7E707')

#uri = uri_helper.uri_from_env(default='radio://0/60/2M/E7E7E7E716')

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)


class Logger:
    """
    Simple logging example class that logs the Stabilizer from a supplied
    link uri and disconnects after 5s.
    """

    def __init__(self, link_uri):
        """ Initialize and run the example with the specified link_uri """

        self._cf = Crazyflie(rw_cache='./cache')

        # Connect some callbacks from the Crazyflie API
        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)

        print('Connecting to %s' % link_uri)

        # Try to connect to the Crazyflie
        self._cf.open_link(link_uri)

        # Variable used to keep main loop occupied until disconnect
        self.is_connected = True

        #Variables in which all states are registered and initialisation
        self.states = dict()

        self.states['t'] = 0
        self.states['stateEstimate.x'] = 0
        self.states['stateEstimate.y'] = 0

        # STEP DETECTION
        # Legacy : size of filter used for step detection
        self.N_filter = 10
        self.states['z_range_hist'] = [0]*self.N_filter

        self.last_time_down = 0
        self.last_time_up = 0

        self.old_measurement = 0
        self.old_filtered = 0
        self.hand_stopped = False

        # Height poke variables and definition of wanted hover height
        self.obstacle_height = 0.1 #m
        self.hover_height = 0.5 #m
        self.is_on_obstacle = True
        self.desired_height = self.hover_height    

        # Lock for the callback to prevent data corruption while operating on it
        self.block_callback =  False  

        # Flag for the emergency stop : put a hand less than 20 cm above the drone and it will land immediately
        self.emergency_stop = False

    def _connected(self, link_uri):
        """ This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""
        print('Connected to %s' % link_uri)

        # The definition of the logconfig can be made before connecting
        self._lg_stab = LogConfig(name='Stabilizer', period_in_ms=50)
        self._lg_stab.add_variable('stateEstimate.x', 'float')
        self._lg_stab.add_variable('stateEstimate.y', 'float')
        self._lg_stab.add_variable('stateEstimate.z', 'float')
        self._lg_stab.add_variable('range.front')
        self._lg_stab.add_variable('range.back')
        self._lg_stab.add_variable('range.up')
        self._lg_stab.add_variable('range.left')
        self._lg_stab.add_variable('range.right')
        self._lg_stab.add_variable('range.zrange')
        # The fetch-as argument can be set to FP16 to save space in the log packet
        # self._lg_stab.add_variable('pm.vbat', 'FP16')

        # Adding the configuration cannot be done until a Crazyflie is
        # connected, since we need to check that the variables we
        # would like to log are in the TOC.
        try:
            self._cf.log.add_config(self._lg_stab)
            # This callback will receive the data
            self._lg_stab.data_received_cb.add_callback(self._stab_log_data)
            # This callback will be called on errors
            self._lg_stab.error_cb.add_callback(self._stab_log_error)
            # Start the logging
            self._lg_stab.start()
        except KeyError as e:
            print('Could not start log configuration,'
                  '{} not found in TOC'.format(str(e)))
        except AttributeError:
            print('Could not add Stabilizer log config, bad configuration.')

        # Start a timer to disconnect in 10s
        t = Timer(100, self._cf.close_link)
        t.start()

    def _stab_log_error(self, logconf, msg):
        """Callback from the log API when an error occurs"""
        print('Error when logging %s: %s' % (logconf.name, msg))

    def _stab_log_data(self, timestamp, data, logconf):
        """Callback from a the log API when data arrives"""

        if not(self.block_callback):
            # Fill in the states variables
            self.block_callback = True
            self.states['dt'] = timestamp - self.states['t']
            self.states['t'] = timestamp

            self.states['vx'] = (data['stateEstimate.x'] - self.states['stateEstimate.x']) / self.states['dt']
            self.states['vy'] = (data['stateEstimate.y'] - self.states['stateEstimate.y']) / self.states['dt']

            for name, value in data.items():
                self.states[name] = value
            # print(self.states['stateEstimate.z'])
            self.height_update()

            # Engagement of emergency stop
            if self.states["range.up"] < 200 and self.states["range.zrange"] > 70:
                print("EMERGENCY STOP")
                self.emergency_stop = True
                for y in range(10):
                    cf.commander.send_hover_setpoint(0, 0, 0, (9-y)/ 10.0 * le.hover_height)
                    time.sleep(0.1)

                cf.commander.send_stop_setpoint()
                self.is_connected = False
                print("Stopped")
            else :
                self.block_callback = False


    def _connection_failed(self, link_uri, msg):
        """Callback when connection initial connection fails (i.e no Crazyflie
        at the specified address)"""
        print('Connection to %s failed: %s' % (link_uri, msg))
        self.is_connected = False

    def _connection_lost(self, link_uri, msg):
        """Callback when disconnected after a connection has been made (i.e
        Crazyflie moves out of range)"""
        print('Connection to %s lost: %s' % (link_uri, msg))

    def _disconnected(self, link_uri):
        """Callback when the Crazyflie is disconnected (called in all cases)"""
        print('Disconnected from %s' % link_uri)
        self.is_connected = False

    def height_update(self) :
        """ Implements the height update with filtering and step detection"""

        self.states['z_range_hist'].pop(0)
        self.states['z_range_hist'].append(self.states['range.zrange'])
        z_hist = np.array(self.states['z_range_hist'])

        # print("VALUE OF STEP",new_value)
        if z_hist[-2] - z_hist[-1] < - 17 and not(self.is_on_obstacle): # condition used to convolve z_hist with filter but was abandoned for something simpler
            print('FOUND LANDING PAD UP')
            self.is_on_obstacle = True
            self.last_time_up = self.states['t']

        elif z_hist[-2] - z_hist[-1] > + 17 and self.is_on_obstacle :
            print('FOUND LANDING PAD DOWN')
            self.is_on_obstacle = False
            self.last_time_down = self.states['t']

        # Tried to do altitude adaptation to keep constant altitude but attempt failed, to slow to set new height setpoint

        return self.hover_height
    
    def send_hover_setpoint(self, arg1, arg2, arg3, arg4):
        '''Just a copy from the cf.commander function but taking the emergency stop as an input'''
        if not(self.emergency_stop):
            cf.commander.send_hover_setpoint(arg1, arg2, arg3,arg4)

    def send_position_setpoint(self, arg1, arg2, arg3, arg4):
        '''Just a copy from the cf.commander function but taking the emergency stop as an input'''
        if not(self.emergency_stop):
            cf.commander.send_position_setpoint(arg1, arg2, arg3,arg4)



if __name__ == '__main__':
    # Initialize the low-level drivers and the logger
    cflib.crtp.init_drivers()
    le = Logger(uri)
    cf = le._cf

    def landing_drone(cf):
        ''' Landing function for the drone, with motor cut-off'''
        for _ in range(10):
            le.send_hover_setpoint(0, 0, 0, le.hover_height)
            time.sleep(0.1)
        for y in range(10):
            le.send_hover_setpoint(0, 0, 0, (9-y)/ 9.0 * le.hover_height)
            time.sleep(0.4)
        for _ in range(5):
            le.send_hover_setpoint(0, 0, 0, 0)
            time.sleep(0.1)
        cf.commander.send_stop_setpoint()


    def taking_off_drone(cf):
        '''Take-off function'''
        for y in range(10):
            le.send_hover_setpoint(0, 0, 0, y/9.0*le.hover_height)
            time.sleep(0.1)
        for _ in range(20):
            le.send_hover_setpoint(0, 0, 0, le.hover_height)
            time.sleep(0.1)

    def action_from_keyboard():
        '''
        Not used in the task, but used throughout debugging, enables control through keyboard:
         - I for forward
         - K for backward
         - J for left banking
         - L for right banking
         - U for left rotation
         - O for rigth rotation
         - S for landing
         Multiple keys can be combined e.g. I + J to go forward and left
         '''
        forward_velocity = 0.0
        left_velocity = 0.0
        yaw_rate = 0.0
        altitude = 0.5
        #key = self.keyboard.getKey()
        try:  # used try so that if user pressed other than the given key error will not be shown
            if keyboard.is_pressed('i'):  # if key 'i' is pressed
                # print('You Pressed i!')
                forward_velocity = 0.3
            elif keyboard.is_pressed('k'):  # if key 'k' is pressed
                # print('You Pressed k!')
                forward_velocity = -0.3
            if keyboard.is_pressed('l'):  # if key 'l' is pressed
                # print('You Pressed l!')
                left_velocity = -0.3
            elif keyboard.is_pressed('j'):  # if key 'j' is pressed
                # print('You Pressed j!')
                left_velocity = 0.3
            if keyboard.is_pressed('u'):  # if key 'u' is pressed
                # print('You Pressed u!')
                yaw_rate = -50
            elif keyboard.is_pressed('o'):  # if key 'o' is pressed
                # print('You Pressed o!')
                yaw_rate = 50
            if keyboard.is_pressed('s'):  # if key 's' is pressed
                return None
            return [forward_velocity, left_velocity, yaw_rate, altitude]
        except:
            return [forward_velocity, left_velocity, yaw_rate, altitude]

    def find_center_lp2(le, cf):
        ''' 
        Function to center drone on the pad using inertia.
        Use of inertia to avoid unprecise positioning due to skew in optical flow.
        Not the best of techniques, but simplest that we found that yields satisfactory results
        '''
        vx = le.states['vx']
        vy = le.states['vy']

        if abs(vx) > abs(vy) :  
            # Find main direction of drone speed, if true, found platform from top or bottom

            # Stabilization, assume the drone will be above the platform
            le.send_hover_setpoint(0,0,0,le.hover_height)

            # Go to perpendicular direction approx for 0.3 m : direction = left
            le.send_hover_setpoint(0, 0.3, 0, le.hover_height)
            time.sleep(1)

            # Stabilize
            le.send_hover_setpoint(0,0,0,le.hover_height)
            time.sleep(1)

            # Make sure pad is not wrongly detected.
            le.is_on_obstacle = False

            # Move right until pad is detected again
            while not(le.is_on_obstacle) :
                le.send_hover_setpoint(0, -0.3, 0, le.hover_height)
                time.sleep(0.1)
            
            # Once detected, let inertia move it completely onto pad

            le.send_hover_setpoint(0,0,0,le.hover_height)


        else : 
            le.send_hover_setpoint(0,0,0,le.hover_height)
            #time.sleep(0.1)

            le.send_hover_setpoint(0.3, 0, 0, le.hover_height)
            time.sleep(1)
            
            le.send_hover_setpoint(0,0,0,le.hover_height)
            time.sleep(1)

            le.is_on_obstacle = False

            while not(le.is_on_obstacle) :
                le.send_hover_setpoint( -0.3, 0, 0, le.hover_height)
                time.sleep(0.1)
            
            le.send_hover_setpoint(0.0,0,0,le.hover_height)
            #time.sleep(0.2)


    # Entering the main function
    map = Map()

    # Start displaying the map, since first display takes quite some time.
    map.display_map_using_cv()

    # Create the waypoints from the left, could switch depending from start_position of the drone
    map.create_waypoints(True)

    # Defining state machine
    state = 'Taking_off_1'

    # Main while loop
    while le.is_connected :

        time.sleep(0.01) # time step of execution of the loop

        if state == 'Taking_off_1':
            taking_off_drone(cf)
            state = 'Go_to_landing_area'
            print("State :", state)
            

        if state == 'Keyboard':
            # For debbuging state, not used during task
            command = action_from_keyboard()
            cf.commander.send_hover_setpoint(command[0], command[1], command[2], le.hover_height)
            if command == None:
                state = 'Final_landing'
                print("State :", state)
            map.update_map(le.states)
            map.display_map_using_cv(le.states)

        if state == 'Go_to_landing_area':
            # Perform A_star to position 3.5, 1.5 m

            # Manual override
            command = action_from_keyboard()
            if command == None:
                state = 'Final_landing'
                print("State :", state)

            # Update map from sensors            
            map.update_map(le.states)

            # Perform A_star
            start_cell = map.cell_from_pos([le.states["stateEstimate.x"], le.states["stateEstimate.y"]])
            original_target = map.cell_from_pos((3.5 - map.x_start_pos, 1.5 - map.y_start_pos))

            if map.grown_map[original_target] < 0: # if target cell is marked as an obstacle, move on to landing area exploration.
                state = 'Search_landing_area'
                print("State :", state)
            
            map.perform_a_star(start_cell, original_target)

            # Send command to drone
            if len(map.optimal_cell_path) > 1: # while path to target is not a singleton
                target_pos = map.simplify_path()
                cf.commander.send_position_setpoint(target_pos[0] ,
                                                    target_pos[1] ,
                                                    le.hover_height,
                                                    0)
            else:
                state = 'Search_landing_area' 
                print("State :", state)
            time.sleep(0.05)

        if state == 'Search_landing_area':
            # State for landing area exploration

            # manual override
            command = action_from_keyboard()
            if command == None:
                state = 'Final_landing'

            # updating
            is_on_step = le.is_on_obstacle

            if is_on_step : # condition to leave state : be on the pad
                state = 'Centering_on_landing_pad'
                print("State :", state)

            else:
                # iterate through free waypoints inside the landing area
                map.update_map(le.states)
                map.update_height_map(le.states,is_on_step)
                
                start_cell = map.cell_from_pos([le.states["stateEstimate.x"], le.states["stateEstimate.y"]])

                map.perform_a_star(start_cell, map.get_current_waypoint())

                if map.optimal_cell_path is not None and len(map.optimal_cell_path) > 1:

                    if len(map.optimal_cell_path) > 3:
                        target_pos = map.simplify_path()
                    else :
                        target_pos = map.pos_from_cell(map.optimal_cell_path[1])
                    cf.commander.send_position_setpoint(target_pos[0] ,
                                                        target_pos[1] ,
                                                        le.hover_height,
                                                        0)
                elif len(map.optimal_cell_path) == 1 : 
                    next_waypoint = map.get_next_waypoint()

                    pos_next_waypoint = map.pos_from_cell(next_waypoint)

                    cf.commander.send_position_setpoint(pos_next_waypoint[0] ,
                                                        pos_next_waypoint[1] ,
                                                        le.hover_height,
                                                        0)

                    map.perform_a_star(start_cell, next_waypoint)
                    target_pos = map.simplify_path()
                    cf.commander.send_position_setpoint(target_pos[0] ,
                                                        target_pos[1] ,
                                                        le.hover_height,
                                                        0)

            

        if state == 'Centering_on_landing_pad':
            find_center_lp2(le,cf)
            state = 'Landing_1'
            print("State :", state)

        if state == 'Landing_1':
            landing_drone(cf)
            cf.commander.send_stop_setpoint() # to be sure : cut motors
            time.sleep(0.5)
            state = 'Taking_off_2'
            print("State :", state)

        if state == 'Taking_off_2':
            taking_off_drone(cf)
            state = 'Go_to_starting_point'
            le.is_on_obstacle = False
            map.create_final_waypoints()
            print("State :", state)

        if state == 'Go_to_starting_point': 
            # This state will go through the waypoints to find the starting pad

            # manual override
            command = action_from_keyboard()
            if command == None:
                state = 'Final_landing'

            # updating
            is_on_step = le.is_on_obstacle
            if is_on_step and le.states['stateEstimate.x'] + map.x_start_pos < 2.0 :
                state = 'Centering_on_starting_pad'
                print("State :", state)
            else:
                map.update_map(le.states)
                map.update_height_map(le.states,is_on_step)
                
                start_cell = map.cell_from_pos([le.states["stateEstimate.x"], le.states["stateEstimate.y"]])

                map.perform_a_star(start_cell, map.get_current_waypoint())

                if map.optimal_cell_path is not None and len(map.optimal_cell_path) > 1:

                    if len(map.optimal_cell_path) >3 :
                        target_pos = map.simplify_path()
                    else :
                        target_pos = map.pos_from_cell(map.optimal_cell_path[1])
                    cf.commander.send_position_setpoint(target_pos[0] ,
                                                        target_pos[1] ,
                                                        le.hover_height,
                                                        0)
                elif len(map.optimal_cell_path) == 1 : 
                    next_waypoint = map.get_next_waypoint()

                    pos_next_waypoint = map.pos_from_cell(next_waypoint)

                    cf.commander.send_position_setpoint(pos_next_waypoint[0] ,
                                                        pos_next_waypoint[1] ,
                                                        le.hover_height,
                                                        0)

                    map.perform_a_star(start_cell, next_waypoint)
                    target_pos = map.simplify_path()
                    cf.commander.send_position_setpoint(target_pos[0] ,
                                                        target_pos[1] ,
                                                        le.hover_height,
                                                        0)

            

        if state == 'Centering_on_starting_pad' :
            find_center_lp2(le,cf)
            state = 'Final_landing'
            print("State :", state)
        
        if state == 'Final_landing':
            landing_drone(cf)
            state = 'Finish'
            print("State :", state)

        if state == 'Finish':
            cf.commander.send_stop_setpoint()
            le.is_connected = False
            break

        # Display our (beautiful) cell map
        map.display_map_using_cv(le.states, state=state)



