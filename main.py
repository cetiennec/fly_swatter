import logging
import time
from threading import Timer

import cflib.crtp  # noqa
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.utils import uri_helper

import keyboard  # using module keyboard

from mapping import Map


uri = uri_helper.uri_from_env(default='radio://0/70/2M/E7E7E7E707')

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

        self.states = dict()

        self.old_measurement = 0
        self.old_filtered = 0
        self.hand_stopped = False

    def _connected(self, link_uri):
        """ This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""
        print('Connected to %s' % link_uri)

        # The definition of the logconfig can be made before connecting
        self._lg_stab = LogConfig(name='Stabilizer', period_in_ms=50)
        self._lg_stab.add_variable('stateEstimate.x', 'float')
        self._lg_stab.add_variable('stateEstimate.y', 'float')
        self._lg_stab.add_variable('stateEstimate.z', 'float')
        self._lg_stab.add_variable('stabilizer.yaw', 'float')
        self._lg_stab.add_variable('range.front')
        self._lg_stab.add_variable('range.back')
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
        # print(f'[{timestamp}][{logconf.name}]: ', end='')
        # for name, value in data.items():
        #     print(f'{name}: {value:3.3f} ', end='')
        # print()
        for name, value in data.items():
            self.states[name] = value

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

    def HP_filter(self,new_value):
        filtered = 0.1*self.old_filtered + 0.1*(new_value-self.old_measurement )
        print("VALUE OF STEP",abs(new_value-self.old_measurement ))
        if new_value-self.old_measurement >=20 and self.old_measurement !=0:
            print('FOUND LANDING PAD UPP')
            return True
        elif new_value-self.old_measurement  <= -20 and self.old_measurement !=0:
            print('FOUND LANDING PAD DOWN')
            return True
        self.old_filtered = filtered
        self.old_measurement = new_value
        return False


if __name__ == '__main__':
    # Initialize the low-level drivers
    # Tools
    def landing_drone(cf):
        for _ in range(20):
            cf.commander.send_hover_setpoint(0, 0, 0, 0.4)
            time.sleep(0.1)
        for y in range(10):
            cf.commander.send_hover_setpoint(0, 0, 0, (10-y)/ 25)
            time.sleep(0.1)
        for _ in range(10):
            cf.commander.send_hover_setpoint(0, 0, 0, 0)
            time.sleep(0.1)


    def taking_off_drone(cf):
        for y in range(10):
            cf.commander.send_hover_setpoint(0, 0, 0, y / 25)
            time.sleep(0.1)
        for _ in range(20):
            cf.commander.send_hover_setpoint(0, 0, 0, 0.4)
            time.sleep(0.1)

    def action_from_keyboard():
        forward_velocity = 0.0
        left_velocity = 0.0
        yaw_rate = 0.0
        altitude = 0.5
        #key = self.keyboard.getKey()
        try:  # used try so that if user pressed other than the given key error will not be shown
            if keyboard.is_pressed('i'):  # if key 'i' is pressed
                # print('You Pressed i!')
                forward_velocity = 0.3
            if keyboard.is_pressed('k'):  # if key 'k' is pressed
                # print('You Pressed k!')
                forward_velocity = -0.3
            if keyboard.is_pressed('l'):  # if key 'l' is pressed
                # print('You Pressed l!')
                left_velocity = -0.3
            if keyboard.is_pressed('j'):  # if key 'j' is pressed
                # print('You Pressed j!')
                left_velocity = 0.3
            if keyboard.is_pressed('u'):  # if key 'u' is pressed
                # print('You Pressed u!')
                yaw_rate = -50
            if keyboard.is_pressed('o'):  # if key 'o' is pressed
                # print('You Pressed o!')
                yaw_rate = 50
            if keyboard.is_pressed('s'):  # if key 's' is pressed
                return None
            return [forward_velocity, left_velocity, yaw_rate, altitude]
        except:
            return [forward_velocity, left_velocity, yaw_rate, altitude]

    cflib.crtp.init_drivers()

    le = Logger(uri)
    cf = le._cf

    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')
    time.sleep(2)

    map = Map()
    # map.display_cell_map()
    map.display_map_using_cv()
    # The Crazyflie lib doesn't contain anything to keep the application alive,
    # so this is where your application should do something. In our case we
    # are just waiting until we are disconnected.

    etat = 'Taking_off_1'
    while le.is_connected :
        try :
            time.sleep(0.01)

            if etat == 'Taking_off_1':
                taking_off_drone(cf)
                etat = 'Go_landing_zone'
                cf.commander.send_position_setpoint(0,0,0.4, 0)
                # etat = 'Go_landing_zone'

            if etat == 'Keyboard':
                command = action_from_keyboard()
                cf.commander.send_hover_setpoint(command[0], command[1], command[2], 0.4)
                if command == None:
                    etat = 'Landing_final'
                map.update_map(le.states)
                map.display_map_using_cv(le.states)

            if etat == 'Go_landing_zone':
                command = action_from_keyboard()
                if command == None:
                    etat = 'Landing_final'
                le.HP_filter(le.states['range.zrange'])
                # cf.commander.send_hover_setpoint(0, 0, 0, 0.4)
                map.update_map(le.states)
                start_cell = map.cell_from_pos([le.states["stateEstimate.x"], le.states["stateEstimate.y"]])
                # start_cell = map.cell_from_pos([le.states["stateEstimate.x"] + 2.5, 1.5 + le.states["stateEstimate.y"]])
                original_target = map.cell_from_pos((3.5 - map.x_start_pos, 1.5 - map.y_start_pos))
                if map.grown_map[original_target] < 0:
                    etat = 'Search_landing_pad'
                map.perform_a_star(start_cell, original_target)
                if len(map.optimal_cell_path) > 1:
                    target_pos = map.simplify_path()
                    cf.commander.send_position_setpoint(target_pos[0] ,
                                                        target_pos[1] ,
                                                        0.4,
                                                        0)
                else:
                    etat = 'Search_landing_pad' 
                    print(etat)
                time.sleep(0.05)

            if etat == 'Search_landing_pad':
                # manual override
                command = action_from_keyboard()
                if command == None:
                    etat = 'Landing_final'

                # updating
                is_on_step = le.HP_filter(le.states['range.zrange'])
                map.update_map(le.states)
                map.update_height_map(le.states,is_on_step)
                
                start_cell = map.cell_from_pos([le.states["stateEstimate.x"], le.states["stateEstimate.y"]])

                map.perform_a_star(start_cell, map.get_current_waypoint())

                if len(map.optimal_cell_path) > 1:

                    target_pos = map.simplify_path()
                    cf.commander.send_position_setpoint(target_pos[0] ,
                                                        target_pos[1] ,
                                                        0.4,
                                                        0)
                elif len(map.optimal_cell_path) == 1 : 

                    map.perform_a_star(start_cell, map.get_next_waypoint())
                    target_pos = map.simplify_path()
                    cf.commander.send_position_setpoint(target_pos[0] ,
                                                        target_pos[1] ,
                                                        0.4,
                                                        0)

                if is_on_step :
                    etat = 'Landing_1'
                else:
                    time.sleep(0.05)
                #etat = 'Search_center_pad'

            if etat == 'Search_center_pad':
                etat = 'Landing_1'

            if etat == 'Landing_1':
                landing_drone(cf)
                etat = 'Taking_off_2'

            if etat == 'Taking_off_2':
                taking_off_drone(cf)
                etat = 'Go_starting_point'

            if etat == 'Go_starting_point':
                #manual override
                command = action_from_keyboard()
                if command == None:
                    etat = 'Landing_final'
                
                map.update_map(le.states)
                start_cell = map.cell_from_pos([le.states["stateEstimate.x"], le.states["stateEstimate.y"]])
                end_cell = map.cell_from_pos([0,0])
                map.perform_a_star(start_cell, end_cell)
                if len(map.optimal_cell_path) > 1:
                    target_pos = map.simplify_path()
                    cf.commander.send_position_setpoint(target_pos[0] ,
                                                        target_pos[1] ,
                                                        0.4,
                                                        0)
                else:
                    cf.commander.send_position_setpoint(0.0 ,
                                                        0.0 ,
                                                        0.4,
                                                        0)
                    etat = 'Landing_final'
                    #need to refind the grid 
                    time.sleep(0.3)
                time.sleep(0.1)
            
            if etat == 'Landing_final':
                landing_drone(cf)
                etat = 'Finish'

            if etat == 'Finish':
                cf.commander.send_stop_setpoint()
                le.is_connected = False
                break

            map.display_map_using_cv(le.states)
        except Exception as e: 
            landing_drone(cf)
            etat = 'Finish'
            le.is_connected = False
            print(e)

