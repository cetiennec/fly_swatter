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

        self.states['t'] = 0
        self.states['stateEstimate.x'] = 0
        self.states['stateEstimate.y'] = 0
        self.N_filter = 10

        self.states['z_range_hist'] = [0]*self.N_filter

        self.last_time_down = 0
        self.last_time_up = 0

        self.old_measurement = 0
        self.old_filtered = 0
        self.hand_stopped = False

        # for the height thing
        self.obstacle_height = 0.1 #m
        self.hover_height = 0.5 #m
        self.is_on_obstacle = True
        self.desired_height = self.hover_height    

        self.block_callback =  False  

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
        t = Timer(50, self._cf.close_link)
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
        if not(self.block_callback):
            self.block_callback = True
            self.states['dt'] = timestamp - self.states['t']
            self.states['t'] = timestamp

            self.states['vx'] = (data['stateEstimate.x'] - self.states['stateEstimate.x']) / self.states['dt']
            self.states['vy'] = (data['stateEstimate.y'] - self.states['stateEstimate.y']) / self.states['dt']

            for name, value in data.items():
                self.states[name] = value
            # print(self.states['stateEstimate.z'])
            self.height_update()

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
        """Callback when connection initial connection fails (i.e no Crazyflie)
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
    
    def height_update(self) :

        self.states['z_range_hist'].pop(0)
        self.states['z_range_hist'].append(self.states['range.zrange'])
        z_hist = np.array(self.states['z_range_hist'])
        
        # z_amp = np.max(z_hist) - np.min(z_hist)
        # filter = np.array([-1.0]*(self.N_filter//2) + [1.0]*(self.N_filter//2))
        # z_hist = z_hist - np.average(z_hist)
        # result = np.sum(z_hist * filter)/self.N_filter


        # print("VALUE OF STEP",new_value)
        if z_hist[-2] - z_hist[-1] < - 13 and not(self.is_on_obstacle):
            print('FOUND LANDING PAD UP')
            self.is_on_obstacle = True
            self.last_time_up = self.states['t']
            # print(z_hist)
            # print("variance", np.std(z_hist))


        elif z_hist[-2] - z_hist[-1] > + 13 and self.is_on_obstacle :
            print('FOUND LANDING PAD DOWN')
            self.is_on_obstacle = False
            self.last_time_down = self.states['t']
            # print(z_hist)
            # print("variance", np.std(z_hist))



        if self.is_on_obstacle :
            self.desired_height = self.hover_height
        else :
            self.desired_height = self.hover_height

        return self.desired_height
    
    def send_hover_setpoint(self, arg1, arg2, arg3, arg4):
        if not(self.emergency_stop):
            cf.commander.send_hover_setpoint(arg1, arg2, arg3,arg4)

    def send_position_setpoint(self, arg1, arg2, arg3, arg4):
        if not(self.emergency_stop):
            cf.commander.send_position_setpoint(arg1, arg2, arg3,arg4)



if __name__ == '__main__':
    # Initialize the low-level drivers
    # Tools

    cflib.crtp.init_drivers()

    le = Logger(uri)
    cf = le._cf

    def landing_drone(cf):
        for _ in range(20):
            le.send_hover_setpoint(0, 0, 0, le.hover_height)
            time.sleep(0.1)
        for y in range(10):
            le.send_hover_setpoint(0, 0, 0, (9-y)/ 9.0 * le.hover_height)
            time.sleep(0.1)
        for _ in range(10):
            le.send_hover_setpoint(0, 0, 0, 0)
            time.sleep(0.1)


    def taking_off_drone(cf):
        for y in range(10):
            le.send_hover_setpoint(0, 0, 0, y/10.0*le.hover_height)
            time.sleep(0.1)
        for _ in range(20):
            le.send_hover_setpoint(0, 0, 0, le.hover_height)
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

    def find_center_landing_pad(le, cf):
        
        x_cross = le.states['stateEstimate.x'] - le.states['vx'] * (le.states['t'] - le.last_time_up)
        y_cross = le.states['stateEstimate.y'] - le.states['vy'] * (le.states['t'] - le.last_time_up)

        print("Start X shape from position : ", x_cross, y_cross)
        for dx, dy in zip([0.1, -0.1, -0.1, 0.1], [0.1, 0.1, -0.1, -0.1]):
            le.send_hover_setpoint(x_cross + dx, y_cross + dy, 0, le.desired_height)
            print("Travel along : ", dx, dy, "for 1 second")
            time.sleep(1)
            print("Stabilizing on waypoint")
            le.send_hover_setpoint(0, 0, 0, le.desired_height)
            time.sleep(1)
            if le.is_on_obstacle :
                print("Found landing pad for this waypoint")
                break




        while le.is_on_obstacle :
           le.send_hover_setpoint(parsing_speed, 0, 0, le.desired_height) 
        
        # Found top border !
        x_center = le.states['stateEstimate.x'] - le.states['vx'] * (le.states['t'] - le.last_time_up) - length_lp/2.0

        print("Found x_center : ", x_center)

        #back up a bit
        print("Backing up")
        le.send_hover_setpoint(-parsing_speed, 0, 0, le.desired_height)
        time.sleep(1)

        while le.is_on_obstacle :
           le.send_hover_setpoint(0, parsing_speed, 0, le.desired_height) 
        
        y_center = le.states['stateEstimate.y'] - le.states['vy'] * (le.states['t'] - le.last_time_up) - width_lp/2.0

        print("Found y_center : ", y_center)
        print("Converging back to center")
        le.send_position_setpoint(x_center, y_center, 0, le.desired_height)
        time.sleep(0.5)

    


    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')
    time.sleep(2)


    taking_off_drone(cf)

    le.is_on_obstacle = False
    print("Finished take-off")


    for _ in range(400) :
        if le.is_connected:
            try :

                command = action_from_keyboard()
                if command == None:
                    break
                
                le.send_hover_setpoint(command[0], command[1], command[2], le.desired_height)

                # if le.is_on_obstacle :
                #     print("Found landing pad !")
                #     find_center_lp2(le, cf)
                #     landing_drone(cf)
                #     cf.commander.send_stop_setpoint()
                #     le.is_connected = False
                #     break

                time.sleep(0.1)
            
                
            except Exception as e: 
                print("Error ! ")
                landing_drone(cf)
                le.is_connected = False
                print(e)

    if le.is_connected :
        landing_drone(cf)
        cf.commander.send_stop_setpoint()
        le.is_connected = False
