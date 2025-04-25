from donkeycar.parts.controller import Joystick, JoystickController


class MyJoystick(Joystick):
    #An interface to a physical joystick available at /dev/input/js0
    def __init__(self, *args, **kwargs):
        super(MyJoystick, self).__init__(*args, **kwargs)


        self.button_names = {
            0x100 : 'SA',
            0x101 : 'SD',
            0x102 : 'SE_2',
            0x103 : 'SE_3',
            0x104 : 'SE_4',
            0x105 : 'SE_5',
            0x106 : 'SE_6',
            0x107 : 'SE_7',
            0x108 : 'SE_8',
            0x109 : 'SE_9',
            0x10a : 'SE_10',
        }


        self.axis_names = {
            0x0 : '0',
            0x1 : '1',
            0x3 : '3',
            0x4 : '4',
        }



class MyJoystickController(JoystickController):
    #A Controller object that maps inputs to actions
    def __init__(self, *args, **kwargs):
        super(MyJoystickController, self).__init__(*args, **kwargs)


    def init_js(self):
        #attempt to init joystick
        try:
            self.js = MyJoystick(self.dev_fn)
            self.js.init()
        except FileNotFoundError:
            print(self.dev_fn, "not found.")
            self.js = None
        return self.js is not None


    def init_trigger_maps(self):
        #init set of mapping from buttons to function calls

        self.button_down_trigger_map = {
            'SA' : self.erase_last_N_records,   #left button on top
            'SD' : self.emergency_stop,         #right button on top
            # note that back is the position of pressing the part closer to the operator down
            # forward is clicked the far part doww
            # 'SE_3' : self.toggle_mode,               #left forward  right middle
            # 'SE_4' : self.increase_max_throttle,     #left forward right back
            # 'SE_8' : self.decrease_max_throttle,     #left back right forward
            # 'SE_10' : self.toggle_constant_throttle, #left back right back
            # 'SE_2' : self.toggle_manual_recording,   #left forward back forward
        }


        self.axis_trigger_map = {
            '3' : self.set_steering,
            '1' : self.set_throttle,
        }
