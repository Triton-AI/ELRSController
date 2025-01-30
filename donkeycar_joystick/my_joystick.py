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
            0x0 : 'steering',
            0x4 : 'acceleration',
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
            'SE_2' : self.toggle_mode,
            'SE_3' : self.toggle_manual_recording,
        }


        self.axis_trigger_map = {
            'steering' : self.set_steering,
            'acceleration' : self.set_throttle,
        }