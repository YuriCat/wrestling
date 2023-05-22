# Copyright 1996-2023 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Minimalist controller example for the Robot Wrestling Tournament.
   Demonstrates how to play a simple motion file."""

from controller import Robot
import sys

# We provide a set of utilities to help you with the development of your controller. You can find them in the utils folder.
# If you want to see a list of examples that use them, you can go to https://github.com/cyberbotics/wrestling#demo-robot-controllers
sys.path.append('..')
from utils.accelerometer import Accelerometer
from utils.motion_library import MotionLibrary
from utils.current_motion_manager import CurrentMotionManager
from utils.camera import Camera


class Wrestler(Robot):
    def __init__(self):
        Robot.__init__(self)

        # retrieves the WorldInfo.basicTimeTime (ms) from the world file
        self.time_step = int(self.getBasicTimeStep())
        self.accelerometer = Accelerometer(self, self.time_step)
        self.leds = {
            'right': self.getDevice('Face/Led/Right'),
            'left': self.getDevice('Face/Led/Left')
        }

        # Shoulder roll motors for getting up from a side fall
        self.RShoulderRoll = self.getDevice('RShoulderRoll')
        self.LShoulderRoll = self.getDevice('LShoulderRoll')

        # load motion files
        self.current_motion = CurrentMotionManager()
        self.library = MotionLibrary()

    def run(self):
        self.leds['right'].set(0x0000ff)
        self.leds['left'].set(0x0000ff)

        status = 'DEFAULT'
        count = 0
        camera = Camera()
        already_fall = False

        self.current_motion.set(self.library.get('ForwardLoop'))

        while self.step(self.time_step) != -1:
            prev_status = status
            fall_status = self.detect_fall()
            if fall_status is not None:
                status = fall_status
                already_fall = True

            if status == 'FRONT_FALL':
                self.current_motion.set(self.library.get('GetUpFront'))
                status = 'BLOCKING'
            elif status == 'BACK_FALL':
                self.current_motion.set(self.library.get('GetUpBack'))
                status = 'BLOCKING'
            elif status == 'BLOCKING':
                if self.current_motion.is_over():
                    status = 'DEFAULT'

            if status == 'DEFAULT':
                if not already_fall:
                    self.current_motion.set(self.library.get('ForwardLoop'))
                else:
                    self.current_motion.set(self.library.get('TurnLeft180'))
                count += 1

    def detect_fall(self):
        '''Detect a fall and update the FSM state.'''
        next_status = None
        [acc_x, acc_y, _] = self.accelerometer.get_new_average()
        if acc_x < -7:
            next_status = 'FRONT_FALL'
        elif acc_x > 7:
            next_status = 'BACK_FALL'
        if acc_y < -7:
            # Fell to its right, pushing itself on its back
            self.RShoulderRoll.setPosition(-1.2)
        elif acc_y > 7:
            # Fell to its left, pushing itself on its back
            self.LShoulderRoll.setPosition(1.2)
        return next_status

# create the Robot instance and run main loop
wrestler = Wrestler()
wrestler.run()
