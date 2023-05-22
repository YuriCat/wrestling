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
sys.path.append('.')
sys.path.append('..')
from ..utils.accelerometer import Accelerometer
from ..utils.motion_library import MotionLibrary
from ..utils.current_motion_manager import CurrentMotionManager


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

        self.status = 'DEFAULT'

        # load motion files
        self.current_motion = CurrentMotionManager()
        self.library = MotionLibrary()

    def run(self):
        self.leds['right'].set(0x0000ff)
        self.leds['left'].set(0x0000ff)

        self.current_motion.set(self.library.get('Stand'))
        self.fsm.transition_to('BLOCKING_MOTION')

        while self.step(self.time_step) != -1:
            self.detect_fall()
            if self.status == 'FRONT_FALL':
                self.current_motion.set(self.library.get('GetUpFront'))
                self.status = 'BLOCKING'
            elif self.status == 'BACK_FALL':
                self.current_motion.set(self.library.get('GetUpBack'))
                self.status = 'BLOCKING'
            elif self.status == 'BLOCKING':
                if self.current_motion.is_over():
                    self.status = 'DEFAULT'

            if self.status == 'DEFAULT':
                if self.current_motion.get() != self.library.get('ForwardLoop'):
                    self.current_motion.set(self.library.get('ForwardLoop'))

    def detect_fall(self):
        '''Detect a fall and update the FSM state.'''
        [acc_x, acc_y, _] = self.accelerometer.get_new_average()
        if acc_x < -7:
            self.status = 'FRONT_FALL'
        elif acc_x > 7:
            self.status = 'BACK_FALL'
        if acc_y < -7:
            # Fell to its right, pushing itself on its back
            self.RShoulderRoll.setPosition(-1.2)
        elif acc_y > 7:
            # Fell to its left, pushing itself on its back
            self.LShoulderRoll.setPosition(1.2)

# create the Robot instance and run main loop
wrestler = Wrestler()
wrestler.run()
