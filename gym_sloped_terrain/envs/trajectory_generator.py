# ### Trajectory Generator
# Written by Shishir Kolathaya shishirk@iisc.ac.in
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Utilities for realizing walking controllers.

Action space is defined as:
action[:4] = x_shift fl fr bl br
action[4:8] = y_shift fl fr bl br
action[8:12] = z_shift fl fr bl br
action[12] = linear x velocity of robot (lin_x_vel)
action[13] = linear y velocity of robot (lin_y_vel)
action[14] = angular velocity of robot about z (ang_z_vel)
"""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
from dataclasses import dataclass
from collections import namedtuple
from utils.ik_class import StochliteKinematics
import numpy as np

PI = np.pi
no_of_points = 100

@dataclass
class leg_data:
    name: str
    ID: int
    theta: float = 0.0
    motor_hip: float = 0.0
    motor_knee: float = 0.0
    motor_abduction: float = 0.0
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    x_shift: float = 0.0
    y_shift: float = 0.0
    z_shift: float = 0.0


@dataclass
class robot_data:
    front_left: leg_data = leg_data('FL', 1)
    front_right: leg_data = leg_data('FR', 2)
    back_left: leg_data = leg_data('BL', 3)
    back_right: leg_data = leg_data('BR', 4)


class TrajectoryGenerator():
    def __init__(self, gait_type='trot', phase=[0, PI, PI, 0]):
        self.gait_type = gait_type
        self._phase = robot_data(front_right=phase[0], front_left=phase[1], back_right=phase[2], back_left=phase[3])

        self.front_left = leg_data('FL', 1)
        self.front_right = leg_data('FR', 2)
        self.back_left = leg_data('BL', 3)
        self.back_right = leg_data('BR', 4)

        self.link_lengths_stochlite = [0.096, 0.146, 0.172]

        self.robot_width = 0.192
        self.robot_length = 0.334
        self.stochlite_kin = StochliteKinematics()

    def update_leg_theta(self, theta):
        '''
        Depending on the gait, the theta for every leg is calculated.
        '''

        def constrain_theta(theta):
            theta = np.fmod(theta, 2 * no_of_points)
            if (theta < 0):
                theta = theta + 2 * no_of_points
            return theta

        self.front_right.theta = constrain_theta(theta + self._phase.front_right)
        self.front_left.theta = constrain_theta(theta + self._phase.front_left)
        self.back_right.theta = constrain_theta(theta + self._phase.back_right)
        self.back_left.theta = constrain_theta(theta + self._phase.back_left)

    def initialize_traj_shift(self, x_shift, y_shift, z_shift):
        '''
        Initialize desired X, Y, Z offsets of trajectory for each leg
        '''

        self.front_left.x_shift = x_shift[0]
        self.front_right.x_shift = x_shift[1]
        self.back_left.x_shift = x_shift[2]
        self.back_right.x_shift = x_shift[3]
        
        self.front_left.y_shift = y_shift[0]
        self.front_right.y_shift = y_shift[1]
        self.back_left.y_shift = y_shift[2]
        self.back_right.y_shift = y_shift[3]
        
        self.front_left.z_shift = z_shift[0]
        self.front_right.z_shift = z_shift[1]
        self.back_left.z_shift = z_shift[2]
        self.back_right.z_shift = z_shift[3]

    def calculate_leg_linvel_comp(self, lin_x_vel, lin_y_vel):

    def calculate_leg_angvel_comp(self, ang_z_vel):

    def initialize_leg_state(self, theta, action):
        '''
        Initialize all the parameters of the leg trajectories
        Args:
            theta  : trajectory cycle parameter theta
            action : trajectory modulation parameters predicted by the policy
        Ret:
            legs   : namedtuple('legs', 'front_right front_left back_right back_left')
        '''

        Legs = namedtuple('legs', 'front_left front_right back_left back_right')
        legs = Legs(front_left=self.front_left, front_right=self.front_right,
                    back_left=self.back_left,  back_right=self.back_right)

        self.update_leg_theta(theta)

        self.initialize_traj_shift(action[:4], action[4:8], action[8:12])

        self.calculate_leg_linvel_comp(action[12], action[13])
        self.calculate_leg_angvel_comp(action[14])

        return legs

    def generate_traj(self, theta, action, motor_angles):
        '''
        Velocity based tragectory generator
        Args:
            theta  : trajectory cycle parameter theta
            action : trajectory modulation parameters predicted by the policy
        Ret:
            leg_motor_angles : list of motors positions for the desired action [FLH FLK FRH FRK BLH BLK BRH BRK FLA FRA BLA BRA]
            Note: we are using the right hand rule for the conventions of the leg which is - x->front, y->left, z->up
        '''