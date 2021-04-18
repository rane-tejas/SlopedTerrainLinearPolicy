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
import matplotlib.pyplot as plt

PI = np.pi
no_of_points = 100

@dataclass
class leg_data:
    name: str
    ID: int
    theta: float = 0.0
    prev_motor_hip: float = 0.0
    prev_motor_knee: float = 0.0
    prev_motor_abd: float = 0.0
    motor_hip: float = 0.0
    motor_knee: float = 0.0
    motor_abd: float = 0.0
    prev_x: float = 0.0
    prev_y: float = 0.0
    prev_z: float = 0.0
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
    def __init__(self, gait_type='trot', phase=[0, 100, 100, 0]):
        self.gait_type = gait_type
        self.phase = robot_data(front_left=phase[0], front_right=phase[1], back_left=phase[2], back_right=phase[3])
        self.frequency = 2.5
        # self.omega = 2 * PI * self.frequency
        self.omega = 2 * no_of_points * self.frequency
        self.theta = 0

        self.front_left = leg_data('FL', 1)
        self.front_right = leg_data('FR', 2)
        self.back_left = leg_data('BL', 3)
        self.back_right = leg_data('BR', 4)

        self.robot_width = 0.192
        self.robot_length = 0.334
        self.link_lengths_stochlite = [0.096, 0.146, 0.172]
        self.stochlite_kin = StochliteKinematics()

        self.foot_clearance = 0.06
        self.walking_height = -0.25
        self.swing_time = 0.25
        self.max_linear_xvel = 0.4 #0.4, made zero for only ang vel # calculation is < 0.2 m steplength times the frequency 2.5 Hz
        self.max_linear_yvel = 0.25 #0.25, made zero for only ang vel # calculation is < 0.14 m times the frequency 2.5 Hz
        self.max_ang_vel = 3.5 #considering less than pi/2 steer angle # less than one complete rotation in one second

    def update_leg_theta(self, dt):
        '''
        Function to calculate the leg cycles of the trajectory, depending on the gait.
        '''

        # def constrain_theta(theta):
        #     theta = np.fmod(theta, 2 * PI)
        #     if (theta < 0):
        #         theta = theta + 2 * PI
        #     return theta

        def constrain_theta(theta):
            theta = np.fmod(theta, 2 * no_of_points)
            if (theta < 0):
                theta = theta + 2 * no_of_points
            return theta

        self.theta = constrain_theta(self.theta + self.omega * dt) 

        # self.front_left.theta = constrain_theta(self.theta + self.phase.front_left) 
        # self.front_right.theta = constrain_theta(self.theta + self.phase.front_right)
        # self.back_left.theta = constrain_theta(self.theta + self.phase.back_left)
        # self.back_right.theta = constrain_theta(self.theta + self.phase.back_right)

        #converting no of points notation to PI
        self.front_left.theta = constrain_theta(self.theta + self.phase.front_left) * (2*PI)/(2*no_of_points) 
        self.front_right.theta = constrain_theta(self.theta + self.phase.front_right) * (2*PI)/(2*no_of_points)
        self.back_left.theta = constrain_theta(self.theta + self.phase.back_left) * (2*PI)/(2*no_of_points)
        self.back_right.theta = constrain_theta(self.theta + self.phase.back_right) * (2*PI)/(2*no_of_points)

    def reset_theta(self):
        '''
        Function to reset the main cycle of the trajectory.
        '''

        self.theta = 0

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

    def initialize_prev_motor_ang(self, prev_motor_angles):
        '''
        Initialize motor angles of previous time-step for each leg
        '''

        self.front_left.prev_motor_hip = prev_motor_angles[0]
        self.front_left.prev_motor_knee = prev_motor_angles[1]
        self.front_left.prev_motor_abd = prev_motor_angles[8]
     
        self.front_right.prev_motor_hip = prev_motor_angles[2]
        self.front_right.prev_motor_knee = prev_motor_angles[3]
        self.front_right.prev_motor_abd = prev_motor_angles[9]

        self.back_left.prev_motor_hip = prev_motor_angles[4]
        self.back_left.prev_motor_knee = prev_motor_angles[5]
        self.back_left.prev_motor_abd = prev_motor_angles[10]

        self.back_right.prev_motor_hip = prev_motor_angles[6]
        self.back_right.prev_motor_knee = prev_motor_angles[7]
        self.back_right.prev_motor_abd = prev_motor_angles[11]

    def foot_step_planner(self, leg, v_leg):
        s = v_leg * self.swing_time
        x = s[0] + leg.x_shift
        y = s[1] + leg.y_shift
        z = s[2] + leg.z_shift

        # print("FSP,", x, y, z)

        return [x, y, z]

    def calculate_planar_traj(self, leg, v_x, v_y, w_z, dt):
        '''
        Calculates the x and y component of the trajectory based on the commanded velocities (either from joystick or augmented by the policy).
        Args:
            leg : trajectory cycle parameter
            v_x : trajectory modulation parameter predicted by the policy
            v_y : boolean defining swing phase and stance phase of the leg
            w_z : boolean defining swing phase and stance phase of the leg
            dt  : boolean defining swing phase and stance phase of the leg
        Ret:
            z   : calculated z component of the trajectory.
        '''
        
        cmd_lvel = np.array([v_x, v_y, 0])
        cmd_avel = np.array([0, 0, w_z])
        v_leg = [0, 0, 0]
        next_step = [0, 0, 0]
        swing_vec = [0, 0, 0]

        if (leg.name == "FL"):
            leg_frame = [+self.robot_length/2, +self.robot_width/2, 0]
        elif (leg.name == "FR"):
            leg_frame = [+self.robot_length/2, -self.robot_width/2, 0]
        elif (leg.name == "BL"):
            leg_frame = [-self.robot_length/2, +self.robot_width/2, 0]
        elif (leg.name == "BR"):
            leg_frame = [-self.robot_length/2, -self.robot_width/2, 0]
        
        prev_foot_pos = np.array([leg.prev_x, leg.prev_y, 0])
        prev_r = prev_foot_pos + np.array(leg_frame)

        v_lcomp = cmd_lvel
        v_acomp = np.cross(cmd_avel, prev_r)
        v_leg = v_lcomp + v_acomp

        if leg.theta > PI: # theta taken from +x, CW # Flip this sign if the trajectory is mirrored
            flag = -1 #during stance_phase of walking, leg moves backwards to push body forward
            dr = v_leg * dt * flag
            r = prev_r + dr - np.array(leg_frame)
        else:
            flag = 1 #during swing_phase of walking, leg moves forward to next step
            next_step = self.foot_step_planner(leg, v_leg) + np.array(leg_frame)
            swing_vec = next_step - prev_r
            dr = swing_vec * (0.25/100) * flag
            r = prev_r + dr - np.array(leg_frame)


        x = r[0] #+ leg.x_shift
        y = r[1] #+ leg.y_shift

        return [x, y]

    def calculate_vert_comp(self, leg):
        '''
        Calculates the z component of the trajectory. The function for the z component can be changed here.
        The z component calculation is kept independent as it is not affected by the velocity calculations. 
        Various functions can be used to smoothen out the foot impacts while walking.
        Args:
            leg : trajectory cycle parameter
        Ret:
            z   : calculated z component of the trajectory.
        '''

        if leg.theta > PI: # theta taken from +x, CW # Flip this sigh if the trajectory is mirrored
            flag = 0 #z-coordinate of trajectory, during stance_phase of walking
        else:
            flag = 1 #z-coordinate of trajectory, during swing_phase of walking

        z = self.foot_clearance * np.sin(leg.theta) * flag + self.walking_height + leg.z_shift 
        return z

    def initialize_leg_state(self, action, prev_motor_angles, dt):
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

        self.update_leg_theta(dt)

        self.initialize_traj_shift(action[:4], action[4:8], action[8:12])
        self.initialize_prev_motor_ang(prev_motor_angles)

        return legs

    def generate_trajectory(self, action, prev_motor_angles, dt):
        '''
        Velocity based trajectory generator. The controller assumes a default trot gait. 
        Args:
            theta  : trajectory cycle parameter theta
            action : trajectory modulation parameters predicted by the policy
        Ret:
            leg_motor_angles : list of motors positions for the desired action [FLH FLK FRH FRK BLH BLK BRH BRK FLA FRA BLA BRA]
            Note: we are using the right hand rule for the conventions of the leg which is - x->front, y->left, z->up
        '''

        legs = self.initialize_leg_state(action, prev_motor_angles, dt)
        lin_vel_x = action[12] * self.max_linear_xvel
        lin_vel_y = action[13] * self.max_linear_yvel
        ang_vel_z = action[14] * self.max_ang_vel

        for leg in legs:

            [leg.prev_x, leg.prev_y, leg.prev_z] = self.stochlite_kin.forwardKinematics(leg.name, [leg.prev_motor_abd, leg.prev_motor_hip, leg.prev_motor_knee])
            # [leg.prev_x, leg.prev_y, leg.prev_z] = [leg.x, leg.y, leg.z]
            [leg.x, leg.y] = self.calculate_planar_traj(leg, lin_vel_x, lin_vel_y, ang_vel_z, dt)
            leg.z = self.calculate_vert_comp(leg)

            # print(leg)

            branch = "<"
            _,[leg.motor_abd, leg.motor_hip, leg.motor_knee] = self.stochlite_kin.inverseKinematics(leg.name, [leg.x, leg.y, leg.z], branch)

        leg_motor_angles = [legs.front_left.motor_hip, legs.front_left.motor_knee, legs.front_right.motor_hip,
                            legs.front_right.motor_knee,
                            legs.back_left.motor_hip, legs.back_left.motor_knee, legs.back_right.motor_hip,
                            legs.back_right.motor_knee,
                            legs.front_left.motor_abd, legs.front_right.motor_abd,
                            legs.back_left.motor_abd, legs.back_right.motor_abd]
        
        # Plotting in base frame
        fl_foot_pos = [legs.front_left.x + self.robot_length/2, legs.front_left.y + self.robot_width/2, legs.front_left.z]
        fr_foot_pos = [legs.front_right.x + self.robot_length/2, legs.front_right.y - self.robot_width/2, legs.front_right.z]
        bl_foot_pos = [legs.back_left.x - self.robot_length/2, legs.back_left.y + self.robot_width/2, legs.back_left.z]
        br_foot_pos = [legs.back_right.x - self.robot_length/2, legs.back_right.y - self.robot_width/2, legs.back_right.z]

        # Plotting in leg frame
        # fl_foot_pos = [legs.front_left.x, legs.front_left.y, legs.front_left.z]
        # fr_foot_pos = [legs.front_right.x, legs.front_right.y, legs.front_right.z]
        # bl_foot_pos = [legs.back_left.x, legs.back_left.y, legs.back_left.z]
        # br_foot_pos = [legs.back_right.x, legs.back_right.y, legs.back_right.z]
        
        return leg_motor_angles, fl_foot_pos + fr_foot_pos + bl_foot_pos + br_foot_pos

if __name__ == '__main__':
    trajgen = TrajectoryGenerator()
    action = np.array([0.0, 0.0, 0.0, 0.0,
                       0.0, 0.0, 0.0, 0.0,
                       0.0, 0.0, 0.0, 0.0,
                       0.5, 0.0, 0.0])
    
    plotdata = []
    dt = 0.01
    prev_motor_angles = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    ax = plt.axes(projection='3d')

    for i in range(200):
        prev_motor_angles, foot_pos = trajgen.generate_trajectory(action, prev_motor_angles, dt)
        plotdata.append(foot_pos)

    x_fl = [p[0] for p in plotdata]
    y_fl = [p[1] for p in plotdata]
    z_fl = [p[2] for p in plotdata]
    x_fr = [p[3] for p in plotdata]
    y_fr = [p[4] for p in plotdata]
    z_fr = [p[5] for p in plotdata]
    x_bl = [p[6] for p in plotdata]
    y_bl = [p[7] for p in plotdata]
    z_bl = [p[8] for p in plotdata]
    x_br = [p[9] for p in plotdata]
    y_br = [p[10] for p in plotdata]
    z_br = [p[11] for p in plotdata]

    ax.plot3D(x_fl, y_fl, z_fl, 'red')
    ax.plot3D(x_fr, y_fr, z_fr, 'blue')
    ax.plot3D(x_bl, y_bl, z_bl, 'blue')
    ax.plot3D(x_br, y_br, z_br, 'red')

    plt.show()