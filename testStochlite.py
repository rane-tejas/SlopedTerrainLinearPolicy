import sys, os
import gym_sloped_terrain.envs.stochlite_pybullet_env as e
import argparse
from fabulous.color import blue,green,red,bold
import gym
import pybullet as p
import numpy as np
import time
import math
import matplotlib.pyplot as plt
import matplotlib
# matplotlib.use( 'tkagg' )
PI = np.pi


if (__name__ == "__main__"):
	parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
	
	parser.add_argument('--PolicyDir', help='directory of the policy to be tested', type=str, default='23July3')
	parser.add_argument('--FrontMass', help='mass to be added in the first', type=float, default=0)
	parser.add_argument('--BackMass', help='mass to be added in the back', type=float, default=0)
	parser.add_argument('--FrictionCoeff', help='foot friction value to be set', type=float, default=0.8)
	parser.add_argument('--WedgeIncline', help='wedge incline degree of the wedge', type=int, default=0)
	parser.add_argument('--WedgeOrientation', help='wedge orientation degree of the wedge', type=float, default=0)
	parser.add_argument('--MotorStrength', help='maximum motor Strength to be applied', type=float, default=7.0)
	parser.add_argument('--RandomTest', help='flag to sample test values randomly ', type=bool, default=False)
	parser.add_argument('--seed', help='seed for the random sampling', type=float, default=100)
	parser.add_argument('--EpisodeLength', help='number of gait steps of a episode', type=int, default=1000)
	parser.add_argument('--PerturbForce', help='perturbation force to applied perpendicular to the heading direction of the robot', type=float, default=0.0)
	parser.add_argument('--Downhill', help='should robot walk downhill?', type=bool, default=False)
	parser.add_argument('--Stairs', help='test on staircase', type=bool, default=False)
	parser.add_argument('--AddImuNoise', help='flag to add noise in IMU readings', type=bool, default=False)

	args = parser.parse_args()
	# policy = np.load("experiments/"+args.PolicyDir+"/iterations/policy_18.npy")
	# print(policy)

	WedgePresent = True

	if(args.WedgeIncline == 0 or args.Stairs):
		WedgePresent = False
	elif(args.WedgeIncline <0):
		args.WedgeIncline = -1*args.WedgeIncline
		args.Downhill = True
	env = e.StochliteEnv(render=True, wedge=WedgePresent, stairs = args.Stairs, downhill= args.Downhill, seed_value=args.seed,
				      on_rack=False, gait = 'trot',IMU_Noise=args.AddImuNoise)


	if(args.RandomTest):
		env.Set_Randomization(default=False)
	else:
		env.incline_deg = args.WedgeIncline
		env.incline_ori = math.radians(args.WedgeOrientation)
		env.SetFootFriction(args.FrictionCoeff)
		# env.SetLinkMass(0,args.FrontMass)
		# env.SetLinkMass(11,args.BackMass)
		env.clips = args.MotorStrength
		env.pertub_steps = 300
		env.y_f = args.PerturbForce
	state = env.reset()


	print (
	bold(blue("\nTest Parameters:\n")),
	green('\nWedge Inclination:'),red(env.incline_deg),
	green('\nWedge Orientation:'),red(math.degrees(env.incline_ori)),
	green('\nCoeff. of friction:'),red(env.friction),
	# green('\nMass of the front half of the body:'),red(env.FrontMass),
	# green('\nMass of the rear half of the body:'),red(env.BackMass),
	green('\nMotor saturation torque:'),red(env.clips))

	# Simulation starts
	simstep = 1000
	plot_data = []
	t_r = 0
	for i_step in range(simstep):
		action = np.array( [0.0, 0.0, 0.0, 0.0,
		                    -0.05, 0.05, -0.05, 0.05,
							0.0, 0.0, 0.0, 0.0,
							0.0, 0.0, 0.0])
		
		# action = np.array( [-1.0, -1.0, -1.0, -1.0,
		#                     0.0, 0.0, 0.0, 0.0,
		#                     -0.05, -0.05, -0.05, -0.05,
		#                     0.3, 0.3, 0.3, 0.3,
		# 					0.0, 0.0, 0.0, 0.0])
							
		# action = np.array( [0.6, 0.6, 0.6, 0.6, 
		# 					0.0, 0.0, 0.0, 0.0,
	    #                 	-0.06, -0.06, -0.06, -0.06,
		#                     0.2, 0.2, 0.2, 0.2,
		#                     0.0, 0.0, 0.0, 0.0])

		# action = np.array( [0.0, 0.0, 0.0, 0.0, 
		# 					0.0, 0.0, 0.0, 0.0,
	    #                 	0.0, 0.0, 0.0, 0.0,
		#                     -0.5, 0.5, -0.5, 0.5,
		#                     0.0, 0.0, 0.0, 0.0])

		# action = np.array( [0.3, 0.3, 0.3, 0.3, 
		# 					0.0, 0.0, 0.0, 0.0,
	    #                 	-0.08, -0.08, -0.08, -0.08,
		#                     -0.2, 0.2, -0.2, 0.2,
		#                     0.0, 0.0, 0.0, 0.0])
		
		# action = np.array( [-1.0, -1.0, -1.0, -1.0, 
		# 					0.0, 0.0, 0.0, 0.0,
	    #                 	0.0, 0.0, 0.0, 0.0,
		#                     0.4, 0.4, 0.4, 0.4,
		#                     0.0, 0.0, 0.0, 0.0])

		# action = np.array( [-0.46962743, -1.1188081, 0.11541046, -1.65040833,
		# 					-0.27461176, 0.09398115, 1.04455368, 0.1648822,
		# 					-1.26537848, 1.53735276, -1.40197694, 0.18441505,
		# 					2.04760108, 1.23218077, 0.73039901, 0.5590525, 
		# 					-1.57862117, 0.47634525, 2.31610461, -1.42022835])
		# action = policy.dot(state)
		# plot_data.append(action)
		env.updateCommands(i_step, simstep)
		state, r, _, angle = env.step(action)
		# t_r +=r
		plot_data.append(r)
		# print("Plot data", plot_data)
	
	# roll_r = [p[0] for p in plot_data]
	# pitch_r = [p[1] for p in plot_data]
	# yaw_r = [p[2] for p in plot_data]
	# height_r = [p[3] for p in plot_data]
	# stepx_r = [p[4] for p in plot_data]
	# stepy_r = [p[5] for p in plot_data]
	# roll = [p[6]* 180/PI for p in plot_data] 
	# pitch = [p[7]* 180/PI for p in plot_data] 
	# total_r = [p[8] for p in plot_data]

	# plt.plot(roll_r, label = "roll reward")
	# plt.plot(pitch_r, label = "pitch reward")
	# plt.plot(yaw_r, label = "yaw reward")
	# plt.plot(height_r, label = "height reward")
	# plt.plot(stepx_r, label = "step_x reward")
	# plt.plot(stepy_r, label = "step_y reward")
	# plt.plot(roll, label = "roll")
	# plt.plot(pitch, label = "pitch")
	# plt.plot(total_r, label = "total reward")
	# plt.legend()
	# plt.show()

	cmd_xvel = [p[0] for p in plot_data]
	cmd_yvel = [p[1] for p in plot_data]
	x_vel = [p[2] for p in plot_data]
	y_vel = [p[3] for p in plot_data]
	t_r = [p[4] for p in plot_data]
	plt.figure(1)
	plt.plot(cmd_xvel, label = "command vel x")
	plt.plot(cmd_yvel, label = "command vel y")
	plt.plot(x_vel, label = "x vel")
	plt.plot(y_vel, label = "y vel")
	plt.legend()
	plt.show()

	#Plotting only for FL
	# sl = [p[0] for p in plot_data]
	# sa = [p[4] for p in plot_data]
	# xs = [p[8] for p in plot_data]
	# ys = [p[12] for p in plot_data]
	# zs = [p[16] for p in plot_data]
	# plt.plot(sl, label = "Step Length")
	# plt.plot(sa, label = "Steer Angle")
	# plt.plot(xs, label = "X Shift")
	# plt.plot(ys, label = "Y Shift")
	# plt.plot(zs, label = "Z Shift")
	# plt.legend()
	# plt.show()

	# print("Total Reward:", t_r)