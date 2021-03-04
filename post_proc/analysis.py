import sys, os
import argparse
import matplotlib.pyplot as plt
import numpy as np

if __name__ == "__main__":
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)

    parser.add_argument('--PolicyDir', help='directory of the policy to be visualised', type=str, default='26Feb3')
    parser.add_argument('--par', help='parameter of the policy to be visualized (SL, SA, X, Y, Z)', type=str, default='SL')
    parser.add_argument('--obs', help='observation to be plotted against (All, R, P, Y, SP)', type=str, default='All')
    parser.add_argument('--multi', help='plot 2 policies together', type=int, default=0)
    parser.add_argument('--PolName', help='Policy Name', type=str, default='policy_18')

    args = parser.parse_args()

    alpha_val = [0.8, 0.4]

    if args.multi:
        x = input("Input 2 policies for comparison: ")
        x = x.split()

    for i in range(args.multi+1):
        if args.multi:
            args.PolName = x[i]
        
        policy = np.load("../experiments/"+args.PolicyDir+"/iterations/"+args.PolName+".npy")
        # print(policy)
        sec_policy = []

        if args.obs == 'All':
            params = ['R(t-2)', 'P(t-2)', 'Y(t-2)', 'R(t-1)', 'P(t-1)', 'Y(t-1)', 'R(t)', 'P(t)', 'Y(t)', 'SP_R', 'SP_P']
            sec_policy = policy

        elif args.obs == 'R':
            params = ['R(t-2)', 'R(t-1)', 'R(t)']
            for row in policy:
                sec_policy.append([row[0], row[3], row[6]])

        elif args.obs == 'P':
            params = ['P(t-2)', 'P(t-1)', 'P(t)']
            for row in policy:
                sec_policy.append([row[1], row[4], row[7]])

        elif args.obs == 'Y':
            params = ['Y(t-2)', 'Y(t-1)', 'Y(t)']
            for row in policy:
                sec_policy.append([row[2], row[5], row[8]])

        elif args.obs == 'SP':
            params = ['SP_R', 'SP_P']
            for row in policy:
                sec_policy.append([row[9], row[10]])

        if args.par == 'SL':
            plt.subplot(2,2,1)
            plt.bar(params, sec_policy[0], alpha = alpha_val[i])
            plt.title("FL")

            plt.subplot(2,2,2)
            plt.bar(params, sec_policy[1], alpha = alpha_val[i])
            plt.title("FR")

            plt.subplot(2,2,3)
            plt.bar(params, sec_policy[2], alpha = alpha_val[i])
            plt.title("BL")

            plt.subplot(2,2,4)
            plt.bar(params, sec_policy[3], alpha = alpha_val[i])
            plt.title("BR")

            plt.suptitle("Step Length")
            # plt.show()

        elif args.par == 'SA':
            plt.subplot(2,2,1)
            plt.bar(params, sec_policy[4], alpha = alpha_val[i])
            plt.title("FL")

            plt.subplot(2,2,2)
            plt.bar(params, sec_policy[5], alpha = alpha_val[i])
            plt.title("FR")

            plt.subplot(2,2,3)
            plt.bar(params, sec_policy[6], alpha = alpha_val[i])
            plt.title("BL")

            plt.subplot(2,2,4)
            plt.bar(params, sec_policy[7], alpha = alpha_val[i])
            plt.title("BR")

            plt.suptitle("Steer Angle")
            # plt.show()

        elif args.par == 'X':
            plt.subplot(2,2,1)
            plt.bar(params, sec_policy[8], alpha = alpha_val[i])
            plt.title("FL")

            plt.subplot(2,2,2)
            plt.bar(params, sec_policy[9], alpha = alpha_val[i])
            plt.title("FR")

            plt.subplot(2,2,3)
            plt.bar(params, sec_policy[10], alpha = alpha_val[i])
            plt.title("BL")

            plt.subplot(2,2,4)
            plt.bar(params, sec_policy[11], alpha = alpha_val[i])
            plt.title("BR")

            plt.suptitle("X Shift")
            # plt.show()

        elif args.par == 'Y':
            plt.subplot(2,2,1)
            plt.bar(params, sec_policy[12], alpha = alpha_val[i])
            plt.title("FL")

            plt.subplot(2,2,2)
            plt.bar(params, sec_policy[13], alpha = alpha_val[i])
            plt.title("FR")

            plt.subplot(2,2,3)
            plt.bar(params, sec_policy[14], alpha = alpha_val[i])
            plt.title("BL")

            plt.subplot(2,2,4)
            plt.bar(params, sec_policy[15], alpha = alpha_val[i])
            plt.title("BR")

            plt.suptitle("Y Shift")
            # plt.show()

        elif args.par == 'Z':
            plt.subplot(2,2,1)
            plt.bar(params, sec_policy[16], alpha = alpha_val[i])
            plt.title("FL")

            plt.subplot(2,2,2)
            plt.bar(params, sec_policy[17], alpha = alpha_val[i])
            plt.title("FR")

            plt.subplot(2,2,3)
            plt.bar(params, sec_policy[18], alpha = alpha_val[i])
            plt.title("BL")

            plt.subplot(2,2,4)
            plt.bar(params, sec_policy[19], alpha = alpha_val[i])
            plt.title("BR")

            plt.suptitle("Z Shift")
            # plt.show()

    # plt.legend()
    plt.show()