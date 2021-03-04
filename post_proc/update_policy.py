import sys, os
import numpy as np
import argparse

if __name__ == '__main__':
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)

    parser.add_argument('--PolicyDir', help='directory of the policy to be visualised', type=str, default='26Feb3')
    parser.add_argument('--PolName', help='Policy Name', type=str, default='policy_18')
    parser.add_argument('--SaveAs', help='Updated policy to be saved as "filename".npy', type=str, default='updated_policy')

    args = parser.parse_args()
    temp_policy = np.load("../experiments/"+args.PolicyDir+"/iterations/"+args.PolName+".npy")
	# print(policy)

    path = "../experiments/"+args.PolicyDir+"/iterations/"+args.SaveAs
    policy = np.copy(temp_policy)

    while True:
        print(temp_policy)
        choice = input("Enter choice - (c)hange/(u)ndo/(s)ave/(e)xit: ")

        if choice == "e" or choice == "exit":
            break
        
        elif choice == "s"or choice == "save":
            confirm = input("Confirm save changes - y/n: ")
            if confirm == "y":
                policy = temp_policy
                np.save(path, policy)
            else:
                print("oops!")
                continue

        elif choice == "u" or choice == "undo":
            temp_policy = policy
            print("changes reverted")
            continue

        elif choice == "c" or choice == "choice":
            edit = input("change what? - (r)ow/(c)ol/(e)le: ")
            if edit == "r" or choice == "row":
                row = int(input("row number: "))
                new_val = int(input("New Value: "))
                for i in range(11):
                    temp_policy[row][i] = new_val
            
            elif edit == "c" or choice == "col":
                col = int(input("col number: "))
                new_val = int(input("New Value: "))
                for i in range(20):
                    temp_policy[i][col] = new_val

            elif edit == "e" or choice == "ele": 
                leg = int(input("choose leg - fl(1)\tfr(2)\tbl(3)\tbr(4): "))
                param = input("choose parameter - sl/sa/x/y/z: ")
                col = int(input("choose obs - r2(0)\tp2(1)\ty2(2)\tr1(3)\tp1(4)\ty1(5)\tr(6)\tp(7)\ty(8)\tsr(9)\tsp(10): "))

                if param == "sl":
                    row = 0 + leg - 1
                elif param == "sa":
                    row = 4 + leg - 1
                elif param == "x":
                    row = 8 + leg - 1
                elif param == "y":
                    row = 12 + leg - 1
                elif param == "z":
                    row = 16 + leg - 1
                else:
                    continue

                print("Current Value: ", temp_policy[row][col])
                new_val = float(input("New value: "))
                temp_policy[row][col] = new_val
        
        else:
            continue