import numpy as np
import matplotlib.pyplot as plt

def lmake_xy( dir_file):
        xy = np.genfromtxt(dir_file, delimiter='', dtype = np.float64)
        # print(xy.shape)
        return xy

data = lmake_xy("/home/kist/euncheol/Dual-arm/data/Sim_data/2dof_mpc.txt")

ref = data[:,0:2] * 180/np.pi
q = data[:,2:4] * 180/np.pi
goal = data[:,-2:] * 180/np.pi

plt.figure(figsize=(20,10))
for i in range(2):
        plt.subplot(2,1,i+1)
        # plt.plot(ref[:,i], label = "ref")
        # # plt.plot(pred_q[:,i], '-.', label = "pred")
        # plt.plot(q[:,i], ':', label = "q_MPC")
        # plt.plot(q2[:,i], ':', label = "q_MPC_ref_step")
        # plt.plot(ref_q2[:,i], ':', label = "q_MPC_ref_step_ref")
        # plt.plot(q1[:,i],  label = "q_JS")
        plt.plot(goal[:,i], label = "goal")
        plt.plot(ref[:,i], label = "ref")
        # plt.plot(pred_q[:,i], '-.', label = "pred")
        plt.plot(q[:,i], ':', label = "q_MPC")
        plt.legend()
plt.show()