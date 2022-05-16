import rtde_receive
import rtde_control





rtde_c = rtde_control.RTDEControlInterface("192.168.1.10")

# rtde_c.moveJ([2.112581729888916, -1.3242619794658204, 1.634308163319723, -1.9810549221434535, -1.6361706892596644, -0.5263827482806605], 0.5, 0.3)
# rtde_c.moveL([-1.2100699583636683, -1.1043456357768555, 1.608825985585348, -2.0828758678831996, -1.5834038893329065, 1.3384003639221191], 0.5, 0.3)



rtde_r = rtde_receive.RTDEReceiveInterface("192.168.1.10")
actual_q = rtde_r.getActualQ()
actual_q = rtde_r.getActualTCPPose()[0:2]

print(actual_q)

