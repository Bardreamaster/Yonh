import pid as PID
import time
import matplotlib.pyplot as plt


def test_pid(P, I , D):

    pid = PID.pid([0.1, 0.1, 0.1],[0, 0, 0],[0, 0, 0])

    pid.target= [1,1,1]
    pid.setSampleTime(0.01)

    feedback = [0, 0, 0]
    output_list = []

    for i in range(1, 100):
        pid.update(feedback)
        output = pid.output
        feedback += output
        time.sleep(0.01)
        output_list.append(output[1])

    plt.figure(0)
    plt.grid(True)
    plt.plot(output_list,'r')
    plt.xlabel('time (s)')
    plt.ylabel('PID (PV)')
    plt.title('PythonTEST PID',fontsize=15)
    plt.grid(True)
    plt.show()

if __name__ == "__main__":
    test_pid(0.5, 0.1, 0.001)