from gripper import Gripper
import time
# import os
#
# if os.name == 'nt':
#     import msvcrt
#     def getch():
#         return msvcrt.getch().decode()
# else:
#     import sys, tty, termios
#     fd = sys.stdin.fileno()
#     old_settings = termios.tcgetattr(fd)
#     def getch():
#         try:
#             tty.setraw(sys.stdin.fileno())
#             ch = sys.stdin.read(1)
#         finally:
#             termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
#         return ch

g = Gripper()
index = 0
while 1:
    print("Press any key to continue! (or press ESC to quit!)")
    # if getch() == chr(0x1b):
    #     break


    if index == 0:
        g.close(20)
        index = 1
        time.sleep(2)
    else:
        g.open()
        index = 0
        time.sleep(2)


    break