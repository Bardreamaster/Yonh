from pynput import mouse, keyboard


# mouse.Button;
# mouse.Controller;

# keyboard.Key;
# keyboard.Controller;

# controller=mouse.Controller();

# # 获取鼠标位置
# print(controller.position);

# # # 定位
# controller.position=(0,20);

# # # 移动
# controller.move(150,32)

# # 单击右键
# controller.click(mouse.Button.right,1)

# # 双击左键
# controller.click(mouse.Button.left,2)

# # 按住左键
# controller.press(mouse.Button.left)
# # 释放左键
# controller.release(mouse.Button.left)

# 鼠标滚动,负数往下滚
# controller.scroll(0,-100);
def on_move(x, y):
    print(x, y)


def on_click(x, y, button, pressed):
    print(x, y)


def on_scroll(x, y, dx, dy):
    print(x, y)


# #监听鼠标
# with mouse.Listener(on_move=on_move,on_click=on_click,on_scroll=on_scroll) as listener:
#     listener.join()

# 停止监听,或者在回调中返回False
# mouse.Listener.stop()


# #以下是键盘
# # #控制键盘
# keyboardController = keyboard.Controller();
# # 按住空格
# keyboardController.press(keyboard.Key.space);
# # 松开空格键
# keyboardController.release(keyboard.Key.space);
# # 按住a
# keyboardController.press('a');
# keyboardController.release('a');


def on_press(key):
    print(key);


def on_release(key):
    print(key);
# # 监听键盘按键
with keyboard.Listener(on_press=on_press,on_release=on_release) as listener:
    listener.join()
# 停止监听,或者在回调中返回False
# keyboard.Listener.stop()dg