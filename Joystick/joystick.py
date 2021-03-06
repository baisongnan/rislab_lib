import pygame


def joystick_saturation(joystick_obj):
    saturation_level = 0.15
    d_pad_z = -(joystick_obj.get_axis(4))  # get axis values
    if abs(d_pad_z) < saturation_level:
        d_pad_z = 0
    d_pad_y = (joystick_obj.get_axis(0))  # get axis values
    if abs(d_pad_y) < saturation_level:
        d_pad_y = 0
    d_pad_x = (joystick_obj.get_axis(1))  # get axis values
    if abs(d_pad_x) < saturation_level:
        d_pad_x = 0
    return d_pad_z, d_pad_y, d_pad_x


def joystick_saturation_single(axis):
    if abs(axis) < 0.15:
        axis = 0
    axis_out = axis
    return axis_out


pygame.init()
pygame.joystick.init()
global joystick
for event in pygame.event.get():
    if event.type == pygame.QUIT:
        done = True
joystick = pygame.joystick.Joystick(0)
joystick.init()
print('pygame started')
name = joystick.get_name()
print("Joystick name: {}".format(name))


def step():
    global joystick
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            done = True
    joystick = pygame.joystick.Joystick(0)
    joystick.init()


def quit():
    pygame.quit()
    print('pygame stopped')


def get():
    global joystick
    return joystick


def get_axis_lx():
    global joystick
    return joystick.get_axis(0)


def get_axis_ly():
    global joystick
    return joystick.get_axis(1)


def get_axis_lt():
    global joystick
    return joystick.get_axis(2)


def get_axis_rx():
    global joystick
    return joystick.get_axis(3)


def get_axis_ry():
    global joystick
    return joystick.get_axis(4)


def get_axis_rt():
    global joystick
    return joystick.get_axis(5)


def get_button_a():
    global joystick
    return joystick.get_button(0)


def get_button_b():
    global joystick
    return joystick.get_button(1)


def get_button_x():
    global joystick
    return joystick.get_button(2)


def get_button_y():
    global joystick
    return joystick.get_button(3)


def get_button_lb():
    global joystick
    return joystick.get_button(4)


def get_button_rb():
    global joystick
    return joystick.get_button(5)


def get_button_l():
    global joystick
    return joystick.get_button(6)


def get_button_r():
    global joystick
    return joystick.get_button(7)


def get_button_start():
    global joystick
    return joystick.get_button(8)


def get_button_back():
    global joystick
    return joystick.get_button(9)


def get_button_xbox():
    global joystick
    return joystick.get_button(10)


def get_button_up():
    global joystick
    return joystick.get_button(11)


def get_button_down():
    global joystick
    return joystick.get_button(12)


def get_button_left():
    global joystick
    return joystick.get_button(13)


def get_button_right():
    global joystick
    return joystick.get_button(14)

# ps 4 controller


def get_button_block():
    global joystick
    return joystick.get_button(0)


def get_button_cross():
    global joystick
    return joystick.get_button(1)


def get_button_circle():
    global joystick
    return joystick.get_button(2)


def get_button_triangle():
    global joystick
    return joystick.get_button(3)


def get_button_l1():
    global joystick
    return joystick.get_button(4)


def get_button_r1():
    global joystick
    return joystick.get_button(5)


def get_button_l2():
    global joystick
    return joystick.get_button(6)

def get_button_r2():
    global joystick
    return joystick.get_button(7)


def get_button_share():
    global joystick
    return joystick.get_button(8)


def get_button_option():
    global joystick
    return joystick.get_button(9)


def get_button_l_ps4():
    global joystick
    return joystick.get_button(10)


def get_button_r_ps4():
    global joystick
    return joystick.get_button(11)


def get_button_ps():
    global joystick
    return joystick.get_button(12)


def get_button_table():
    global joystick
    return joystick.get_button(13)


def get_hat_up():
    global joystick
    return joystick.get_hat(0)[1] == 1


def get_hat_down():
    global joystick
    return joystick.get_hat(0)[1] == -1


def get_hat_right():
    global joystick
    return joystick.get_hat(0)[0] == 1


def get_hat_left():
    global joystick
    return joystick.get_hat(0)[0] == -1


def get_axis_ry_ps4():
    global joystick
    return joystick.get_axis(5)


def get_axis_rx_ps4():
    global joystick
    return joystick.get_axis(2)


def get_axis_ly_ps4():
    global joystick
    return joystick.get_axis(1)


def get_axis_lx_ps4():
    global joystick
    return joystick.get_axis(0)


def get_axis_r2_ps4():
    global joystick
    return joystick.get_axis(4)


def get_axis_l2_ps4():
    global joystick
    return joystick.get_axis(3)


def get_power():
    global joystick
    return joystick.get_power_level()