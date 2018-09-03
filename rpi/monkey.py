import math
import toy3d
import toygpu


#  Advance the angle for one frame, returning the new angle
def step_angle(angle):
    angle = angle + math.pi / 240
    if angle >= 2 * math.pi:  #  Wrap at 2*Pi
        angle = angle - 2 * math.pi

    return angle


monkey = toy3d.import_obj('monkey.obj')

toygpu.opengpu()
toygpu.set_project_matrix(
    toy3d.matrix_project(0.1, 10.0, math.pi / 2.0, 4.0 / 3.0))
toygpu.set_view_matrix(
    toy3d.matrix_view(
        (0.0, 0.0, 2.5, 1.0),
        (0.0, 0.0, 0.0, 1.0),
        (0.0, 1.0, 0.0, 0.0)))

angle = 0.0
while True:
    toygpu.set_model_matrix(
        toy3d.matrix_rotate_y(angle))
    toygpu.draw_model(monkey)
    toygpu.swap()

    angle = step_angle(angle)
