# based on distance of the face of the center of the frame get appropriate distance to move servos
def step_x(position):
    delta = 0
    if position > 0.9:
        delta = 0.10
    elif position > 0.8:
        delta = 0.05
    elif position > 0.7:
        delta = 0.02
    elif position > 0.6:
        delta = 0.01
    elif position > 0.53:
        delta = 0.001
    elif position > 0.51:
        delta = 0.0001

    if position < 0.1:
        delta = -0.10
    elif position < 0.2:
        delta = -0.05
    elif position < 0.3:
        delta = -0.02
    elif position < 0.4:
        delta = -0.01
    elif position < 0.47:
        delta = -0.001
    elif position < 0.49:
        delta = -0.0001

    return delta


def step_y(position):
    delta = 0
    if position < 0.1:
        delta = 0.10
    elif position < 0.2:
        delta = 0.05
    elif position < 0.3:
        delta = 0.02
    elif position < 0.4:
        delta = 0.01
    elif position < 0.47:
        delta = 0.001
    elif position < 0.49:
        delta = 0.0001

    if position > 0.9:
        delta = -0.10
    elif position > 0.8:
        delta = -0.05
    elif position > 0.7:
        delta = -0.02
    elif position > 0.6:
        delta = -0.01
    elif position > 0.53:
        delta = -0.001
    elif position > 0.51:
        delta = -0.0001

    # print 'step_y', 'face ', position, 'delta', delta
    return delta
