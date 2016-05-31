
import nxt
print "connecting to brick..."
brick = nxt.locator.find_one_brick()
print brick
motorb = nxt.motor.Motor(brick, nxt.motor.PORT_B)
motorc = nxt.motor.Motor(brick, nxt.motor.PORT_C)

def clamp(n, (minn, maxn)):
    """
    Given a number and a range, return the number, or the extreme it is closest to.
    :param n: number
    :return: number
    """
    return max(min(maxn, n), minn)


def scale(val, src, dst):
    """
    Scale the given value from the scale of src to the scale of dst.
    val: float or int
    src: tuple
    dst: tuple
    example: print scale(99, (0.0, 99.0), (-1.0, +1.0))
    """
    return (float(val - src[0]) / (src[1] - src[0])) * (dst[1] - dst[0]) + dst[0]

def scalestick(value):
    return scale(value,(-100,100),(-100,100))

def dc_clamp(value):
    return clamp(value,(-100,100))


# Main loop
while True:
    try:
        motorb.run(power=dc_clamp(L_motor_speed),regulated=True)
        motorc.run(power=dc_clamp(R_motor_speed),regulated=True)
    except KeyboardInterrupt:
        break
motorb.run(power=0,regulated=False)
motorc.run(power=0,regulated=False)
