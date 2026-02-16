"""Minimal PROS-style autonomous routine example for simulator usage."""


def autonomous(api):
    api.set_tank(0.7, 0.7)
    for _ in range(900):
        yield from api.delay(2)

    api.set_tank(0.5, -0.5)
    for _ in range(400):
        yield from api.delay(2)

    api.set_tank(0.8, 0.8)
    for _ in range(700):
        yield from api.delay(2)

    api.set_tank(0.0, 0.0)
    while True:
        yield from api.delay(5)
