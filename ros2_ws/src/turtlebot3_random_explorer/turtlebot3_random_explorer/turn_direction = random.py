import random

for _ in range(10):
    # Generate a random integer, 0 or 1
    direction = random.randint(0, 1)

    # Map the random integer to a direction
    if direction == 0:
        turn_direction = "turn_left"
    else:
        turn_direction = "turn_right"


    print(turn_direction)