import time

def cyclic_number_generator():
    n = 1
    direction = 1  # 1 for increasing, -1 for decreasing
    while True:
        yield n
        if n == 10:
            direction = -1
        elif n == 1:
            direction = 1
        n += direction

# Example usage:
generator = cyclic_number_generator()

# Call the generator function multiple times to get the cycling numbers
while True:
    number = next(generator)
    print(number)
    time.sleep(0.1)