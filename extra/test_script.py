import time
import datetime

import random

def recurring(interval, callable):
    i = 0
    start = time.time()
    while True:
        i += 1
        callable()
        remaining_delay = max(start + (i * interval) - time.time(), 0)
        print("remaining delay: %s" % remaining_delay)
        time.sleep(remaining_delay)

def tick_delay():
    print('tick start')
    time.sleep(random.randrange(1, 4))
    print('tick end')

recurring(5, tick_delay)