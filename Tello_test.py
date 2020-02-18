from tello import Tello
import sys
from datetime import datetime
import time

drone = Tello()
# drone.status()
while True:
    command = input("COMMAND>")
    drone.send_command(command)
