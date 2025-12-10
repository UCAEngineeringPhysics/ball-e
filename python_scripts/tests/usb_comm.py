import serial
from time import sleep

messenger = serial.Serial(port="/dev/ttyACM0", baudrate=115200)
print(messenger.name)
dev_name = "USB"
out_msg = "d{dev_name}\n"

for i in range(100):
    if messenger.inWaiting() > 0:
        in_msg = messenger.readline().strip().decode("utf-8", "ignore")
        print(f"{dev_name} recieved: {in_msg}")
    if 25 <= i < 75:
        out_msg = "0.4, 0.0, -1, 1\n"
    else:
        out_msg = "0.0, 0.0, 1, -1\n"
    messenger.write(out_msg.encode("utf-8"))
    sleep(0.05)
print("All messages sent.")
messenger.close()

# messenger.write("0.0, 0.0, 0, 0\n".encode("utf-8"))
