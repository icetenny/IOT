from raspiNetpi import PublishertoNetpie
import serial
import time
import json
import socket
import RPi.GPIO as GPIO
import time


# Define the serial port (change '/dev/ttyACM0' to your specific port)
serial_port = serial.Serial('/dev/ttyACM0', baudrate=115200, timeout=1)
#'/dev/ttyACM0'

#to pi
broker = "broker.netpie.io"
port = 1883
sub_topic = "@msg/Server"
pub_topic = "@msg/Raspi"
username = "cps"
password = "21035350"
a = PublishertoNetpie(broker, port,pub_topic)

########################################### Start Bham

servo_pin_rot = 18
servo_pin_cat = 13
servo_freq = 50
duty_cycle_min = 2.5
duty_cycle_max = 12.5
GPIO.cleanup()
GPIO.setmode(GPIO.BCM)
GPIO.setup(servo_pin_rot, GPIO.OUT)
GPIO.setup(servo_pin_cat, GPIO.OUT)

pwm_rot = GPIO.PWM(servo_pin_rot, servo_freq)
pwm_rot.start(0)

pwm_cat = GPIO.PWM(servo_pin_cat, servo_freq)
pwm_cat.start(0)
print("started")

def servo_control_rot(angle):
    duty_cycle = (angle/180.0) * (duty_cycle_max - duty_cycle_min) + duty_cycle_min
    pwm_rot.ChangeDutyCycle(duty_cycle)

def servo_control_cat(angle):
    duty_cycle = (angle/180.0) * (duty_cycle_max - duty_cycle_min) + duty_cycle_min
    pwm_cat.ChangeDutyCycle(duty_cycle)

print("Start Servo")

servo_control_cat(15)
servo_control_rot(85)

time.sleep(1)

#################################

# Communication with Pi
def receive_data(serial_port):
        received_data = serial_port.read(100)  # Read up to 100 bytes (adjust as needed)
        decoded_received_data = received_data.decode('utf8')
        #  print(f"Received data: {decoded_received_data}")  # Decode and print received data

        return decoded_received_data

def send_data(serial_port):
            data_to_send = '1' #data send to open pi
            serial_port.write(data_to_send.encode('utf8'))  # Sending encoded data via serial
            print(f"Sent data: {data_to_send}")

#dealing with data
def manage_info_data(info_data):
        data_row = info_data.split("\n")
        data = []
        if len(data_row) == 0:
             return None, None

        for r in data_row[:-1]:
            data = r.strip().split(" ")
            if len(data) == 6:
                if int(data[5]) == 1:
                    break
            # if len(data) == 6:
                #  break
       #  else:
             # return None, 0
        if len(data) != 6:
            return None, 0
        info_data = data
        print(info_data)
        data_jason = { 'power' : cal_status(),
                    'wet_amount' : cal_amount(info_data[1]) ,
                    'gen_amount' : cal_amount(info_data[2]) ,
                    'rec_amount' : cal_amount_rec(info_data[3]) ,
                    'rec_weigth' : cal_weight(info_data[4])}
        lit_status = int(info_data[5])

        try:
            data_json = json.dumps(data_jason)  # Parse received data into JSON
            return data_json, lit_status
        except json.JSONDecodeError:
            print("Received data is not in valid JSON format")
            return None, None


#calculation
def cal_weight(weight):
    weight = float(weight)
    if weight <= 1:
         return 0
    else:
        weight_re = (weight / 71.3432) - 0.06371
        return weight_re

def cal_amount(amount):
    max_amount = 14
    amount_re = int(((max_amount - float(amount)) / max_amount)* 100)
    amount_re = max(0, amount_re)
    return amount_re

def cal_amount_rec(amount):
    max_amount = 11
    amount_re = int(((max_amount - float(amount)) / max_amount)* 100)
    amount_re = max(0, amount_re)
    return amount_re

def cal_status():
    if serial_port.is_open:
        status = 'active'
    else :
        status = 'inactive'

    return status

def main(serial_port):
    last_lit_status = 0

    while serial_port.is_open :
        time.sleep(0.2)
        print('Serial Port is Ready')
        # send_data(serial_port)
        # time.sleep(1)
        info_data = receive_data(serial_port)
        netpiedata, lit_status = manage_info_data(info_data) #given json
        print("Lit Status:", lit_status)
        if netpiedata:
            a.pub_function(netpiedata)

        if lit_status == 0 and last_lit_status == 1:
            print("YOLOing..........")
            host = 'localhost'  # server's address
            port = 10000         # server's port

            client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            client_socket.connect((host, port))
                # Receive data from the server
            data = client_socket.recv(1024).decode()
            print(f"Received from server: {data}")

            # Close the connection with the server
            client_socket.close()
            if data == "wet":
                servo_control_rot(20)
                time.sleep(1)
                servo_control_cat(45)
                time.sleep(1)
                servo_control_cat(15)
                time.sleep(1)
                servo_control_rot(85)

            elif data == "general":
                servo_control_cat(45)
                time.sleep(1)
                servo_control_cat(15)

            elif data == "recycle":
                servo_control_rot(150)
                time.sleep(1)
                servo_control_cat(45)
                time.sleep(1)
                servo_control_cat(15)
                time.sleep(1)
                servo_control_rot(85)
                time.sleep(2)

                send_data(serial_port)
                time.sleep(1)

        last_lit_status = lit_status


if __name__ == '__main__':
    try:
        main(serial_port)
        a.run()
    except KeyboardInterrupt:
        serial_port.close()  # Close the serial port on Ctrl+C
