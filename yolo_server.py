import socket
from ultralytics import YOLO
import cv2

model = YOLO("best_final.pt")
#cap = cv2.VideoCapture(0)
print("Initiating YOLO...................")

# Create a socket object
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Bind the socket to a specific address and port
host = "localhost"  # localhost
port = 10000         # port to bind to

server_socket.bind((host, port))

# Listen for incoming connections
server_socket.listen(5)

print(f"Server listening on {host}:{port}")

# Accept connections from clients
while True:
    # Establish connection with the client
    client_socket, addr = server_socket.accept()
    print(f"Got connection from {addr}")
    cap = cv2.VideoCapture(0)
    ret, frame = cap.read()
    cap.release()
    # cv2.imwrite("a.jpg", frame)


    result = model.predict(frame)
    cat = result[0].names[result[0].probs.top1]


    # Send a welcome message to the client
    message = cat
    client_socket.send(message.encode())

    # Close the connection with the client
    client_socket.close()
