import socket
import time

# Create a socket object
client_socket = socket.socket()

# Get the hostname and the port number of the server
host = socket.gethostname()
port = 12345

# Connect to the server
client_socket.connect((host, port))

while True:
    client_socket.send(b'RFID: 1')
