import socket

# Create a socket object
server_socket = socket.socket()

# Get the hostname and the port number
host = socket.gethostname()
port = 12345

# Bind the socket to the host and port
server_socket.bind((host, port))

# Start listening for incoming connections
server_socket.listen()

while True:
    # Accept an incoming connection
    connection, address = server_socket.accept()

    # Receive data from the client
    data = connection.recv(1024)

    # Print the received data
    print(data.decode())

    # Close the connection
    connection.close()
