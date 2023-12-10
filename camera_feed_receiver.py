import socket
import cv2
import numpy as np

def receive_images():
    clientsocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    clientsocket.connect(('localhost', 23001))
    data = b""
    while True:
        packet = clientsocket.recv(4096)
        if not packet: break
        data += packet

        start = data.find(b'\xff\xd8')
        end = data.find(b'\xff\xd9')
        if start != -1 and end != -1:
            jpg = data[start:end+2]
            data = data[end+2:]
            frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
            cv2.imshow('Received Image', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    clientsocket.close()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    receive_images()
