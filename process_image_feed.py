from ultralytics import YOLO
import cv2
import socket
import numpy as np
from ultralytics.utils.plotting import Annotator

model = YOLO("yolov8n.pt")

def receive_and_process_images():
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

            results = model.predict(frame)

            annotator = Annotator(frame)
            for r in results:
                boxes = r.boxes
                for box in boxes:
                    b = box.xyxy[0]
                    c = box.cls
                    annotator.box_label(b, model.names[int(c)])

            frame = annotator.result()
            cv2.imshow('YOLO V8 Detection', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    clientsocket.close()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    receive_and_process_images()
