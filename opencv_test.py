import cv2

def show_image(image_path):
    image = cv2.imread(image_path)
    if image is None:
        print("Failed to load image.")
        return

    cv2.imshow('Test Image', image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    image_path = 'test.png' # Replace with image path
    show_image(image_path)
