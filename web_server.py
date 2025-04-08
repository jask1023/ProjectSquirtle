from flask import Flask, request
import os
from ultralytics import YOLO
import cv2
import serial

app = Flask(__name__)

# Load YOLOv8 model
model = YOLO('yolov8n.pt')

# Set target object (Fire but for now cell phone)
target_object = "cell phone"

# Upload images from esp32
UPLOAD_FOLDER = 'uploaded_images'
os.makedirs(UPLOAD_FOLDER, exist_ok=True)

# Serial Communication to K66F
SERIAL_PORT = 'COM6'
BAUD_RATE = 9600


# for communication with esp32
@app.route('/upload', methods=['POST'])
def upload_image():
    # No image error
    if 'image' not in request.files:
        return 'No image part', 400
    
    file = request.files['image']
    if file.filename == '':
        return 'No selected file', 400
    
    # Save Image (for now)
    file_path = os.path.join(UPLOAD_FOLDER, file.filename)
    file.save(file_path)
    
    # Read image
    img = cv2.imread(file_path)
    image_height, image_width, _ = img.shape
    
    # Calculate the center of the image
    image_center_x = image_width / 2
    image_center_y = image_height / 2
    
    # Run YOLOv8 model for object detection
    results = model(img)
    
    # Extract detected objects information
    detected_object = []
    for result in results[0].boxes.data:  # Loop through detected boxes
        xmin, ymin, xmax, ymax = result[0:4].cpu().numpy()  # Get bounding box coordinates (xyxy format)
        conf = result[4].cpu().numpy()  # Get the confidence score
        cls = int(result[5].cpu().numpy())  # Get the class index
        label = model.names[cls]  # Get the class name (label)

         # If the detected object matches the target object, add to the list
        if label.lower() == target_object.lower():
            # Calculate the center of the bounding box
            box_center_x = (xmin + xmax) / 2
            box_center_y = (ymin + ymax) / 2

            diff_x = image_center_x - box_center_x
            diff_y = image_center_y - box_center_y

            # Append object details along with center info
            detected_object.append({
                "label": label,
                "confidence": conf,
                "bounding_box": [xmin, ymin, xmax, ymax],
                "box_center": [box_center_x, box_center_y],
                "image_center": [image_center_x, image_center_y],
                "diff_y": diff_y,
                "diff_x": diff_x
            })

            for obj in detected_object:
                send_data(obj)

    # Print detected objects
    print("Detected objects:")
    for obj in detected_object:
        print(f"Label: {obj['label']}, Confidence: {obj['confidence']:.2f},box_center: {obj['box_center']},image_center: {obj['image_center']}")
    
    # OK response
    return 'Image processed successfully', 200

def send_data(obj):
    try:
        with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser:
            message = f"Diff_X: {obj['diff_x']}, Diff_Y: {obj['diff_y']}\n"
            print(f"Sending data to FRDM K66F: {message.strip()}")
            ser.write(message.encode())
    except serial.SerialException as e:
        print(f"Serial port error: {e}")


if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)  # All IP's
