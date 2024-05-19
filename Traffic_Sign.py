import cv2
import numpy as np
import tflite_runtime.interpreter as tflite
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import gpiozero
import Distance as dtc


TRIG = 21
ECHO = 20
dtc.config_dtc(TRIG, ECHO)
robot = gpiozero.Robot(left=(24, 23), right=(17, 27))
forward_speed = 0.8
backward_speed = 0.7
turn_speed = 0.65

def perform_action(label):
    if label == 1:
        robot.left(turn_speed)
        print("Turn left")
        time.sleep(2.3)
    elif label == 2:
        robot.right(turn_speed)
        print("Turn right")
        time.sleep(2.3)
    elif label == 3:
        robot.stop()
        print("Stop")
        time.sleep(5)
    elif label == 0:
        robot.forward(forward_speed)
        print("Forward")
        time.sleep(2)

# Các lớp biển báo giao thông
classes = ['AHEAD', 'TURN_LEFT', 'TURN_RIGHT', 'STOP']

# Tải mô hình đã huấn luyện
model_path = "/home/pi/AutomousVehicle/Traffic_Sign/traffic_sign.tflite"
interpreter = tflite.Interpreter(model_path=model_path)
interpreter.allocate_tensors()

# Lấy thông tin về các tensor
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()

# Khởi tạo camera
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
raw_capture = PiRGBArray(camera, size=(640, 480))

# Để camera khởi động
time.sleep(0.1)

IMG_SIZE = 64

def preprocess_image_1(image):
    # Chuyển đổi ảnh sang thang độ xám
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    # Làm mờ ảnh để giảm nhiễu
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    
    # Sử dụng adaptive thresholding để làm nổi bật cạnh biển báo
    thresh = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                   cv2.THRESH_BINARY_INV, 11, 2)
    return thresh

def find_largest_contour(edged):
    # Tìm các contours trong ảnh
    contours, _ = cv2.findContours(edged, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if contours:
        # Chọn contour lớn nhất
        largest_contour = max(contours, key=cv2.contourArea)
        return largest_contour
    else:
        return None

def crop_sign(image, contour):
    # Tính toán bounding box cho contour lớn nhất
    x, y, w, h = cv2.boundingRect(contour)
    
    # Cắt ảnh ra từ bounding box
    cropped_image = image[y:y+h, x:x+w]
    
    return cropped_image

def preprocess_image(image):
    # Resize ảnh về kích thước mô hình yêu cầu
    image = cv2.resize(image, (IMG_SIZE, IMG_SIZE))
    image = np.array(image)
    image = np.expand_dims(image, axis=0)
    return image

def detect_sign_and_act():
    for frame in camera.capture_continuous(raw_capture, format="bgr", use_video_port=True):
        # Lấy mảng numpy đại diện cho hình ảnh
        image = frame.array
        
        # Tiền xử lý ảnh để phát hiện cạnh
        edged = preprocess_image_1(image)
        
        # Tìm contour lớn nhất
        largest_contour = find_largest_contour(edged)
        
        if largest_contour is not None:
            # Cắt ảnh chứa biển báo
            cropped_image = crop_sign(image, largest_contour)
            
            # Tiền xử lý ảnh cho mô hình
            preprocessed_image = preprocess_image(cropped_image)
            
            # Chuẩn bị đầu vào cho mô hình
            input_data = np.array(preprocessed_image, dtype=np.float32)

            # Đặt tensor đầu vào
            interpreter.set_tensor(input_details[0]['index'], input_data)
            
            # Chạy mô hình
            interpreter.invoke()
            
            # Lấy kết quả từ tensor đầu ra
            output_data = interpreter.get_tensor(output_details[0]['index'])
            predicted_pro = output_data[0]
            
            # Lấy nhãn dự đoán
            label_idx = np.argmax(predicted_pro)
            label = classes[label_idx]
            
            # Hiển thị tên biển báo lên khung hình
            cv2.putText(image, label, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # Thực hiện hành động
            perform_action(label_idx)
        
        # Hiển thị khung hình
        cv2.imshow('Traffic Sign Recognition', image)
        
        # Nhấn 'q' để thoát
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        
        # Xóa khung hình cũ để chuẩn bị cho khung hình tiếp theo
        raw_capture.truncate(0)
        break  # Thoát khỏi vòng lặp sau khi xử lý một khung hình

# Vòng lặp chính
try:
    while True:
        distance = dtc.measure_distance(TRIG, ECHO)
        robot.forward(forward_speed)  # Tiếp tục chạy thẳng sau khi thực hiện hành động
        print(f"Distance: {distance} cm")
        if distance <= 25:
            robot.stop()  # Dừng xe nếu khoảng cách nhỏ hơn hoặc bằng 17 cm
            time.sleep(2)
            detect_sign_and_act()
        
        time.sleep(0.1)         
        
except KeyboardInterrupt:
    print("Chương trình dừng lại.")
    cv2.destroyAllWindows()
