import cv2
import numpy as np
from tensorflow.keras.models import load_model

# Các lớp biển báo giao thông
classes = ['AHEAD', 'TURN_LEFT', 'TURN_RIGHT', 'STOP']

# Tải mô hình đã huấn luyện
model = load_model("C:/Users/ASUS/Downloads/trained_model_v8.h5")

# Khởi tạo camera
cap = cv2.VideoCapture(0)

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

while True:
    # Đọc khung hình từ camera
    ret, frame = cap.read()
    if not ret:
        break
    
    # Tiền xử lý ảnh để phát hiện cạnh
    edged = preprocess_image_1(frame)
    
    # Tìm contour lớn nhất
    largest_contour = find_largest_contour(edged)
    
    if largest_contour is not None:
        # Cắt ảnh chứa biển báo
        cropped_image = crop_sign(frame, largest_contour)
        
        # Tiền xử lý ảnh cho mô hình
        preprocessed_image = preprocess_image(cropped_image)
        
        # Dự đoán xác suất của các lớp
        predicted_pro = model.predict(preprocessed_image)[0]
        
        # Lấy nhãn dự đoán
        label_idx = np.argmax(predicted_pro)
        label = classes[label_idx]
        
        # Hiển thị tên biển báo lên khung hình
        cv2.putText(frame, label, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    
    # Hiển thị khung hình
    cv2.imshow('Traffic Sign Recognition', frame)
    
    # Nhấn 'q' để thoát
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Giải phóng tài nguyên
cap.release()
cv2.destroyAllWindows()
