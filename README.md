# TRAFFIC-SIGN-RECOGNITION-PROJECT_2567

# CODE MOTOR CONTROL


import websockets
import json
import asyncio
import threading
import cv2
import os
from flask import Flask, render_template, Response
from gpiozero import Motor,DistanceSensor
import time
from ultralytics.utils.plotting import Annotator
from ultralytics import YOLO
from datetime import datetime
from mpu6050 import mpu6050

app = Flask(__name__)


camera = cv2.VideoCapture(0)  # ดึงสัญญาณจากกล้อง (0 คือกล้องเริ่มต้น)
if not camera.isOpened():
    raise ValueError("ไม่สามารถเปิดกล้องได้ กรุณาตรวจสอบการเชื่อมต่อของกล้อง.")

latest_frame = None
motor_lock = threading.Lock()
gyro_lock = threading.Lock()


@app.route('/')
def index():
    return render_template('holdbtn.html')

def generate_frames():
    global latest_frame
    while True:
        if latest_frame is None:
            continue
        frame = latest_frame.copy()
        
        time.sleep(0.1)
        

        ret, buffer = cv2.imencode('.jpg', frame)
        if not ret:
            continue
        frame = buffer.tobytes()

        # ส่งเฟรมเป็นไบต์ในรูปแบบของสตรีมวิดีโอ
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        time.sleep(0.03333)

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

#-------------------------------------------------------------------------------------------------------#

# การตั้งค่าขา GPIO สำหรับมอเตอร์
motor1 = Motor(forward=17, backward=27)  # มอเตอร์ตัวที่ 1 ขวาหน้า
motor2 = Motor(forward=26, backward=19)  # มอเตอร์ตัวที่ 2 ซ้ายหน้า
motor3 = Motor(forward=25, backward=9)   # มอเตอร์ตัวที่ 3 ซ้ายหลัง
motor4 = Motor(forward=20, backward=21)  # มอเตอร์ตัวที่ 4 ขวาหลัง

# การตั้งค่าขา GPIO สำหรับเซ็นเซอร์อัลตร้าโซนิค
ultrasonic_sensor = DistanceSensor(echo=6, trigger=5)

# ฟังก์ชันควบคุมมอเตอร์
def stop_all():
    global auto_running, stop_running
    auto_running = False
    stop_running = True
    stop_movement()  # หยุดมอเตอร์ทั้งหมด
    print("System stopped.")
    
def stop_all_threads():
    global auto_running, sensor_running, yolo_detected
    auto_running = False  # หยุดการทำงานของโหมดอัตโนมัติ
    sensor_running = False  # หยุดการทำงานของเซ็นเซอร์อัลตร้าโซนิค
    yolo_detected = False  # ปิดการทำงานของ YOLO
    stop_movement()  # หยุดมอเตอร์ทั้งหมด
    print("All systems and threads stopped.")

def stop_movement():
    print("หยุดมอเตอร์ทั้งหมด") 
    motor1.stop()
    motor2.stop()
    motor3.stop()
    motor4.stop()
    
def move_forward(): #(speed=0.5):
    with gyro_lock:  # ใช้ lock เพื่อป้องกันการทำงานซ้อนทับ
        motor1.forward(0.5)
        motor2.forward(0.5)
        motor3.forward(0.7)
        motor4.forward(0.5)

def move_backward():
    print("Moving backward")  # เพิ่มคำสั่งพิมพ์เพื่อดีบัก
    motor1.backward(0.5)
    motor2.backward(0.5)
    motor3.backward(0.5)
    motor4.backward(0.5)
    
def turn_left():
    print("Attempting to turn left")
    motor1.forward(0.5)
    motor4.forward(0.5)
    motor2.backward(0.5)
    motor3.backward(0.5)

    
def turn_right():
    print("Attempting to turn right")
    motor2.forward(0.5)
    motor3.forward(0.5)
    motor1.backward(0.5)
    motor4.backward(0.5)
    
def no_turn_left():
    print("เจอป้ายห้ามเลี้ยวซ้าย , เลี้ยวขวาค่ะ")
    motor2.forward(0.5)
    motor3.forward(0.5)
    motor1.backward(0.5)
    motor4.backward(0.5)
    
def no_turn_right():
    print("เจอป้ายห้ามเลี้ยวขวา , เลี้ยวซ้ายค่ะ")
    motor1.forward(0.5)
    motor4.forward(0.5)
    motor2.backward(0.5)
    motor3.backward(0.5)

def turn_left_opj():
    print("สิ่งกีดขวาง เลี้ยวซ้าย")
    motor1.forward(0.4)
    motor4.forward(0.4)
    motor2.backward(0.4)
    motor3.backward(0.4)
    time.sleep(0.2)
    
def back():
    print("สิ่งกีดขวาง เลี้ยวซ้าย")
    motor2.backward(0.5)  
    # ล้อซ้ายหน้าหมุนไปข้างหน้า
    motor1.forward(0.5)   
    # ล้อซ้ายหลังหมุนไปข้างหน้า
    motor3.forward(0.5)   
    # ล้อขวาหลังหมุนถอยหลัง
    motor4.backward(0.5)  

# ฟังก์ชันสำหรับหมุนรถกลับ 180 องศา
def turn_around_180():
    print("กำลังหมุนรถกลับ 180 องศา.")

    # หมุนล้อซ้ายไปข้างหน้า และล้อขวาไปข้างหลัง
    motor1.backward(0.5)  # ขวาหน้าหมุนไปข้างหลัง
    motor4.backward(0.5)
    motor2.forward(0.5)  # ซ้ายหน้าหมุนไปข้างหน้า
    motor3.forward(0.5)  # ซ้ายหลังหมุนไปข้างหน้า
      # ขวาหลังหมุนไปข้างหลัง

    # หน่วงเวลาให้หมุนครบ 180 องศา
    time.sleep(0.3)  # ปรับระยะเวลานี้ให้เหมาะสมตามการทดสอบ

    # หยุดมอเตอร์หลังจากหมุนครบ
    # stop_movement()

    print("หมุนกลับ 180 องศาเสร็จเรียบร้อยแล้ว.")
    
# ฟังก์ชันสำหรับหมุนรถกลับ 180 องศาโดยไม่ใช้เซ็นเซอร์
def turn_around_180_left():
    print("กำลังหมุนรถกลับ 180 องศาไปทางซ้าย.")

    motor3.backward(0.5)  # ซ้ายหน้าหมุนไปข้างหลัง
    motor2.backward(0.5)  # ซ้ายหลังหมุนไปข้างหลัง
    motor1.forward(0.5)  # ขวาหน้าหมุนไปข้างหน้า
    motor4.forward(0.5)  # ขวาหลังหมุนไปข้างหน้า

    # หน่วงเวลาให้หมุนครบ 180 องศา
    time.sleep(0.3)  # ปรับเวลานี้ให้เหมาะสมตามการทดสอบ

    # # หยุดมอเตอร์หลังจากหมุนครบ
    # stop_movement()

    print("หมุนไปทางซ้าย 180 องศาเสร็จเรียบร้อยแล้ว.")





#-------------------------------------------------------------------------------------------------------#





# ฟังก์ชันสำหรับการอ่านค่าระยะทางจากเซ็นเซอร์อัลตร้าโซนิค
ultrasonic_enabled = True  # ตัวแปรนี้ใช้เปิด/ปิดการตรวจจับเซ็นเซอร์

def check_obstacle():
    # ถ้า ultrasonic_enabled เป็น False จะไม่ตรวจจับสิ่งกีดขวาง
    if not ultrasonic_enabled:
        return False
    
    distance = ultrasonic_sensor.distance * 100  # แปลงเป็นเซนติเมตร
    print(f"Distance: {distance:.2f} cm")
    
    # ตรวจสอบระยะทาง ถ้าต่ำกว่า 20 เซนติเมตรให้หยุดรถ
    if distance < 20:  # ถ้าระยะทางน้อยกว่า 20 ซม. ให้หยุดหรือถอยหลัง
        stop_movement()
        time.sleep(1) 
        print("Obstacle detected within 20 cm! Stopping the vehicle.")
        move_backward
        time.sleep(1)
        turn_left_opj()
        time.sleep(0.6)
        move_forward() 
        return True
    return False

#-------------------------------------------------------------------------------------------------------#
# ตัวแปรสำหรับเก็บโหมดปัจจุบันและสถานะการทำงานของโหมดอัตโนมัติ
current_mode = "manual"
auto_running = False  # ตัวแปรสำหรับควบคุมโหมดอัตโนมัติ
stop_running = False  # สถานะสำหรับการหยุด
yolo_detected = False  # ตัวแปรนี้จะเปลี่ยนเป็น True เมื่อ YOLO 
last_detected_sign = None  # ประกาศตัวแปร global ข้างนอกฟังก์ชัน
gyro_running = False  # ตัวแปรสถานะสำหรับการทำงานของ Gyroscope
sensor_running = False  # ตัวแปรสถานะสำหรับการทำงานของเซ็นเซอร์

# 

# ฟังก์ชัน YOLO สำหรับตรวจจับวัตถุและบันทึกภาพในโฟลเดอร์ captures
def yolo_inference():
    global latest_frame, yolo_detected, current_mode, last_detected_sign, objects_detected
    model = YOLO("best16.pt")
    
    if not os.path.exists("captures"):
        os.makedirs("captures")
        
    # ตัวแปรสำหรับบันทึกป้ายที่ตรวจจับล่าสุด และเวลาที่ตรวจจับล่าสุด
    last_sign_detected = None
    last_detection_time = 0
    cooldown_period = 5  # ช่วงเวลาหน่วง 5 วินาที

    # ตัวแปรสำหรับบันทึกสถานะการบันทึกภาพ
    captured_signs = {}

    while True:
        if latest_frame is None or current_mode == "manual":
            continue  # ข้ามไปถ้าอยู่ในโหมด manual
        frame = latest_frame.copy()

        frame_resized = cv2.resize(frame, (480, 480))
        results = model(frame_resized, conf=0.7)

        annotator = Annotator(frame_resized)
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        objects_detected = False

        # ตั้งตัวแปรชั่วคราวสำหรับเก็บค่าป้ายที่ตรวจพบ
        detected_sign = None

        for result in results:
            boxes = result.boxes
            if len(boxes) > 0:
                objects_detected = True
                for box in boxes:
                    b = box.xyxy[0].tolist()
                    c = int(box.cls)
                    confidence = box.conf.item()  # แปลง Tensor เป็น float
                    label = f"{model.names[c]} {confidence:.2f}"  # เพิ่มความเชื่อมั่นลงใน label

                    if confidence >= 0.6:
                        annotator.box_label(b, label)

                    # ดีบักข้อมูลเพื่อเช็คว่าป้ายที่ตรวจพบคืออะไร
                    print(f"Detected: {model.names[c]} with confidence {confidence}")

                    # ตรวจจับป้ายจราจร
                    if ultrasonic_sensor.distance * 100 <= 60 and model.names[c] in ['No-Entry', 'Turn-Left', 'Turn-Right', 'End', 'Stop', 'No-Turn-Left', 'No-Turn-Right', 'U-Turn', 'Not-know']:
                        detected_sign = model.names[c]  # เก็บชื่อป้ายที่ตรวจเจอ
                        yolo_detected = True
                        print(f"Sign detected: {detected_sign}")  # ดีบักป้ายที่ตรวจเจอ

        if detected_sign:
            # เช็คว่าป้ายถูกบันทึกไปแล้วในรอบนี้หรือยัง
            if detected_sign not in captured_signs or captured_signs[detected_sign] is False:
    
                # สร้างโฟลเดอร์สำหรับแต่ละป้าย ถ้ายังไม่มี
                sign_folder = f"captures/{detected_sign}"
                if not os.path.exists(sign_folder):
                    os.makedirs(sign_folder)

                # บันทึกภาพตามประเภทป้ายที่ตรวจเจอ
                last_detected_sign = detected_sign
                
                print(f"{last_detected_sign} sign detected, saving image.")

                frame_with_detections = annotator.result()
                image_path = f"{sign_folder}/{timestamp}.jpg"  # ตั้งชื่อไฟล์ด้วย timestamp
                cv2.imwrite(image_path, frame_with_detections)
                print(f"Saved image: {image_path}")

                # ทำเครื่องหมายว่าป้ายนี้ถูกบันทึกแล้ว
                captured_signs[detected_sign] = True
            else:
                print(f"{detected_sign} already captured, waiting for next detection.")

        # รีเซ็ตสถานะการบันทึกเมื่อป้ายไม่ถูกตรวจเจอ
        if not detected_sign:
            captured_signs = {sign: False for sign in captured_signs}

        time.sleep(0.1)

def auto_mode():
    global auto_running, current_mode, yolo_detected, last_detected_sign, ultrasonic_enabled
    auto_running = True  # เปิดสถานะการทำงานของโหมดอัตโนมัติ
    sign_detected = False  # ตัวแปรสำหรับตรวจสอบว่ามีการตรวจจับป้ายแล้ว

    while True:
        if auto_running and current_mode == "auto":
            # ตรวจสอบเงื่อนไขของการตรวจจับป้ายจาก YOLO
            if yolo_detected and not sign_detected and last_detected_sign is not None:
                print(f"YOLO detected a sign: {last_detected_sign}, executing action.")
                
                # ปิดการตรวจจับเซ็นเซอร์ชั่วคราว
                ultrasonic_enabled = False
                
                # ดำเนินการตามป้ายที่ตรวจพบ
                if last_detected_sign == 'No-Turn-Left':
                    print("No Turn Left sign detected, turning right.")
                    turn_right()  # เลี้ยวขวา
                    time.sleep(0.7)  # ลดระยะเวลาหน่วง
                    
                elif last_detected_sign == 'No-Entry':
                    print("No Entry sign detected, moving backward.")
                    turn_around_180()  # ฟังก์ชันหมุนรถกลับ 180 องศา
                    time.sleep(1.2)  # รอให้หมุนเสร็จ
                
                elif last_detected_sign == 'U-Turn':
                    print("U Turn sign detected, moving backward.")
                    turn_around_180_left()  # ฟังก์ชันหมุนรถกลับ 180 องศา
                    time.sleep(0.9)  # รอให้หมุนเสร็จ

                elif last_detected_sign == 'Turn-Left':
                    print("เจอป้ายเลี้ยวซ้าย, กำลังเลี้ยวซ้าย.")
                    turn_left()  # เลี้ยวซ้าย
                    time.sleep(0.6)  # ลดระยะเวลาหน่วง
                    
                elif last_detected_sign == 'Turn-Right':
                    print("เจอป้ายเลี้ยวขวา, กำลังเลี้ยวขวา")
                    turn_right()  # เลี้ยวขวา
                    time.sleep(0.75)  # ลดระยะเวลาหน่วง
                    
                elif last_detected_sign == 'No-Turn-Right':
                    print("No Turn Right sign detected, turning left.")
                    no_turn_right()  # เลี้ยวซ้าย
                    time.sleep(0.6)  # ลดระยะเวลาหน่วง
                    
                    
                elif last_detected_sign == 'Stop':
                    print("Stop sign detected. Stopping for 3 seconds.")
                    stop_movement()  # หยุดรถ
                    time.sleep(3)  # หน่วงเวลาสั้นลง
                    turn_left()  # เคลื่อนที่ต่อไปข้างหน้า
                    time.sleep(0.6)
                    
                elif last_detected_sign == 'End':
                    print("End sign detected, stopping all systems.")
                    stop_movement()  # หยุดมอเตอร์ทั้งหมด
                    auto_running = False  # ปิดโหมดอัตโนมัติ
                    yolo_detected = False  # ปิดการตรวจจับ YOLO
                    ultrasonic_enabled = False  # ปิดการทำงานของเซ็นเซอร์อัลตร้าโซนิค
                    stop_all_threads()  # เรียกฟังก์ชันเพื่อหยุดเธรดทั้งหมด
                    print("System fully stopped after detecting End sign.")
                    break
                
                ultrasonic_enabled = True
                # ตั้งค่าให้การตรวจจับป้ายเสร็จสมบูรณ์
                sign_detected = True
                yolo_detected = False  # รีเซ็ตสถานะการตรวจจับป้าย
                last_detected_sign = None  # รีเซ็ตป้ายที่ตรวจจับ
                # เปิดการทำงานของเซ็นเซอร์อัลตร้าโซนิคหลังจากเลี้ยวเสร็จ

                
            elif not yolo_detected:
                # ไม่มีการตรวจจับป้ายจาก YOLO (ทำงานปกติ เช่น เคลื่อนที่ไปข้างหน้า)
                if not check_obstacle():  # ตรวจสอบสิ่งกีดขวางด้วยเซ็นเซอร์อัลตร้าโซนิค
                    move_forward()
                    print("Moving forward")
                sign_detected = False  # กลับไปที่สถานะเริ่มต้นเมื่อไม่มีการตรวจจับป้ายหรือสิ่งกีดขวาง

            time.sleep(0.05)  # ลดการหน่วงเวลาในลูป

#--------------------------------------------------------------------------------------------------------------#
# ฟังก์ชัน WebSocket
async def handler(websocket):
    global current_mode, auto_running,yolo_detected
    try:
        async for message in websocket:
            data = json.loads(message)
            print(f"Received: {data}")

            if data['status'] == "manual":
                current_mode = "manual"
                auto_running = False  # ปิดโหมดอัตโนมัติ
                stop_movement()  # หยุดการทำงานของมอเตอร์ในโหมดอัตโนมัติ
                print("Switched to manual mode.")
                # หยุดการทำงานของ YOLO และ auto mode
                yolo_detected = False
                
            elif data['status'] == "auto":
                current_mode = "auto"
                auto_running = True 
                auto_thread = threading.Thread(target=auto_mode)
                auto_thread.start()# เปิดโหมดอัตโนมัติ
                print("Switched to auto mode.")
            
            elif data['status'] == "stop":
                stop_all()  # เรียกฟังก์ชันหยุดทั้งหมด
                print("Switched to stop mode.")
            
            # การควบคุมในโหมด manual เท่านั้น
            if current_mode == "manual":
                if data['status'] == "w":
                    move_forward()
                elif data['status'] == "s":
                    move_backward()
                elif data['status'] == "a":
                    turn_left()
                elif data['status'] == "d":
                    turn_right()
                elif data['status'] == "stop":
                    stop_all()
    except websockets.exceptions.ConnectionClosedError as e:
        print(f"Connection closed: {e}")

async def main():
    print("WebSocket server starting...")
    async with websockets.serve(handler, "", 1234):
        await asyncio.Future()

def start_websocket_server():
    asyncio.run(main())

def frame_grabber():
    global latest_frame
    while True:
        ret, frame = camera.read()
        if not ret:
            continue
        latest_frame = frame.copy()
        time.sleep(0.01)
        


if __name__ == "__main__":
    # สร้างโฟลเดอร์สำหรับเก็บภาพถ่ายหากยังไม่มี
    if not os.path.exists("captures"):
        os.makedirs("captures")

    # เริ่มเธรดสำหรับ WebSocket server
    websocket_thread = threading.Thread(target=start_websocket_server)
    websocket_thread.start()

    # เริ่มเธรดสำหรับการอ่านเฟรมจากกล้อง
    frame_grabber_thread = threading.Thread(target=frame_grabber)
    frame_grabber_thread.start()


    # เริ่มเธรดสำหรับโหมดอัตโนมัติ
    auto_thread = threading.Thread(target=auto_mode)
    auto_thread.start()

    # เริ่มเธรดสำหรับการตรวจจับป้ายจราจรด้วย YOLO
    yolo_thread = threading.Thread(target=yolo_inference)
    yolo_thread.start()
    
    # # # # เริ่มเธรดสำหรับ gyroscope
    # gyro_thread = threading.Thread(target=gyro_control)
    # gyro_thread.start()
    
    try:
        # เริ่ม Flask server
        app.run(host="0.0.0.0", port=5000)
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        # ปิดกล้องและทำความสะอาด GPIO
        camera.release()
