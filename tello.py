import socket
import threading
import time
import cv2
import numpy as np
from easytello.stats import Stats

class Tello:
    def __init__(self, tello_ip: str='192.168.10.1', debug: bool=True):
        # Opening local UDP port on 8889 for Tello communication
        self.local_ip = ''
        self.local_port = 8889
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind((self.local_ip, self.local_port))
        
        # Setting Tello ip and port info
        self.tello_ip = tello_ip
        self.tello_port = 8889
        self.tello_address = (self.tello_ip, self.tello_port)
        self.log = []

        # Intializing response thread
        self.receive_thread = threading.Thread(target=self._receive_thread)
        self.receive_thread.daemon = True
        self.receive_thread.start()

        # easyTello runtime options
        self.stream_state = False
        self.MAX_TIME_OUT = 15.0
        self.debug = debug
        # Setting Tello to command mode
        #self.command()
        self.streamon()

    def send_command(self, command: str, query: bool =False):
        # New log entry created for the outbound command
        self.log.append(Stats(command, len(self.log)))

        # Sending command to Tello
        self.socket.sendto(command.encode('utf-8'), self.tello_address)
        # Displaying conformation message (if 'debug' os True)
        if self.debug is True:
            print('Sending command: {}'.format(command))
            
        # Checking whether the command has timed out or not (based on value in 'MAX_TIME_OUT')
        start = time.time()
        while not self.log[-1].got_response():  # Runs while no repsonse has been received in log
            now = time.time()
            difference = now - start
            if difference > self.MAX_TIME_OUT:
                print('Connection timed out!')
                break
        # Prints out Tello response (if 'debug' is True)
        if self.debug is True and query is False:
            print('Response: {}'.format(self.log[-1].get_response()))

    def _receive_thread(self):
        while True:
            # Checking for Tello response, throws socket error
            try:
                self.response, ip = self.socket.recvfrom(1024)
                self.log[-1].add_response(self.response)
            except socket.error as exc:
                print('Socket error: {}'.format(exc))

    def _video_thread(self):
        # Creating stream capture object
        cap = cv2.VideoCapture('udp://'+self.tello_ip+':11111')

        # # 人臉辨識
        # # 載入分類器
        # face_cascade = cv2.CascadeClassifier('C:\\Users\\user\\AppData\\Local\\Programs\\Python\\Python38\\Lib\\site-packages\\cv2\\data\\haarcascade_frontalface_default.xml')
        # # 將圖片轉為灰階
        # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        #
        # faces = face_cascade.detectMultiScale(gray, 1.1, 4)
        # for (x, y, w, h) in faces:
        #     cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)  # 畫出外框

        # 移動偵測
        ret, frame = cap.read()
        avg = cv2.blur(frame, (4, 4))
        avg_float = np.float32(avg)

        # Runs while 'stream_state' is True
        while self.stream_state:
            ret, frame = cap.read()

            # 模糊處理
            blur = cv2.blur(frame, (4, 4))

            # 計算目前影格與平均影像的差異值
            difference = cv2.absdiff(avg, blur)

            # 轉成灰階
            gray = cv2.cvtColor(difference, cv2.COLOR_BGR2GRAY)

            # 篩選出變動程度大於門檻值的區域
            ret, thresh = cv2.threshold(gray, 25, 255, cv2.THRESH_BINARY)

            # 使用型態轉換函數去除雜訊
            kernel = np.ones((5, 5), np.uint8)
            thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel, iterations=2)
            thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel, iterations=2)

            # 產生等高線
            cntImg, cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for c in cntImg:
                if cv2.contourArea(c) < 2500:
                    continue

                # 計算等高線外框範圍
                (x, y, w, h) = cv2.boundingRect(c)

                # 畫出外框
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

            # 劃出等高線
            #cv2.drawContours(frame, cntImg, -1, (0, 255, 255), 2)

            cv2.namedWindow('DJI Tello', cv2.WINDOW_NORMAL)                         #正常視窗大小
            cv2.imshow('DJI Tello', frame)                                          #秀出圖片

            # Video Stream is closed if escape key is pressed
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            cv2.accumulateWeighted(blur, avg_float, 0.01)
            avg = cv2.convertScaleAbs(avg_float)

        cap.release()
        cv2.destroyAllWindows()
    
    def wait(self, delay: float):
        # Displaying wait message (if 'debug' is True)
        if self.debug is True:
            print('Waiting {} seconds...'.format(delay))
        # Log entry for delay added
        self.log.append(Stats('wait', len(self.log)))
        # Delay is activated
        time.sleep(delay)
    
    def get_log(self):
        return self.log

    # Controll Commands
    def command(self):
        self.send_command('command')
    
    def takeoff(self):
        self.send_command('takeoff')

    def land(self):
        self.send_command('land')

    def streamon(self):
        self.send_command('streamon')
        self.stream_state = True
        self.video_thread = threading.Thread(target=self._video_thread)
        self.video_thread.daemon = True
        self.video_thread.start()

    def streamoff(self):
        self.stream_state = False
        self.send_command('streamoff')

    def emergency(self):
        self.send_command('emergency')
    
    # Movement Commands
    def up(self, dist: int):
        self.send_command('up {}'.format(dist))

    def down(self, dist: int):
        self.send_command('down {}'.format(dist))

    def left(self, dist: int):
        self.send_command('left {}'.format(dist))

    def right(self, dist: int):
        self.send_command('right {}'.format(dist))
        
    def forward(self, dist: int):
        self.send_command('forward {}'.format(dist))

    def back(self, dist: int):
        self.send_command('back {}'.format(dist))

    def cw(self, degr: int):
        self.send_command('cw {}'.format(degr))
    
    def ccw(self, degr: int):
        self.send_command('ccw {}'.format(degr))

    def flip(self, direc: str):
        self.send_command('flip {}'.format(direc))

    def go(self, x: int, y: int, z: int, speed: int):
        self.send_command('go {} {} {} {}'.format(x, y, z, speed))

    def curve(self, x1: int, y1: int, z1: int, x2: int, y2: int, z2: int, speed: int):
        self.send_command('curve {} {} {} {} {} {} {}'.format(x1, y1, z1, x2, y2, z2, speed))

    # Set Commands
    def set_speed(self, speed: int):
        self.send_command('speed {}'.format(speed))

    def rc_control(self, a: int, b: int, c: int, d: int):
        self.send_command('rc {} {} {} {}'.format(a, b, c, d))

    def set_wifi(self, ssid: str, passwrd: str):
        self.send_command('wifi {} {}'.format(ssid, passwrd))

    # Read Commands
    def get_speed(self):
        self.send_command('speed?', True)
        return self.log[-1].get_response()

    def get_battery(self):
        self.send_command('battery?', True)
        return self.log[-1].get_response()

    def get_time(self):
        self.send_command('time?', True)
        return self.log[-1].get_response()

    def get_height(self):
        self.send_command('height?', True)
        return self.log[-1].get_response()
    
    def get_temp(self):
        self.send_command('temp?', True)
        return self.log[-1].get_response()

    def get_attitude(self):
        self.send_command('attitude?', True)
        return self.log[-1].get_response()

    def get_baro(self):
        self.send_command('baro?', True)
        return self.log[-1].get_response()

    def get_acceleration(self):
        self.send_command('acceleration?', True)
        return self.log[-1].get_response()
    
    def get_tof(self):
        self.send_command('tof?', True)
        return self.log[-1].get_response()

    def get_wifi(self):
        self.send_command('wifi?', True)
        return self.log[-1].get_response()

    # def get_status(self):
    #     return Stats(command='').print_stats()
    
