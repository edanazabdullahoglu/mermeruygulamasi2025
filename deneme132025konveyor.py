import shutil
from typing import Literal
from PySide2.QtWidgets import *
import sys
from PySide2 import QtGui
from PySide2.QtMultimedia import *
from PySide2.QtMultimediaWidgets import *
import cv2
import time
from PySide2.QtCore import *
import numpy as np
from PySide2.QtGui import QImage, QPixmap, QPen,QIntValidator, QDoubleValidator, QTransform , QImageReader
from PySide2.QtCore import *
from PySide2.QtMultimedia import *

from pymodbus.payload import BinaryPayloadBuilder
from pymodbus.constants import Endian   
from pymodbus.client import ModbusTcpClient
import pymodbus.client
from struct import *
import struct

import os
from datetime import datetime 
from glob import glob
from PySide2.QtGui import *
from random import randrange
from PySide2.QtUiTools import QUiLoader

from pyModbusTCP.client import ModbusClient

import pywinusb.hid as hid
from usb import backend 
import usb
import subprocess

from CamOperation_class import CameraOperation
from MvCameraControl_class import *
from MvErrorDefine_const import *
from CameraParams_header import *

from functools import partial

########################################################################
# IMPORT GUI to PY FILE
# from ui_mermer_gui_list_camera import Ui_MainWindow

# IMPORT GUI to PY FILE konveyor 2025 devam
# from mermer_gui_cam_yedek_nocam_finall_2025devam_konveyor import 

# 09.05.2025 mücahid hoca revize 
from mermermucahidrevizekonveyor2025 import Ui_MainWindow

#from ui_mermer_crop_image import Ui_Window_CropMonitor
########################################################################
# IMPORT ERROR GUI to PY FILE
from mermer_error_message_box import *

# 获取选取设备信息的索引，通过[]之间的字符去解析
def TxtWrapBy(start_str, end, all):
    start = all.find(start_str)
    if start >= 0:
        start += len(start_str)
        end = all.find(end, start)
        if end >= 0:
            return all[start:end].strip()

# 将返回的错误码转换为十六进制显示
def ToHexStr(num):
    chaDic = {10: 'a', 11: 'b', 12: 'c', 13: 'd', 14: 'e', 15: 'f'}
    hexStr = ""
    if num < 0:
        num = num + 2 ** 32
    while num >= 16:
        digit = num % 16
        hexStr = chaDic.get(digit, str(digit)) + hexStr
        num //= 16
    hexStr = chaDic.get(num, str(num)) + hexStr
    return hexStr

class MainLoadWindow(QMainWindow):

    def __init__(self):
        super(MainLoadWindow, self).__init__()
        
        self.error_message_box = ErrorMessageBox(self)
        self.information_messag_box=  ErrorMessageBox(self)
        self.error_inforrmation_box= ErrorMessageBox(self)
            
        self.selected_folder_path = None
        self.begin, self.destination = QPoint(), QPoint()

        global deviceList
        deviceList = MV_CC_DEVICE_INFO_LIST()
        global cam
        cam = MvCamera()
        global nSelCamIndex
        nSelCamIndex = 0
        global obj_cam_operation
        obj_cam_operation = 0
        global isOpen
        isOpen = False
        global isGrabbing
        isGrabbing = False
        global isCalibMode  # 是否是标定模式（获取原始图像）
        isCalibMode = True
        global isPlcBasicControlConnection 
        isPlcBasicControlConnection = False
        global plcstarttime
        
        # Kamera özellikleri (MV-CS050-10UC)
        self.sensor_width_mm = 8.8  # 2/3" sensör genişliği (mm)
        self.sensor_height_mm = 6.6  # 2/3" sensör yüksekliği (mm)
        self.pixel_size_um = 3.45  # Pixel boyutu (mikrometre)
        self.image_width_pixels = 2448  # Görüntü genişliği (pixel)
        self.image_height_pixels = 2048  # Görüntü yüksekliği (pixel)
        
        # Varsayılan çalışma mesafesi ve odak uzaklığı (mm)
        # Bu değerleri kurulumunuza göre ayarlayın
        self.working_distance_mm = 500  # Kameradan nesneye mesafe (mm) - değiştirilebilir
        self.focal_length_mm = 25  # Lens odak uzaklığı (mm) - kullandığınız lense göre değiştirin
        
        # GSD hesapla
        self.calculate_gsd()

        self.pen = QPen()
        self.pen.setColor(Qt.red)
        self.pen.setWidth(2)  # Kare kalınlığını 2 piksel olarak ayarla
        self.start = QPoint()
        self.end = QPoint()
        self.setMouseTracking(True)
        self.mousePreseed = False
        self.rect, self.line = None, None
        self.image_item = None
        self.all_rects = []  # Karelerin köşe konumlarını saklamak için liste
        self.all_rects_cm = []  # Karelerin cm cinsinden değerlerini saklamak için
        self.drawing_enabled = False 
        self.rectengles_boder = []
        self.square_drawing = False 
        self.last_x = None
        self.last_y = None
        self.square_heights = [] 
        self.plcstandart=ModbusClient()
        

        # Define folder path
        self.folder_path = "D:\OneDrive\Masaüstü\Mermer_Projesi_Final_Tasarim\edanaz"
        
        # Timer oluştur
        timer = QTimer(self)
        timer.timeout.connect(self.update_time)
        timer.start(1000)  # 1 saniyede bir güncelle
       
        self.readtimer= QTimer(self)
        self.readtimer.timeout.connect(self.m1Direction(11))
        self.readtimer.start(1000)

        self.readtimers= QTimer(self)
        self.readtimers.timeout.connect(self.m1Speed(12))
        self.readtimers.start(1000)
        try:
            self.Main_Load_Window()
            # raise Exception() # burası try kısmını denemek için exept yapıyor ve hata atıyor 
        except Exception as errormessage:
            error_message_st = f"An error occurred during startup: {str(errormessage)}"
            error_icon_st = QMessageBox.Icon.Warning
            self.error_message_box.show_error(error_message=error_message_st, title="Startup Error", error_icon=error_icon_st)

    def calculate_gsd(self):
        """GSD (Ground Sample Distance) hesapla - mm/pixel"""
        # GSD = (pixel_size_um * working_distance_mm) / focal_length_mm
        self.gsd_mm_per_pixel = (self.pixel_size_um / 1000) * self.working_distance_mm / self.focal_length_mm
        self.gsd_cm_per_pixel = self.gsd_mm_per_pixel / 10
        print(f"GSD: {self.gsd_mm_per_pixel:.4f} mm/pixel veya {self.gsd_cm_per_pixel:.4f} cm/pixel")
    
    def cm_to_pixel(self, cm_value):
        """CM değerini pixel değerine çevir"""
        return int(cm_value / self.gsd_cm_per_pixel)
    
    def pixel_to_cm(self, pixel_value):
        """Pixel değerini CM değerine çevir"""
        return pixel_value * self.gsd_cm_per_pixel
    
    def update_camera_parameters(self):
        """Kamera parametrelerini güncelle (GUI'den alınabilir)"""
        try:
            # Eğer GUI'de input alanları varsa oradan alabilirsiniz
            # self.working_distance_mm = float(self.ui.working_distance_input.text())
            # self.focal_length_mm = float(self.ui.focal_length_input.text())
            self.calculate_gsd()
        except:
            pass

    def Main_Load_Window(self):
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.show()
        # Delete contents of the folder
        self.clear_folder()
        self.update_time()

        '''loader = QUiLoader()
        self.ui = loader.load("mermer_gui.ui", None)
        self.show()'''
        
        # Sadece sayı alma işlevi ekledim - Float validator kullanıyoruz artık cm için
        float_validator = QDoubleValidator()
        self.ui.lineEdit.setValidator(float_validator)  # Genişlik (cm)
        self.ui.lineEdit_2.setValidator(float_validator)  # Yükseklik (cm)
        # self.ui.lineEdit_3.setValidator(float_validator)  # Testere boyu (cm) - eğer kullanılırsa
        # self.ui.lineEdit_4.setValidator(float_validator)
        
        # Connect mouse press and move events to the graphicsView
        self.ui.graphicsView.mousePressEvent = self.mousePressEvent
        self.ui.graphicsView.mouseMoveEvent = self.mouseMoveEvent
        
        ####################################################
        # Create a context menu for squares
        """self.square_context_menu = QMenu(self)
        self.delete_square_action = self.square_context_menu.addAction("Delete")
        self.delete_square_action.triggered.connect(self.delete_square)

        # Connect the context menu to the graphicsView
        for name, graphicsView in self.ui.__dict__.items():
            if isinstance(graphicsView, QGraphicsView):
                self.ui.graphicsView.setContextMenuPolicy(Qt.CustomContextMenu)
                self.ui.graphicsView.customContextMenuRequested.connect(self.show_square_context_menu)"""
        ###################################################
         # QComboBox'i görünmez yap
        # self.ui.ComboDevices.setVisible(False)
        self.ui.bnJpg.clicked.connect(self.save_jpg)
        self.ui.bnOpen.clicked.connect(self.enum_devices)
        self.ui.bnClose.clicked.connect(self.close_device)
        self.ui.bnJpg.setEnabled(False)
        self.ui.bnClose.setEnabled(False)
        self.ui.pushButton_32.setEnabled(False)

        self.ui.graphicsView.setMouseTracking(False)
        self.ui.graphicsView.mousePressEvent = self.mousePressEvent
        self.ui.graphicsView.mouseMoveEvent = self.mouseMoveEvent
        self.ui.graphicsView.mouseReleaseEvent = self.mouseReleaseEvent
        self.ui.pushButton_32.clicked.connect(self.drawing_square_text)
        self.ui.plcbaslat.clicked.connect(lambda:self.sendPlcStartCommand(10,1))
        self.ui.plcdurdur.clicked.connect(lambda:self.sendPlcStopCommand(10,0))
        self.ui.forwardconveyor.clicked.connect(lambda:self.sendPlcDirection(11,1))
        self.ui.reverseconveyor.clicked.connect(lambda:self.sendPlcDirection(11,-1))
        self.ui.pluspeedconveyor.clicked.connect(lambda:self.sendPlcSpeedPlus(12,1))
        self.ui.decreeasepedconveyor.clicked.connect(lambda:self.sendPlcSpeedeksi(12,1))
        self.ui.pushButton.clicked.connect(self.undo_last_square)
        
        # PLC'ye kare koordinatlarını CM olarak gönder butonu
        self.ui.pushButton_5.clicked.connect(self.send_corners_to_plc_with_cm)
        
        # PLC bağlantısını başlat (Modbus TCP)
        self.connect_plc("192.168.3.50", 502, True)
        ########################################################

    def update_time(self):
        current_time = QTime.currentTime()
        current_date = QDate.currentDate()
        time_text = current_time.toString(Qt.DefaultLocaleLongDate)
        date_text = current_date.toString(Qt.DefaultLocaleLongDate)
        self.ui.label_9.setText(time_text)
        self.ui.label_10.setText(date_text)

################################################################################################################
    def clear_folder(self):
        if os.path.exists(self.folder_path):
            shutil.rmtree(self.folder_path)
            os.makedirs(self.folder_path) 
            
    # 绑定下拉列表至设备信息索引
    def xFunc(event,self):
        global nSelCamIndex
        # nSelCamIndex = TxtWrapBy("[", "]", self.ui.ComboDevices.get())

    # Decoding Characters
    def decoding_char(self,c_ubyte_value):
        c_char_p_value = ctypes.cast(c_ubyte_value, ctypes.c_char_p)
        try:
            decode_str = c_char_p_value.value.decode('gbk')  # Chinese characters
        except UnicodeDecodeError:
            decode_str = str(c_char_p_value.value)
        return decode_str

    # ch:枚举相机 | en:enum devices
    def enum_devices(self):
        global deviceList
        global obj_cam_operation
        
        deviceList = MV_CC_DEVICE_INFO_LIST()
        ret = MvCamera.MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, deviceList)
        if ret != 0:
            strError = "Enum devices fail! ret = :" + ToHexStr(ret)
            QMessageBox.warning(self, "Error", strError, QMessageBox.Ok)
            return ret

        if deviceList.nDeviceNum == 0:
            QMessageBox.warning(self, "Info", "Find no device", QMessageBox.Ok)
            return ret
        print("Find %d devices!" % deviceList.nDeviceNum)

        devList = []
        for i in range(0, deviceList.nDeviceNum):
            mvcc_dev_info = cast(deviceList.pDeviceInfo[i], POINTER(MV_CC_DEVICE_INFO)).contents
            if mvcc_dev_info.nTLayerType == MV_GIGE_DEVICE:
                print("\ngige device: [%d]" % i)
                user_defined_name = self.decoding_char(mvcc_dev_info.SpecialInfo.stGigEInfo.chUserDefinedName)
                model_name = self.decoding_char(mvcc_dev_info.SpecialInfo.stGigEInfo.chModelName)
                print("device user define name: " + user_defined_name)
                print("device model name: " + model_name)

                nip1 = ((mvcc_dev_info.SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24)
                nip2 = ((mvcc_dev_info.SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16)
                nip3 = ((mvcc_dev_info.SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8)
                nip4 = (mvcc_dev_info.SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff)
                print("current ip: %d.%d.%d.%d " % (nip1, nip2, nip3, nip4))
                devList.append(
                    "[" + str(i) + "]GigE: " + user_defined_name + " " + model_name + "(" + str(nip1) + "." + str(
                        nip2) + "." + str(nip3) + "." + str(nip4) + ")")
                
            elif mvcc_dev_info.nTLayerType == MV_USB_DEVICE:
                print("\nu3v device: [%d]" % i)
                user_defined_name = self.decoding_char(mvcc_dev_info.SpecialInfo.stUsb3VInfo.chUserDefinedName)
                model_name = self.decoding_char(mvcc_dev_info.SpecialInfo.stUsb3VInfo.chModelName)
                print("device user define name: " + user_defined_name)
                print("device model name: " + model_name)

                strSerialNumber = ""
                for per in mvcc_dev_info.SpecialInfo.stUsb3VInfo.chSerialNumber:
                    if per == 0:
                        break
                    strSerialNumber = strSerialNumber + chr(per)
                print("user serial number: " + strSerialNumber)
                devList.append("[" + str(i) + "]USB: " + user_defined_name + " " + model_name
                               + "(" + str(strSerialNumber) + ")")

        # self.ui.ComboDevices.clear()
        # self.ui.ComboDevices.addItems(devList)
        # self.ui.ComboDevices.setCurrentIndex(0)
        self.open_device()

    # ch:打开相机 | en:open device
    def open_device(self):
        global deviceList
        global nSelCamIndex
        global obj_cam_operation
        global isOpen
        if isOpen:
            QMessageBox.warning(self, "Error", 'Camera is Running!', QMessageBox.Ok)
            return MV_E_CALLORDER

        # nSelCamIndex = self.ui.ComboDevices.currentIndex()
        if nSelCamIndex < 0:
            QMessageBox.warning(self, "Error", 'Please select a camera!', QMessageBox.Ok)
            return MV_E_CALLORDER

        obj_cam_operation = CameraOperation(cam, deviceList, nSelCamIndex)
        ret = obj_cam_operation.Open_device()
        
        if 0 != ret:
            strError = "Open device failed ret:" + ToHexStr(ret)
            QMessageBox.warning(self, "Error", strError, QMessageBox.Ok)
            isOpen = False
        else:
            self.set_continue_mode()
            isOpen = True
             
    # ch:开始取流 | en:Start grab image
    def start_grabbing(self):
        global obj_cam_operation
        ret = obj_cam_operation.Start_grabbing()#self.ui.widgetDisplay.winId())
        self.ui.bnOpen.setStyleSheet(u"background-color: green;")
        self.ui.bnJpg.setEnabled(True)
        self.ui.bnClose.setEnabled(True)
        if ret != 0:
            strError = "Start grabbing failed ret:" + ToHexStr(ret)
            QMessageBox.warning(self, "Error", strError, QMessageBox.Ok)
       
    # ch:停止取流 | en:Stop grab image
    def stop_grabbing(self):
        global obj_cam_operation
        #global isGrabbing
        ret = obj_cam_operation.Stop_grabbing()
        self.close_device()
        self.ui.bnClose.setEnabled(False)
        self.ui.bnJpg.setEnabled(False)
        

        if ret != 0:
            strError = "Stop grabbing failed ret:" + ToHexStr(ret)
            QMessageBox.warning(self, "Error", strError, QMessageBox.Ok)
            
    # ch:关闭设备 | Close device
    def close_device(self):
        global isOpen   
        if isOpen:
            obj_cam_operation.Close_device()
            self.clear_folder()
            isOpen = False
            self.ui.bnJpg.setEnabled(False)
            self.ui.bnClose.setEnabled(False)
            self.ui.bnOpen.setStyleSheet(u"background-color: red;")
              
    # ch:设置触发模式 | en:set trigger mode
    def set_continue_mode(self):
        strError = None
        ret = obj_cam_operation.Set_trigger_mode(False)
        self.start_grabbing()
        if ret != 0:
            strError = "Set continue mode failed ret:" + ToHexStr(ret) + " mode is " + str(self.is_trigger_mode)
            QMessageBox.warning(self, "Error", strError, QMessageBox.Ok)
        
    '''# ch:存图 | en:save image
    def save_bmp(self):
        ret = obj_cam_operation.Save_Bmp()
        if ret != MV_OK:
            strError = "Save BMP failed ret:" + ToHexStr(ret)
            QMessageBox.warning(self, "Error", strError, QMessageBox.Ok)
        else:
            print("Save image success")'''

    def save_jpg(self):
        self.drawing_enabled = False
        self.mousePressed = False
        self.rect = None
        ret = obj_cam_operation.Save_jpg()
        
        if ret != MV_OK:
            strError = "Save JPG failed ret:" + ToHexStr(ret)
            QMessageBox.warning(self, "Error", strError, QMessageBox.Ok)
        else:
            print("Save image success")
        self.show_largest_image_in_graphics_view()
  
    def show_largest_image_in_graphics_view(self):
        self.ui.image_list_widget.clear()
        self.rectengles_boder.clear()
        self.ui.image_list_widget.clear()
        self.ui.bnJpg.setEnabled(False)
        self.ui.pushButton_32.setEnabled(False)
        
        
        self.ui.listWidget_2.clear()  
        # Klasör yolunu kontrol edin
        folder_path = "D:\OneDrive\Masaüstü\Mermer_Projesi_Final_Tasarim\edanaz"
        
        # Klasör içindeki dosya isimlerini listeleme
        filenames = os.listdir(folder_path)
        print("Files in folder:", filenames)

        # Klasördeki dosyaları kontrol ederek en büyüğünü bulun
        largest_filename = ""
        largest_number = -1
        for filename in filenames:
            if filename.lower().endswith((".jpg", )):
                number_str = filename.split(".")[0]  # Dosya adından uzantıyı kaldırın
                if number_str.isdigit():
                    number = int(number_str)
                    if number > largest_number:
                        largest_number = number
                        largest_filename = filename
        print("Largest filename:", largest_filename)

        # En büyük sayıya sahip dosyayı yükleyin ve QGraphicsView'da gösterin
        if largest_filename:
            image_path = os.path.join(folder_path, largest_filename)
            print("Image path:", image_path)
            if os.path.exists(image_path):
                image = QImage(image_path)
                print("Image loaded successfully")
                self.pixmap = QPixmap.fromImage(image)
                self.scene = QGraphicsScene()
                self.scene.addPixmap(self.pixmap)
                self.ui.graphicsView.setScene(self.scene)
                self.ui.graphicsView.fitInView(self.scene.sceneRect())
                self.ui.graphicsView.show()
                view_size = self.ui.graphicsView.size()
                print(view_size)
                print(self.pixmap.width(), self.pixmap.height())
                self.drawing_enabled = True
                self.ui.bnJpg.setEnabled(True)
                

            else:
                QMessageBox.warning(self, "HATA", "Resim Bulunamadı.", QMessageBox.Ok)
        else:
            QMessageBox.warning(self, "HATA", "Dosya İçerisinde Resim Bulunamadı.", QMessageBox.Ok)
 
    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton and self.drawing_enabled:
            self.mousePressed = True
            self.start = self.ui.graphicsView.mapToScene(event.pos())
    
    """def mouseMoveEvent(self, event):
        if event.buttons() & Qt.LeftButton and self.drawing_enabled and self.mousePressed :
            self.end = self.ui.graphicsView.mapToScene(event.pos())
            self.drawShape()"""
    
    def mouseMoveEvent(self, event):
        '''if event.buttons() & Qt.LeftButton and self.drawing_enabled and self.mousePressed:
            # Fare imleci resmin sınırlarını aşarsa, karenin koordinatlarını sınırlara çek
            pos = self.ui.graphicsView.mapToScene(event.pos())
            
            # Yeni konumu sınırlara çek
            new_pos = QPoint(
                max(0, min(pos.x(), self.pixmap.width())),
                max(0, min(pos.y(), self.pixmap.height()))
            )
            # Kareyi çizmek için yeni konumu kullan
            self.end = new_pos
            self.drawShape()
        '''
        if event.buttons() & Qt.LeftButton and self.drawing_enabled and self.mousePressed:
            self.end = self.ui.graphicsView.mapToScene(event.pos())

            # Check if end point exceeds image dimensions
            image_width = self.pixmap.width()
            image_height = self.pixmap.height()

            if self.end.x() < 0 and self.end.y() < 0:
                self.end.setX(0)
                self.end.setY(0)
            elif self.end.x() < 0:
                self.end.setX(0)
            elif self.end.x() > image_width:
                self.end.setX(image_width)
            if self.end.y() < 0:
                self.end.setY(0)
            elif self.end.y() > image_height:
                self.end.setY(image_height)
            self.update()
            self.drawShape()
            
    def drawShape(self):
        if self.ui.graphicsView.scene() is None:
            return  # Sahne yoksa çizim yapma
        if self.start.isNull() or self.end.isNull():
            return
        if self.start.x() == self.end.x() and self.start.y() == self.end.y():
            return
        else:
            if self.rect is not None:
                self.ui.graphicsView.scene().removeItem(self.rect)  # Önceki dikdörtgeni kaldır
                self.rect = None
            width = abs(self.start.x() - self.end.x())
            height = abs(self.start.y() - self.end.y())
            x=min(self.start.x(), self.end.x())
            y=min(self.start.y(), self.end.y())
            self.rect = self.ui.graphicsView.scene().addRect(
                x,y,width, height, self.pen)

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.LeftButton  and self.drawing_enabled and self.mousePreseed:
            self.mousePreseed = False
            self.drawShape()
            self.start, self.end = QPoint(), QPoint()
            self.rect, = None
    
    def keyPressEvent(self, event):
        if event.key() == Qt.Key_C:
            # Çizme işlemini devre dışı bırak
            self.drawing_enabled = False
            self.calculate_corners_of_rectangle()
            self.square_drawing = True
            self.ui.pushButton_32.setEnabled(True)
        
    def calculate_corners_of_rectangle(self):
        if self.rect is not None:
            # Resmin boyutunu al
            self.image_width = self.pixmap.width()
            self.image_height = self.pixmap.height()

            # Kare boyutunu ve konumunu al
            self.kare = self.rect.rect()

            # Kare genişliği ve yüksekliği (pixel)
            self.mermer_alani_width = self.kare.width()
            self.mermer_alani_height = self.kare.height()
            self.mermer_alani = self.kare.size()
            
            # CM cinsinden değerleri hesapla
            self.mermer_alani_width_cm = self.pixel_to_cm(self.mermer_alani_width)
            self.mermer_alani_height_cm = self.pixel_to_cm(self.mermer_alani_height)
            
            print(f"Mermer Alanı - Pixel: {self.mermer_alani_height}px x {self.mermer_alani_width}px")
            print(f"Mermer Alanı - CM: {self.mermer_alani_height_cm:.2f}cm x {self.mermer_alani_width_cm:.2f}cm")
            
            # Kareyi tanımlayan dört köşe noktasını hesapla
            self.x1 = self.kare.left()
            self.y1 = self.kare .top()
            self.x2 = self.kare .right()
            self.y2 = self.kare .top()
            self.x3 = self.kare .right()
            self.y3 = self.kare .bottom()
            self.x4 = self.kare .left()
            self.y4 = self.kare .bottom()

            self.last_x = self.x1
            self.last_y = self.y1
            
            # Köşe noktalarını ana listeye ekle (hem pixel hem cm olarak)
            corners = [{'sol_üst':(self.x1, self.y1), 'Sağ_üst':(self.x2, self.y2), 
                       'Sağ_Alt':(self.x3, self.y3), 'Sol_alt':(self.x4, self.y4),
                       'Alan_CM': f"{self.mermer_alani_width_cm:.2f} x {self.mermer_alani_height_cm:.2f}"}]
            self.rectengles_boder.append(corners)

            # Ana liste öğelerini gösteren bir liste widget'ına ekle
            for item in self.rectengles_boder:
                self.ui.image_list_widget.addItem(str(item))
    
    def drawing_square_text(self):
        if self.square_drawing == True:                   
            # Girilen genişlik ve yükseklik değerlerini al (CM cinsinden)
            self.square_width_text = self.ui.lineEdit.text()
            self.square_height_text = self.ui.lineEdit_2.text()
            # self.square_testere_text = self.ui.lineEdit_3.text()  # Testere boyu CM cinsinden

            if not self.square_width_text or not  self.square_height_text:
                QMessageBox.warning(self,"Uyarı", "Lütfen Boş Değer Girmeyiniz!")
                return 

            # CM cinsinden değerleri al
            self.square_width_cm = float(self.square_width_text)
            self.square_height_cm = float(self.square_height_text)
            self.square_aci = 0
            self.square_testere_cm = 1.5  # Testere boyu 1.5 cm (varsayılan)
            
            # CM'den pixel'e çevir
            self.square_widht = self.cm_to_pixel(self.square_width_cm)
            self.square_height = self.cm_to_pixel(self.square_height_cm)
            self.square_testere = self.cm_to_pixel(self.square_testere_cm)
            
            print(f"Kare Boyutları - CM: {self.square_width_cm}cm x {self.square_height_cm}cm")
            print(f"Kare Boyutları - Pixel: {self.square_widht}px x {self.square_height}px")
            
            if self.square_widht > self.mermer_alani_width or self.square_height > self.mermer_alani_height:
                QMessageBox.warning(self,"Uyarı","Mermer Alanının Genişlik ve Yükseklik Değerinden Küçük Değer Giriniz")
                return 
            
            if self.square_width_cm <= 0 or self.square_height_cm <= 0:
                QMessageBox.warning(self, "Uyarı", "Lütfen 0 dan Büyük Değerler Giriniz", QMessageBox.Ok)
                return
            
            self.square_heights.append(self.square_height)

            # Kareyi ekranın altına yerleştirmek için kontrol
            if self.last_x + self.square_widht > self.x1 + self.mermer_alani_width:
                self.last_x = self.x1  # Ekranın soluna al
                max_height = max(self.square_heights)  # Listenin içindeki en büyük yüksekliği al
                self.last_y = self.last_y + max_height + self.square_testere  # En alttan testere boşluğu kadar aşağıya indir
                self.square_heights.clear() 

            # Kareyi çiz
            square = QGraphicsRectItem(self.last_x, self.last_y, self.square_widht, self.square_height)
            
            self.marble_area = QRectF(self.x1, self.y1, self.mermer_alani_width, self.mermer_alani_height) 

            # Kontrol: Kare mermer alanının içinde mi?
            if self.marble_area.contains(square.rect()):
                square.setPen(QPen(Qt.green))
                self.ui.graphicsView.scene().addItem(square)
                
                # Pixel koordinatları
                self.corners = [
                    {'x1': self.last_x, 'y1': self.last_y},
                    {'x2': self.last_x + self.square_widht, 'y2': self.last_y},
                    {'x3': self.last_x + self.square_widht, 'y3': self.last_y + self.square_height},
                    {'x4': self.last_x, 'y4': self.last_y + self.square_height}
                ]
                
                # CM cinsinden koordinatları
                self.corners_cm = [
                    {'x1_cm': self.pixel_to_cm(self.last_x), 'y1_cm': self.pixel_to_cm(self.last_y)},
                    {'x2_cm': self.pixel_to_cm(self.last_x + self.square_widht), 'y2_cm': self.pixel_to_cm(self.last_y)},
                    {'x3_cm': self.pixel_to_cm(self.last_x + self.square_widht), 'y3_cm': self.pixel_to_cm(self.last_y + self.square_height)},
                    {'x4_cm': self.pixel_to_cm(self.last_x), 'y4_cm': self.pixel_to_cm(self.last_y + self.square_height)}
                ]
                
                self.all_rects.append({
                    "corners": self.corners,
                    "rect_item": square,
                    "width_cm": self.square_width_cm,
                    "height_cm": self.square_height_cm
                })
                
                self.all_rects_cm.append({
                    "corners_cm": self.corners_cm,
                    "width_cm": self.square_width_cm,
                    "height_cm": self.square_height_cm
                })
                
                self.last_x += self.square_widht + self.square_testere
                self.update_list_widget()
                
            else:
                QMessageBox.warning(self,"Uyarı", "Mermer Alanı Dışına kare eklenemez")

    def update_list_widget(self):
        # QListWidget içeriğini güncelleme
        self.ui.listWidget_2.clear()  # Önceki öğeleri temizleme
        for i, data in enumerate(self.all_rects, start=1):
            width_cm = data["width_cm"]
            height_cm = data["height_cm"]
            corners = data["corners"]
            item = QListWidgetItem(f"Kare {i}: {width_cm:.2f}cm x {height_cm:.2f}cm - Pixel Köşeler: {corners}")
            self.ui.listWidget_2.addItem(item)    
    
    def undo_last_square(self):
        if self.all_rects:
            lastSquare = self.all_rects.pop()
            if self.all_rects_cm:
                self.all_rects_cm.pop()
            newSquarestartx1 = lastSquare["corners"][0]['x1']
            newSquarestarty1 = lastSquare["corners"][0]['y1']
            self.last_x = newSquarestartx1
            self.last_y = newSquarestarty1
            self.update_list_widget()
            
            rect_item = lastSquare["rect_item"]
            self.ui.graphicsView.scene().removeItem(rect_item)
    
    def send_corners_to_plc_with_cm(self):
        """PLC'ye CM cinsinden değerleri gönder"""
        try:
            # PLC'nin Modbus sunucu adresi ve portunu ayarlayın
            host = "192.168.3.50"
            port = 502
            plc = ModbusClient()

            plc.host(host)
            plc.port(port)
            plc.open()

            if plc.is_open():
                address = 0
                for rect_data in self.all_rects_cm:
                    corners_cm = rect_data["corners_cm"]
                    width_cm = rect_data["width_cm"]
                    height_cm = rect_data["height_cm"]
                    
                    # CM değerlerini integer'a çevir (100 ile çarparak 2 ondalık basamak hassasiyeti koruyabilirsiniz)
                    for corner in corners_cm:
                        for key, value in corner.items():
                            # CM değerini 100 ile çarpıp integer'a çevir (örn: 12.34 cm -> 1234)
                            int_value = int(value * 100)
                            plc.write_single_register(address, int_value)
                            address += 1
                    
                    # Genişlik ve yükseklik değerlerini de gönder
                    plc.write_single_register(address, int(width_cm * 100))
                    address += 1
                    plc.write_single_register(address, int(height_cm * 100))
                    address += 1
                    
                print(f"PLC'ye {len(self.all_rects_cm)} kare başarıyla gönderildi (CM cinsinden)")
                
                # Başarılı mesajı göster
                error_message_st = f"PLC'ye {len(self.all_rects_cm)} kare başarıyla gönderildi (CM cinsinden)"
                error_message_title = "BAŞARILI"
                error_icon_st = QMessageBox.Icon.Information
                self.information_messag_box.show_information(error_message_st, error_icon_st, error_message_title)

        except Exception as e:
            print(f"Error: {e}")
            error_message_st = f"PLC'ye veri gönderme hatası: {str(e)}"
            error_message_title = "HATA"
            error_icon_st = QMessageBox.Icon.Critical
            self.error_message_box.show_error(error_message=error_message_st, title=error_message_title, error_icon=error_icon_st)
        
        finally:
            # Bağlantıyı kapat
            plc.close()
            print("Connection closed")
        
    def send_corners_plc(self):
        """Orijinal pixel değerlerini gönderen fonksiyon (geriye uyumluluk için)"""
        try:
            # PLC'nin Modbus sunucu adresi ve portunu ayarlayın
            host = "192.168.3.50"
            port = 502
            plc = ModbusClient()

            plc.host(host)
            plc.port(port)
            plc.open()

            if plc.is_open():
                address = 0
                for corners in self.all_rects:
                    for corner in corners:
                        for key, value in corner.items():
                            # Her köşe için x ve y koordinatlarını PLC'ye gönder
                            plc.write_multiple_registers(address, value)
                            address += 1

        except Exception as e:
            print(f"Error: {e}")
        
        finally:
            # Bağlantıyı kapat
            plc.close()
            print("Connection closed")

    def rotate_square(self, square, angle):
        # Açı değerini al
        angle_deg = int(angle)

        # Kareyi döndürme işlemi
        # Kareyi döndürmeden önce mevcut konumunu al
        current_pos = square.pos()
        # Kareyi döndürmeden önce merkez noktasını hesapla
        center_point = square.rect().center()
        # Kareyi merkezi etrafında döndür
        transform = QTransform().translate(center_point.x(), center_point.y()).rotate(angle_deg).translate(-center_point.x(), -center_point.y())
        square.setTransform(transform)
        # Döndürüldükten sonra mevcut konumunu geri yükle
        square.setPos(current_pos)
    
    def connect_plc(self,host,port, autopen):
        try:
            self.plcstandart=ModbusClient(host=host,port=port,auto_open=autopen) # modbusclienttxp kullanılıabilir onda connect var autopen yok ama 
            if self.plcstandart.is_open():
                self.isPlcBasicControlConnection = True
                error_message_st = f"PLC Baglantisi Yapildi"
                error_message_title= "BILGI"
                error_icon_st = QMessageBox.Icon.Information
                self.error_inforrmation_box.show_information(error_message_st,error_icon_st,error_message_title)
                #bağlantı olması durumunda bir göstergeeyi aktifleştirelim
            else:
                self.isPlcBasicControlConnection = False
                error_message_st = f"PLC Baglantisi Yapildi"
                error_message_title= "UYARI"
                error_icon_st = QMessageBox.Icon.Warning
                self.error_inforrmation_box.show_information(error_message_st,error_icon_st,error_message_title)

        except:
            self.isPlcBasicControlConnection = False
            error_message_st = f"PLC Baglantisi Yapildi"
            error_message_title= "UYARI"
            error_icon_st = QMessageBox.Icon.Warning
            self.error_inforrmation_box.show_information(error_message_st,error_icon_st,error_message_title)

    def sendPlcStartCommand(self,registerno,sendvalue):
        print("burdasın",registerno,sendvalue)
        
        result=self.plcstandart.write_single_register(registerno,sendvalue)
        if result ==True:
            print("burdasın dogr",registerno,sendvalue)
            error_message_st = f"PLC Baslat Komutu Gonderildi"
            error_message_title= "BILGI"
            error_icon_st = QMessageBox.Icon.Information
            self.error_inforrmation_box.show_information(error_message_st,error_icon_st,error_message_title)
        
        else :
            print("burdasın yalis",registerno,sendvalue)
            error_message_st = f"PLC Komutu Gonderilemedi"
            error_message_title= "UYARI"
            error_icon_st = QMessageBox.Icon.Warning
            self.error_inforrmation_box.show_information(error_message_st,error_icon_st,error_message_title)

    def sendPlcStopCommand(self,registerno,sendvalue):
        
        result=self.plcstandart.write_single_register(registerno,sendvalue)
        if result ==True:
            print("burdasın dogr",registerno,sendvalue)
            error_message_st = f"PLC Baslat Komutu Gonderildi"
            error_message_title= "BILGI"
            error_icon_st = QMessageBox.Icon.Information
            self.error_inforrmation_box.show_information(error_message_st,error_icon_st,error_message_title)
        
        else :
            print("burdasın yalis",registerno,sendvalue)
            error_message_st = f"PLC Komutu Gonderilemedi"
            error_message_title= "UYARI"
            error_icon_st = QMessageBox.Icon.Warning
            self.error_inforrmation_box.show_information(error_message_st,error_icon_st,error_message_title)


    def sendPlcDirection(self,registerno,sendvalue):
        result=self.plcstandart.write_single_register(registerno,sendvalue)
        if result ==True:
            print("burdasın dogr",registerno,sendvalue)
            error_message_st = f"PLC Ileri Komutu Gonderildi"
            error_message_title= "BILGI"
            error_icon_st = QMessageBox.Icon.Information
            self.error_inforrmation_box.show_information(error_message_st,error_icon_st,error_message_title)
        
        else :
            print("burdasın yalis",registerno,sendvalue)
            error_message_st = f"PLC Ileri Gonderilemedi"
            error_message_title= "UYARI"
            error_icon_st = QMessageBox.Icon.Warning
            self.error_inforrmation_box.show_information(error_message_st,error_icon_st,error_message_title)
    
    def sendPlcDirection(self,registerno,sendvalue):
        result=self.plcstandart.write_single_register(registerno,sendvalue)
        if result ==True:
            print("burdasın dogr",registerno,sendvalue)
            error_message_st = f"PLC Yon Gonderildi"
            error_message_title= "BILGI"
            error_icon_st = QMessageBox.Icon.Information
            self.error_inforrmation_box.show_information(error_message_st,error_icon_st,error_message_title)
        
        else :
            print("burdasın yalis",registerno,sendvalue)
            error_message_st = f"PLC Yon Gonderilemedi"
            error_message_title= "UYARI"
            error_icon_st = QMessageBox.Icon.Warning
            self.error_inforrmation_box.show_information(error_message_st,error_icon_st,error_message_title)

    def sendPlcSpeedPlus(self,registerno,sendvalue):
        result=self.plcstandart.read_holding_registers(registerno,1)
        
        if result:
            print("burdasın dogr",registerno,sendvalue)
            error_message_st = f"PLC Hiz Atrirma Gonderildi"
            error_message_title= "BILGI"
            error_icon_st = QMessageBox.Icon.Information
            self.error_inforrmation_box.show_information(error_message_st,error_icon_st,error_message_title)
            value= result[0]
            resultsendvalue=sendvalue+value
            result=self.plcstandart.write_single_register(registerno,resultsendvalue)
        
        else :
            print("burdasın yalis",registerno,sendvalue)
            error_message_st = f"PLC Hiz Atrirma Gonderilemedi"
            error_message_title= "UYARI"
            error_icon_st = QMessageBox.Icon.Warning
            self.error_inforrmation_box.show_information(error_message_st,error_icon_st,error_message_title)
    
    def sendPlcSpeedeksi(self,registerno,sendvalue):
        result=self.plcstandart.read_holding_registers(registerno,1)
        
        if result:
            print("burdasın dogr",registerno,sendvalue)
            error_message_st = f"PLC Hiz Azaltma Gonderildi"
            error_message_title= "BILGI"
            error_icon_st = QMessageBox.Icon.Information
            self.error_inforrmation_box.show_information(error_message_st,error_icon_st,error_message_title)
            value= result[0]
            resultsendvalue=value-sendvalue
            result=self.plcstandart.write_single_register(registerno,resultsendvalue)
        
        else :
            print("burdasın yalis",registerno,sendvalue)
            error_message_st = f"PLC Hiz Azaltma Gonderilemedi"
            error_message_title= "UYARI"
            error_icon_st = QMessageBox.Icon.Warning
            self.error_inforrmation_box.show_information(error_message_st,error_icon_st,error_message_title)

    def m1Direction(self,registerno):
        result=self.plcstandart.read_holding_registers(registerno)
        if result:
            value= result[0]
            self.ui.m1direction.setText(str(value))
        print("eda")

    def m1Speed(self,registerno):
        print("eda")
        result=self.plcstandart.read_holding_registers(registerno)
        if result:
            value= result[0]
            self.ui.m1speed.setText(str(value))

if __name__ == "__main__":
    app = QApplication([])
    window = MainLoadWindow()
    sys.exit(app.exec_())