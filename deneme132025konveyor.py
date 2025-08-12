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
        
        # Sadece sayı alma işlevi ekledim
        int_validator = QIntValidator()
        float_validator = QDoubleValidator ()
        self.ui.lineEdit.setValidator(QIntValidator())
        self.ui.lineEdit_2.setValidator(int_validator)
        # self.ui.lineEdit_3.setValidator(int_validator)
        # self.ui.lineEdit_4.setValidator(int_validator)
        
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

            # Kare genişliği ve yüksekliği
            self.mermer_alani_width = self.kare.width()
            self.mermer_alani_height = self.kare.height()
            self.mermer_alani = self.kare.size()
            print (self.mermer_alani_height)
            
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
            
            
            print(self.mermer_alani_height,self.mermer_alani_width)
            

            # Köşe noktalarını ana listeye ekle
            corners = [{'sol_üst':(self.x1, self.y1), 'Sağ_üst':(self.x2, self.y2), 'Sağ_Alt':(self.x3, self.y3), 'Sol_alt':(self.x4, self.y4)}]
            self.rectengles_boder.append(corners)

            # Ana liste öğelerini gösteren bir liste widget'ına ekle
            for item in self.rectengles_boder:
                self.ui.image_list_widget.addItem(str(item))
                    
    '''def draw_rectengles(self):
        height_text = self.ui.lineEdit.text()
        width_text= self.ui.lineEdit_2.text()
        angle_text = self.ui.lineEdit_3.text()
        margin_text = self.ui.lineEdit_4.text()

        # Boş değer kontrolü
        if not width_text or not height_text or not angle_text:
            QMessageBox.warning(self, "Warning", "Please input Height or Width or Angle.", QMessageBox.Ok)
            return
        
        width = int(width_text)
        height = int(height_text)
        angle = int(angle_text)
        margin = int(margin_text)
        image_widht_check = self.ui.graphicsView.sceneRect().width()
        image_height_check = self.ui.graphicsView.sceneRect().height()
        print(image_height_check,image_widht_check)

        # 0 değer kontrolü
        if width <= 0 or height <= 0:
            QMessageBox.warning(self, "Warning", "Height or Width is not valid.", QMessageBox.Ok)
            return

        # Check if width and height are greater than image dimensions
        if width > image_widht_check or height > image_height_check:
            QMessageBox.warning(self, "Warning", "Input Values exceed image size.", QMessageBox.Ok)
            return
        
        center_x = image_widht_check / 2
        center_y = image_height_check / 2

        # Kareyi oluşturun
        square_item = QGraphicsRectItem(-width / 2, -height / 2, width, height)
        square_item.setPos(center_x, center_y)
        square_item.setRotation(angle)
        color_random = QColor(randrange(40), randrange(40), randrange(40))
        square_item.setBrush(QBrush(color_random))  # Kareye renk ekleyin

        # Kareyi sahneye ekleyin
        self.ui.graphicsView.scene().addItem(square_item)

        # Kareyi gelecekte referans için kare öğesine ekleyin
        self.square_item = square_item'''

    def is_float(str):
        try:
            float(str)
            return True
        except ValueError:
            return False
    
    def closeEvent(self, event):
        reply = QMessageBox.question(self,"Onay", "Çıkmak istediğinizden emin misiniz?",QMessageBox.Yes | QMessageBox.No,QMessageBox.No)
        if reply == QMessageBox.Yes:
            event.accept()
            #用过sys.exit(0)和sys.exit(app.exec_())，但没起效果
            self.close_device()
            os._exit(0)
        else:
            event.ignore()

    def drawing_square_text(self):
        if self.square_drawing == True:                   
            # Girilen genişlik ve yükseklik değerlerini al
            self.square_width_text = self.ui.lineEdit.text()
            self.square_height_text = self.ui.lineEdit_2.text()
            # self.square_aci_text = self.ui.lineEdit_3.text()
            # self.square_testere_text = self.ui.lineEdit_4.text()

            if not self.square_width_text or not  self.square_height_text:
                QMessageBox.warning(self,"Uyarı", "Lütfen Boş Değer Girmeyiniz!")
                return 

            self.square_widht = int(self.square_width_text)
            self.square_height = int(self.square_height_text)
            self.square_aci = 0
            self.square_testere = 15
            
            if self.square_widht > self.mermer_alani_width or self.square_height > self.mermer_alani_height:
                QMessageBox.warning(self,"Uyarı","Mermer Alanının Genişlik ve Yükseklik Değerinden Küçük Değer Giriniz")
                return 
            
            if self.square_widht <= 0 or self.square_height <= 0:
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
                

                self.corners = [
                    {'x1': self.last_x, 'y1': self.last_y},
                    {'x2': self.last_x + self.square_widht, 'y2': self.last_y},
                    {'x3': self.last_x + self.square_widht, 'y3': self.last_y + self.square_height},
                    {'x4': self.last_x, 'y4': self.last_y + self.square_height}
                ]
                self.all_rects.append(
                    {"corners":self.corners,
                     "rect_item":square
                     })  # Kare köşe noktalarını ana listeye ekleme 
                self.last_x += self.square_widht + self.square_testere
                self.update_list_widget()
                
            else:
                QMessageBox.warning(self,"Uyarı", "Mermer Alanı Dışına kare eklenemez")

    def update_list_widget(self):
        # QListWidget içeriğini güncelleme
        self.ui.listWidget_2.clear()  # Önceki öğeleri temizleme
        for i, data in enumerate(self.all_rects, start=1):
            corners=data["corners"]
            item = QListWidgetItem(f"Kare {i}: {corners}")
            self.ui.listWidget_2.addItem(item)    
    
    def undo_last_square(self):
        lastSquare=self.all_rects.pop()
        newSquarestartx1=lastSquare["corners"][0]['x1']
        newSquarestarty1=lastSquare["corners"][0]['y1']
        self.last_x=newSquarestartx1
        self.last_y=newSquarestarty1
        self.update_list_widget()
    
        # for item in self.ui.graphicsView.scene().items():
        #     if isinstance(item,QGraphicsRectItem):
        #         if item.rect().x()==newSquarestartx1 and item.rect().y()== newSquarestarty1:
        #             self.ui.graphicsView.scene().removeItem(item)
        rect_item=lastSquare["rect_item"]
        self.ui.graphicsView.scene().removeItem(rect_item)
        
    def send_corners_plc(self):
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

            '''# İkinci kareyi çiz
            square_item = QGraphicsRectItem(self.solust , self.y1 , self.square_widht, self.square_height)
            
            # Kareyi döndür
            self.rotate_square(square_item, self.square_aci)
            
            color_random = QColor(randrange(40), randrange(40), randrange(40))
            square_item.setPen(QPen(Qt.green,2))  # Kareye rastgele renk ekle

            # Kareyi sahneye ekle
            self.ui.graphicsView.scene().addItem(square_item)

            # Kareyi gelecekte referans için kare öğesine ekleyin
            self.all_rects.append(square_item)'''

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
    
 