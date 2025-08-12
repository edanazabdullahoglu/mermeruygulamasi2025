# error_message_box.py file in this here

from PySide2.QtWidgets import QMessageBox, QPushButton
from PySide2.QtCore import QTimer

class ErrorMessageBox:
    def __init__(self, parent=None):
        self.parent = parent

    def show_error(self, error_message, error_icon, title="Error Message", ):
        msg_box = QMessageBox()
        msg_box.setIcon(error_icon)
        msg_box.setWindowTitle(title)
        msg_box.setText(error_message)
        # Kapatma düğmesi ekle
        okey_button = QPushButton("OKEY")
        msg_box.addButton(okey_button, QMessageBox.ButtonRole.AcceptRole)

        # Kapatma düğmesine tıklandığında pencereyi kapat
        okey_button.clicked.connect(msg_box.close)
        msg_box.exec_()

    def show_information(self, iformation_message, iformation_icon, title="iformation Message", ):
        msg_info_box = QMessageBox()
        msg_info_box.setIcon(iformation_icon)
        msg_info_box.setWindowTitle(title)
        msg_info_box.setText(iformation_message)
        # msg_info_box.standardButton(QMessageBox.NoButton)
        QTimer.singleShot(1000,msg_info_box.close)
        msg_info_box.exec_()

    def show_error_info(self, iformation_message, iformation_icon, title="iformation Message", ):
        msg_info_box1 = QMessageBox()
        msg_info_box1.setIcon(iformation_icon)
        msg_info_box1.setWindowTitle(title)
        msg_info_box1.setText(iformation_message)
        # msg_info_box1.standardButton(QMessageBox.NoButton)
        QTimer.singleShot(1000,msg_info_box1.close)
        msg_info_box1.exec_()

