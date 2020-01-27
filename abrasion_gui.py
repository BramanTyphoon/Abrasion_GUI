import sys
from PyQt5 import QtCore, QtWidgets
from mainwindow import MainWindow

if __name__=="__main__":
    import sys
    #sys.settrace()
    app = QtWidgets.QApplication(sys.argv)
    ui = MainWindow()
    ui.show()
    sys.exit(app.exec_())
