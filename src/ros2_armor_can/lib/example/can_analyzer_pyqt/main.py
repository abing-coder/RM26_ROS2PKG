import sys
from PyQt5.QtWidgets import QApplication
from widget import Widget

if __name__ == '__main__':
    app = QApplication(sys.argv)
    w = Widget()

    w.show()
    sys.exit(app.exec_())