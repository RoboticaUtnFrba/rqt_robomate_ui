import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, Slot
from python_qt_binding.QtWidgets import QWidget
from std_msgs.msg import Float32

NUM_DESKS = 9
DESKS = [False] * NUM_DESKS

class MyPlugin(Plugin):

    def __init__(self, context):
        super(MyPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('MyPlugin')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_robomate_ui'), 'resource', 'widget.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('MyPluginUi')
        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        # Callback to actions
        self._widget.pushButton_1.clicked.connect(self.add_button_clicked)
        self._widget.pushButton_2.clicked.connect(self.remove_button_clicked)
        self._widget.spinBox.valueChanged.connect(self.spinBox_valueChanged)
        self._widget.spinBox.setRange(1, 9)

        # Add widget to the user interface
        context.add_widget(self._widget)
        # Create topic for node comunication
        self.pub = rospy.Publisher("/robomate/ui", Float32, queue_size=10)


    @Slot()
    def add_button_clicked(self):
        arg = self._widget.spinBox.value()
        print("User registered in the list", self._widget.spinBox.value())
        self.pub.publish(self._widget.spinBox.value())
        self._widget.pushButton_1.setEnabled(0)
        self._widget.pushButton_2.setEnabled(1)
        DESKS[arg-1] = True

    def remove_button_clicked(self):
        arg = self._widget.spinBox.value()
        msg = - self._widget.spinBox.value()
        self.pub.publish(msg)
        print("User deleted from the list", msg)
        self._widget.pushButton_1.setEnabled(1)
        self._widget.pushButton_2.setEnabled(0)
        DESKS[arg-1] = False

    def spinBox_valueChanged(self):

        arg = self._widget.spinBox.value()

        if DESKS[arg-1]:
            self._widget.pushButton_1.setEnabled(0)
            self._widget.pushButton_2.setEnabled(1)
        else:
            self._widget.pushButton_1.setEnabled(1)
            self._widget.pushButton_2.setEnabled(0)

    def shutdown_plugin(self):
        self.pub.unregister()

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass
