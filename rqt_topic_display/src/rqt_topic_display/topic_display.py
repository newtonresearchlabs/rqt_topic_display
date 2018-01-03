import os
import rospkg
import rospy

from qt_gui.plugin import Plugin
from python_qt_binding import QtCore
from python_qt_binding import loadUi
from python_qt_binding.QtCore import QTimer, Signal
from python_qt_binding.QtWidgets import QLabel, QWidget
from std_msgs.msg import String


class TopicDisplay(Plugin):
    do_update_label = QtCore.pyqtSignal(String)

    def __init__(self, context):
        super(TopicDisplay, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('TopicDisplay')
        rp = rospkg.RosPack()

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
        # Get path to UI file which is a sibling of this file
        # in this example the .ui and .py file are in the same folder
        ui_file = os.path.join(rp.get_path('rqt_topic_display'), 'resource', 'topic_display.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('TopicDisplayUi')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        # if self.topic_name
        self.topic_name = "string"
        self.update_topic()
        self.label = self._widget.findChild(QLabel, 'label')
        self.do_update_label.connect(self.update_label)


    def handle_callback(self, msg):
        self.do_update_label.emit(msg)

    def update_label(self, msg):
        self.label.setText(msg.data)

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        instance_settings.set_value('topic', self.topic_name)

    def update_topic(self):
        rospy.loginfo("updating topic: " + self.topic_name)
        self.sub = rospy.Subscriber(self.topic_name, String, self.handle_callback,
                                    queue_size=1)


    def restore_settings(self, plugin_settings, instance_settings):
        if instance_settings.contains('topic'):
            self.topic_name = instance_settings.value('topic')
        rospy.loginfo('topic name: ' + self.topic_name)
        self.update_topic()

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog
