import os
import roslib.message
import rospkg
import rospy

from qt_gui.plugin import Plugin
from python_qt_binding import QtCore
from python_qt_binding import loadUi
from python_qt_binding.QtCore import QTimer
from python_qt_binding.QtWidgets import QLabel, QWidget
# from std_msgs.msg import Float32, Float64, Int16, Int32, Int64, Int8, UInt16, UInt32, UInt64, UInt8


class NumberDisplay(Plugin):
    do_update_label = QtCore.pyqtSignal(float)

    def __init__(self, context):
        super(NumberDisplay, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('NumberDisplay')
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
            print('arguments: ', args)
            print('unknowns: ', unknowns)

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which is a sibling of this file
        # in this example the .ui and .py file are in the same folder
        ui_file = os.path.join(rp.get_path('rqt_topic_display'), 'resource', 'number_display.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('NumberDisplayUi')
        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        self.supported_types = ['marti_common_msgs/BoolStamped',
                                'marti_common_msgs/Float32Stamped',
                                'marti_common_msgs/Float64Stamped',
                                'marti_common_msgs/Int16Stamped',
                                'marti_common_msgs/Int32Stamped',
                                'marti_common_msgs/Int64Stamped',
                                'marti_common_msgs/Int8Stamped',
                                'marti_common_msgs/UInt16Stamped',
                                'marti_common_msgs/UInt32Stamped',
                                'marti_common_msgs/UInt64Stamped',
                                'marti_common_msgs/UInt8Stamped',
                                'std_msgs/Int16',
                                'std_msgs/Int32',
                                'std_msgs/Int64',
                                'std_msgs/Int8',
                                'std_msgs/UInt8',
                                'std_msgs/UInt16',
                                'std_msgs/UInt32',
                                'std_msgs/UInt64',
                                'std_msgs/Float32',
                                'std_msgs/Float64',
                                ]

        # if self.topic_name
        self.label_text = ""
        self.topic_name = "number"
        self.precision = 3
        # TODO(lucasw) ros param
        self.sub = None
        self.update_topic()
        self.label = self._widget.findChild(QLabel, 'label')
        self.do_update_label.connect(self.update_label)

        self.timer = QTimer()
        self.timer.start(200)
        self.timer.timeout.connect(self.qt_update)

    def qt_update(self):
        pass

    def handle_callback(self, msg):
        # TODO(lucasw) throttle this if too frequent
        # or use qt_update above instead of emit
        value = 0.0
        if hasattr(msg, 'data'):
            value = msg.data
        elif hasattr(msg, 'value'):
            value = msg.value
        else:
            rospy.logwarn_once(f"no date or value in msg {msg}")
            return
        self.do_update_label.emit(value)

    def update_label(self, data):
        text = "{:.{}f}".format(data, self.precision)
        if self.label_text and self.label_text != "":
            text = self.label_text + ": " + text
        # TODO(lucasw)
        # If the refresh button is pressed this will throw
        # maybe because the emit is triggered after the plugin is torn down?
        try:
            self.label.setText(text)
        except Exception as ex:
            rospy.logdebug(ex)
            pass

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        instance_settings.set_value('label', self.label_text)
        instance_settings.set_value('topic', self.topic_name)
        instance_settings.set_value('precision', self.precision)

    def init_callback(self, msg):
        topic_type = msg._connection_header['type']
        self.sub.unregister()

        topic_class = roslib.message.get_message_class(topic_type)
        if topic_type in self.supported_types:
            rospy.loginfo(topic_type + " " + str(topic_class))
            self.sub = rospy.Subscriber(self.topic_name, topic_class,
                                        self.handle_callback,
                                        queue_size=2)
        else:
            rospy.logwarn(f"unsupported type {topic_class} {topic_type}")

    def update_topic(self):
        # rospy.loginfo("updating topic: " + self.topic_name)
        if self.sub:
            self.sub.unregister()
        self.sub = rospy.Subscriber(self.topic_name, rospy.AnyMsg, self.init_callback,
                                    queue_size=1)

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO(lucasw) make rosparam override saved settings
        if instance_settings.contains('label'):
            self.label_text = instance_settings.value('label')
        if instance_settings.contains('topic'):
            self.topic_name = instance_settings.value('topic')
        if instance_settings.contains('precision'):
            self.precision = instance_settings.value('precision')
        rospy.loginfo('topic name: ' + self.topic_name)
        self.update_topic()

    # def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog
