import os
import rospkg
import rospy

from qt_gui.plugin import Plugin
from python_qt_binding import QtCore
from python_qt_binding import loadUi
from python_qt_binding.QtCore import QTimer, Signal
from python_qt_binding.QtWidgets import QLabel, QScrollArea, QWidget
from rqt_topic_display.srv import *
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

        self.last_max_scroll = 0
        self.accumulated = []
        self.accumulate_req = AccumulateRequest()
        # Temp debug
        # self.accumulate_req.accumulate = True

        # if self.topic_name
        self.topic_name = "string"
        self.accumulate_service_name = "~accumulate"
        # TODO(lucasw) ros param
        self.sub = None
        self.accumulate_service = None
        self.update_topic()
        self.label = self._widget.findChild(QLabel, 'label')
        self.scroll_area = self._widget.findChild(QScrollArea, 'scroll_area')
        self.do_update_label.connect(self.update_label)

        self.timer = QTimer()
        self.timer.start(200)
        self.timer.timeout.connect(self.qt_update)

    def qt_update(self):
        # TODO(lucasw) after setText the scroll bar takes a moment to respond,
        # so can't simply go to maximum after calling it
        # TODO(lucasw) may not want to do this if user has been moving scroll bar
        # around- need to detect that scroll bar was at bottom and only
        # autoscroll then.
        if self.accumulate_req.accumulate:
            # self.dirty = False
            cur_scroll = self.scroll_area.verticalScrollBar().value()
            max_scroll = self.scroll_area.verticalScrollBar().maximum()
            if max_scroll > self.last_max_scroll and cur_scroll >= self.last_max_scroll:
                self.scroll_area.verticalScrollBar().setValue(max_scroll)
            self.last_max_scroll = max_scroll

    def handle_accumulate(self, req):
        self.accumulate_req = req
        if not self.accumulate_req.accumulate:
            self.accumulated = []
        return AccumulateResponse()

    def handle_callback(self, msg):
        string = None
        if self.accumulate_req.accumulate:
            string = String()
            self.accumulated.append(msg.data)
            for i in range(len(self.accumulated)):
                string.data += self.accumulated[i] + "\n"
            # remove final line ending
            string.data = string.data[:-1]
        else:
            string = msg
        self.do_update_label.emit(string)

    def update_label(self, msg):
        # If the refresh button is pressed this will throw
        # maybe because the emit is triggered after the plugin is torn down?
        try:
            self.label.setText(msg.data)
        except:
            pass

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        instance_settings.set_value('topic', self.topic_name)
        instance_settings.set_value('accumulate', self.accumulate_service_name)

    def update_topic(self):
        # rospy.loginfo("updating topic: " + self.topic_name)
        if self.sub:
            self.sub.unregister()
        self.sub = rospy.Subscriber(self.topic_name, String, self.handle_callback,
                                    queue_size=1)

        if self.accumulate_service:
            self.accumulate_service.shutdown('')
        try:
            self.accumulate_service = rospy.Service(self.accumulate_service_name,
                                                    Accumulate,
                                                    self.handle_accumulate)
        except rospy.ServiceException, ex:
            rospy.logwarn("Can't run a rqt_topic_display accumulate service " +
                          "with name already used: " +
                          str(ex) + ", " + self.accumulate_service_name)

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO(lucasw) make rosparam override saved settings
        if instance_settings.contains('topic'):
            self.topic_name = instance_settings.value('topic')
        rospy.loginfo('topic name: ' + self.topic_name)
        if instance_settings.contains('accumulate'):
            self.accumulate_service_name = instance_settings.value('accumulate')
        self.update_topic()

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog
