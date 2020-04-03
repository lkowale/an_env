import os
import rospy
import rospkg
from std_msgs.msg import String

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget


class MRPlugin(Plugin):

    def __init__(self, context):
        super(MRPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('MRPlugin')

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
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_marker_recorder'), 'resource', 'MRPlugin.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('MRPluginUi')
        # multiple windows numbered
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # widgets init
        self._widget.btn_start.clicked.connect(self._clicked_btn_start)
        self._widget.btn_point_set.clicked.connect(self._clicked_btn_point_set)
        # Add widget to the user interface
        context.add_widget(self._widget)
        # ROS
        self.subscribers = [rospy.Subscriber('marker_recorder/marker_new_point', String, self._new_point),
                            rospy.Subscriber('marker_recorder/marker_readings', String, self._readings)]

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    def _clicked_btn_start(self):
        pub = rospy.Publisher("marker_recorder/btn_start", String, queue_size=10)
        pub.publish('clicked_btn_start')

    def _clicked_btn_point_set(self):
        pub = rospy.Publisher("marker_recorder/btn_point_set", String, queue_size=10)
        pub.publish('clicked_btn_point_set')

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog

    def _new_point(self, msg):
        self._widget.l_new_point.setText(msg.data)

    def _readings(self, msg):
        self._widget.l_readings.setText(msg.data)