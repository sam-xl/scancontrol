import os
from argparse import ArgumentParser

import rospkg
import rospy
import rosservice
from rospy.exceptions import ROSException, ROSInterruptException
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QLineEdit, QLineEdit, QPushButton, QMessageBox
from qt_gui.plugin import Plugin

from micro_epsilon_scancontrol_msgs.srv import GetAvailableResolutions, SetResolution, GetResolution, SetFeature, GetFeature
from std_srvs.srv import SetBool

class ScanControlGui(Plugin):

    ID_LASER_POWER      = 0xf0f00824
    ID_EXPOSURE_TIME    = 0xf0f0081c
    ID_IDLE_TIME        = 0xf0f00800
    ID_PROCESSING       = 0xf0f00804
    ID_SCANNER_TYPE     = 0x00000000
    
    def __init__(self, context):
        super(ScanControlGui, self).__init__(context)
        self.setObjectName('ScanControlGui')

        # Parse Arguments
        parser = ArgumentParser()
        parser.add_argument("-q", "--quiet", action="store_true", dest="quiet", help="Put plugin in silent mode")
        self.args, _ = parser.parse_known_args(context.argv())

        # Create QWidget
        self._widget = QWidget()
        ui_file = os.path.join(rospkg.RosPack().get_path('micro_epsilon_scancontrol_driver'), 'resource', 'me_scancontrol_gui.ui')

        # Load ui file in QWidget
        loadUi(ui_file, self._widget)

        # Disable save (for default mode 0)
        self._widget.pushButton_savemode.setEnabled(False)

        # Setup service handles
        try:
            ns = 'scancontrol_driver'
            rospy.wait_for_service(ns + '/set_resolution', timeout=5)
            self.set_resolution = rospy.ServiceProxy(ns + '/set_resolution', SetResolution)

            rospy.wait_for_service(ns + '/get_available_resolutions', timeout=5)
            self.get_available_resolutions = rospy.ServiceProxy(ns + '/get_available_resolutions', GetAvailableResolutions)

            rospy.wait_for_service(ns + '/get_resolution', timeout=5)
            self.get_resolution = rospy.ServiceProxy(ns + '/get_resolution', GetResolution)

            rospy.wait_for_service(ns + '/set_feature', timeout=5)
            self.set_feature = rospy.ServiceProxy(ns + '/set_feature', SetFeature)

            rospy.wait_for_service(ns + '/get_feature', timeout=5)
            self.get_feature = rospy.ServiceProxy(ns + '/get_feature', GetFeature)

            rospy.wait_for_service(ns + '/invert_x', timeout=5)
            self.invert_x = rospy.ServiceProxy(ns + '/invert_x', SetBool)

            rospy.wait_for_service(ns + '/invert_z', timeout=5)
            self.invert_z = rospy.ServiceProxy(ns + '/invert_z', SetBool)

        except (ROSException, ROSInterruptException):
            rospy.logfatal('Failed to find scanCONTROL services. Make sure the scancontrol node is up and running.')
            return
            
        # Query sensor type
        response = self.get_feature(self.ID_SCANNER_TYPE)
        if response.return_code < 0:
            rospy.logfatal('Failed to retrieve available resolutions. Error code: ' + str(response.return_code))
        self.scanner_type = response.value
        print("scanner_type: " + str(self.scanner_type))
        print("return_code: " + str(response.return_code))

        # Query available and current resolution:
        response = self.get_available_resolutions()
        self.resolutions_list = response.resolutions
        if response.return_code < 0:
            rospy.logfatal('Failed to retrieve available resolutions. Error code: ' + str(response.return_code))
        response = self.get_resolution()
        self.resolution = response.resolution
        if response.return_code < 0:
            rospy.logfatal('Failed to retrieve current resolution. Error code: ' + str(response.return_code))
        
        # Query Laser power
        response = self.get_feature(self.ID_LASER_POWER)
        self.laser_power = int(bin(response.value)[-2:],2)
        if response.return_code < 0:
            rospy.logfatal('Failed to retrieve laser power. Error code: ' + str(response.return_code))

        # Query Exposure time
        response = self.get_feature(self.ID_EXPOSURE_TIME)
        self.exposure_time = int(bin(response.value)[-12:],2)
        if response.return_code < 0:
            rospy.logfatal('Failed to retrieve exposure time. Error code: ' + str(response.return_code))

        # Query profile frequency
        response = self.get_feature(self.ID_IDLE_TIME)
        self.idle_time = int(bin(response.value)[-12:],2)
        if response.return_code < 0:
            rospy.logfatal('Failed to retrieve idle time (for profile frequency). Error code: ' + str(response.return_code))
        self.profile_frequency = self.calculate_frequency(self.exposure_time, self.idle_time)

        # Query profile processing
        response = self.get_feature(self.ID_PROCESSING)
        if response.return_code < 0:
            rospy.logwarn('Profile settings disabled. Failed to retrieve profile processing data.')
            self._widget.checkBox_invertz.setEnabled(False)
            self._widget.checkBox_invertx.setEnabled(False)
            invert_z = False
            invert_x = False
        else:
            invert_z = bool(int(bin(response.value)[-7])) # bit 6
            invert_x = bool(int(bin(response.value)[-8])) # bit 7

        # Set GUI fields
        self._widget.comboBox_laserpower.setCurrentIndex(self.laser_power)
        self._widget.comboBox_resolution.addItems([str(i) for i in self.resolutions_list])
        self._widget.comboBox_resolution.setCurrentIndex(self.resolutions_list.index(self.resolution))
        self._widget.lineEdit_exposuretime.setText(str(self.exposure_time_to_ms(self.exposure_time)))
        self._widget.lineEdit_profilefrequency.setText(str(self.profile_frequency))
        self._widget.checkBox_invertz.setChecked(invert_z)
        self._widget.checkBox_invertx.setChecked(invert_x)

        # Connect functions to UI
        self._widget.comboBox_resolution.currentIndexChanged.connect(self.select_resolution)
        self._widget.comboBox_laserpower.currentIndexChanged.connect(self.select_laser_power)
        self._widget.lineEdit_exposuretime.returnPressed.connect(self.select_exposure_time)
        self._widget.lineEdit_profilefrequency.returnPressed.connect(self.select_profile_frequency)
        self._widget.checkBox_invertz.stateChanged.connect(self.select_invert_z)
        self._widget.checkBox_invertx.stateChanged.connect(self.select_invert_x)
        self._widget.comboBox_mode.currentIndexChanged.connect(self.select_mode)
        self._widget.pushButton_savemode.clicked.connect(self.load_usermode)
        self._widget.pushButton_loadmode.clicked.connect(self.save_usermode)

        # Add widget to user interface
        context.add_widget(self._widget)

    def shutdown_plugin(self):
        pass

    def save_settings(self, pluggin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        pass

    def select_resolution(self):
        # Retrieve requested resolution
        resolution = self._widget.comboBox_resolution.currentText()
        rospy.loginfo('Set resolution to: ' + resolution)
        
        # Set the requested resolution
        return_code = self.set_resolution(int(resolution))
        if return_code < 0:
            rospy.logwarn('Failed to set resolution. Error code: '+ str(resolution))
            # Return to old index
            index = self._widget.comboBox_resolution.findText(str(self.resolution))
            self._widget.comboBox_resolution.setCurrentIndex(index)
        else:
            # Store new resolution
            self.resolution = int(resolution)
    
    def select_laser_power(self):
        # Retrieve requested laser power
        index = self._widget.comboBox_laserpower.currentIndex()
        rospy.loginfo('Set laserpower to: ' + self._widget.comboBox_laserpower.currentText())

        # Set requested laser power
        response = self.set_feature(self.ID_LASER_POWER, index)
        if response.return_code < 0:
            rospy.logwarn('Failed to set current laser power. Error code: ' + str(response.return_code))
            # Return to old index
            index = self._widget.comboBox_laserpower.findText(self.laser_power)
            self._widget.comboBox_laserpower.setCurrentIndex(index)
        else:
            # Store new index
            self.laser_power = index

    def select_exposure_time(self):
        # Retrieve requested exposure time
        exposure_time = float(self._widget.lineEdit_exposuretime.text())*1e2
        exposure_time = int(round(exposure_time))
        rospy.loginfo('Set exposure time to (Rounded to nearest 10us): ' + str(self.exposure_time_to_ms(exposure_time)) + ' ms.')

        # Set requested exposure time: 
        response = self.set_feature(self.ID_EXPOSURE_TIME, exposure_time)
        if response.return_code < 0:
            rospy.logwarn('Failed to set exposure time. Error code: ' + str(response.return_code))
            # Return to old value
            self._widget.lineEdit_exposuretime.setText(str(self.exposure_time_to_ms(self.exposure_time)))
        else:
            self.exposure_time = exposure_time
            self.select_profile_frequency()
    
    def select_profile_frequency(self):
        # Retrieve requested profile frequency and calculate idle_time and corrected frequency
        frequency = float(self._widget.lineEdit_profilefrequency.text())
        idle_time = self.calculate_idle_time(self.exposure_time, frequency)
        frequency = self.calculate_frequency(self.exposure_time, idle_time)
        rospy.loginfo('Set idle time to: ' + str(idle_time) + ', to achieve frequency of ' + str(frequency) + ' Hz.')
        
        # Set requested idle time to achieve requested frequency
        response = self.set_feature(self.ID_IDLE_TIME, idle_time)
        if response.return_code < 0:
            rospy.logwarn('Failed to set idle time. Error code: ' + str(response.return_code))
        else:
            self.profile_frequency = frequency
        self._widget.lineEdit_profilefrequency.setText(str(self.profile_frequency))

    def select_invert_z(self):
        # Retrieve status of invert z checkbox
        invert_coordinate = self._widget.checkBox_invertz.isChecked()

        # Send service request
        response = self.invert_z(invert_coordinate)
        if not response.succes:
            rospy.logwarn("Failed to set 'Invert Z'. Error code: " + response.message)
            self._widget.checkBox_invertz.setChecked(~invert_coordinate)

    def select_invert_x(self):
        # Retrieve status of invert z checkbox
        invert_coordinate = self._widget.checkBox_invertz.isChecked()

        # Send service request
        response = self.invert_x(invert_coordinate)
        if not response.succes:
            rospy.logwarn("Failed to set 'Invert X'. Error code: " + response.message)
            self._widget.checkBox_invertz.setChecked(~invert_coordinate)

    def select_mode(self):
        # Retrieve selected mode
        index = self._widget.comboBox_mode.currentIndex()

        # Enable/Disable save button for mode 0 (default settings)
        if index == 0:
            self._widget.pushButton_savemode.setEnabled(False)
        else:
            self._widget.pushButton_savemode.setEnabled(True)
        rospy.logdebug('Selected mode: ' + str(index))
    
    def load_usermode(self):
        # Retrieve selected mode
        index = self._widget.comboBox_mode.currentIndex()

        rospy.logwarn('load_usermode: NotImplemented')

    def save_usermode(self):
        # Retrieve selected mode
        index = self._widget.comboBox_mode.currentIndex()

        rospy.logwarn('save_usermode: NotImplemented')

    def calculate_frequency(self, exposure_time, idle_time):
        return 1./((exposure_time + idle_time)*10e-6)

    def calculate_idle_time(self, exposure_time, frequency):
        return int(round(1./(frequency*1e-5) - exposure_time))
    
    def exposure_time_to_ms(self, exposure_time):
        return exposure_time*1e-2

class ScanControlManualSettingsGui(Plugin):
    feature2hex =  {"FEATURE_FUNCTION_SERIAL": 0xf0000410, "FEATURE_FUNCTION_PEAKFILTER_WIDTH": 0xf0b02000,
                    "FEATURE_FUNCTION_PEAKFILTER_HEIGHT": 0xf0b02004, "FEATURE_FUNCTION_FREE_MEASURINGFIELD_Z": 0xf0b02008,
                    "FEATURE_FUNCTION_FREE_MEASURINGFIELD_X": 0xf0b0200c, "FEATURE_FUNCTION_DYNAMIC_TRACK_DIVISOR": 0xf0b02010,
                    "FEATURE_FUNCTION_DYNAMIC_TRACK_FACTOR": 0xf0b02014, "FEATURE_FUNCTION_CALIBRATION_0": 0xf0b02020,
                    "FEATURE_FUNCTION_CALIBRATION_1": 0xf0b02024, "FEATURE_FUNCTION_CALIBRATION_2": 0xf0b02028,
                    "FEATURE_FUNCTION_CALIBRATION_3": 0xf0b0202c, "FEATURE_FUNCTION_CALIBRATION_4": 0xf0b02030,
                    "FEATURE_FUNCTION_CALIBRATION_5": 0xf0b02034, "FEATURE_FUNCTION_CALIBRATION_6": 0xf0b02038,
                    "FEATURE_FUNCTION_CALIBRATION_7": 0xf0b0203c, "FEATURE_FUNCTION_LASERPOWER": 0xf0f00824,
                    "FEATURE_FUNCTION_MEASURINGFIELD": 0xf0f00880, "FEATURE_FUNCTION_TRIGGER": 0xf0f00830,
                    "FEATURE_FUNCTION_SHUTTERTIME": 0xf0f0081c, "FEATURE_FUNCTION_IDLETIME": 0xf0f00800,
                    "FEATURE_FUNCTION_PROCESSING_PROFILEDATA": 0xf0f00804, "FEATURE_FUNCTION_THRESHOLD": 0xf0f00810,
                    "FEATURE_FUNCTION_MAINTENANCEFUNCTIONS": 0xf0f0088c, "FEATURE_FUNCTION_REARRANGEMENT_PROFILE": 0xf0f0080c,
                    "FEATURE_FUNCTION_PROFILE_FILTER": 0xf0f00818, "FEATURE_FUNCTION_RS422_INTERFACE_FUNCTION": 0xf0f008c0,
                    "FEATURE_FUNCTION_PACKET_DELAY": 0x00000d08, "FEATURE_FUNCTION_SHARPNESS": 0xf0f00808,
                    "FEATURE_FUNCTION_TEMPERATURE": 0xf0f0082c, "FEATURE_FUNCTION_SATURATION": 0xf0f00814,
                    "FEATURE_FUNCTION_CAPTURE_QUALITY": 0xf0f008c4}

    def __init__(self, context):
        super(ScanControlManualSettingsGui, self).__init__(context)
        self.setObjectName('ScanControlSettings')

        # Parse Arguments
        parser = ArgumentParser()
        parser.add_argument("-q", "--quiet", action="store_true", dest="quiet", help="Put plugin in silent mode")
        self.args, _ = parser.parse_known_args(context.argv())

        # Create QWidget
        self._widget = QWidget()
        ui_file = os.path.join(rospkg.RosPack().get_path('micro_epsilon_scancontrol_driver'), 'resource', 'me_scancontrol_gui_manual_settings.ui')

        # Load ui file in QWidget
        loadUi(ui_file, self._widget)
        self._widget.pushButton_confirm.clicked.connect(self.on_button_clicked)

        # Retrieve service
        rospy.wait_for_service('scancontrol_set_feature', timeout = 5)
        self.set_feature = rospy.ServiceProxy('scancontrol_set_feature', SetFeature)

        if len(self.serials) > 0:
            self._widget.comboBox_serial.addItems(self.serials)
        else:
            modalWindow = QMessageBox() 
            modalWindow.setIcon(QMessageBox.Information)
            modalWindow.setText('No active ScanControlSettingsService service found. First start a scanCONTROL node and reload this rqt-plugin.')
            modalWindow.exec_()

        # Add widget to user interface
        context.add_widget(self._widget)

    def shutdown_plugin(self):
        pass

    def save_settings(self, pluggin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        pass

    def on_button_clicked(self):
        # Disable button to prevent repeated sends
        self._widget.pushButton_confirm.setEnabled(False)
        self._widget.repaint()

        # Read lineEdit fields
        setting = self._widget.lineEdit_setting.text()
        try:
            setting = int(setting, 16)
        except ValueError:
            setting = self.feature2hex[setting]
        value = self._widget.lineEdit_value.text()
        serial = str(self._widget.comboBox_serial.currentText())

        # Print if to console if not quiet
        if not self.args.quiet: 
            print('Sending request to change setting:')
            print('Setting field: ' + setting)
            print('Value field: ' + value)
            if serial:
                print('Serial: ' + serial)

        # Send settings over the SetFeature service
        return_code = self.set_feature(setting, value)

        # Comunicate service outcome
        modalWindow = QMessageBox() 
        modalWindow.setIcon(QMessageBox.Information)
        if return_code >= 1:
            modalWindow.setText('Setting change succesfull.')
        else:
            modalWindow.setText('Setting change failed.\n\nError code:\n' + str(return_code))
        modalWindow.exec_()

        # Re-enable push button 
        self._widget.pushButton_confirm.setEnabled(True)




        
    