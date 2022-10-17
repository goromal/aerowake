import argparse, os
from python_qt_binding import QT_BINDING
from python_qt_binding.QtCore import qDebug, QTimer # ???
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget

from PyQt5.QtGui import *
from PyQt5.QtCore import *

from .rosPubSubs import CommandPubSub

from math import ceil

PWD = os.path.dirname(os.path.abspath(__file__))

keymap = {'ROSflight': {'idx':2,'Ftitle':'Throttle (Unitless)', 'Fmax':0.85,  'Fmin':0.0,  'Fdef':0.0,  'Finc':0.01,  'Funit':'-',
                                'xtitle':'Roll (deg)',          'xmax':30,    'xmin':-30,  'xdef':0.0,  'xinc':1,     'xunit':'deg',
                                'ytitle':'Pitch (deg)',         'ymax':30,    'ymin':-30,  'ydef':0.0,  'yinc':1,     'yunit':'deg',
                                'ztitle':'Yawrate (deg/s)',     'zmax':20,    'zmin':-20,  'xdef':0.0,  'xinc':1,     'zunit':'deg/s'},
           'Aerowake': {'idx':1,'Ftitle':'Altitude (m)',        'Fmax':5.0,   'Fmin':0.0,  'Fdef':0.0,  'Finc':0.05,  'Funit':'m',
                                'xtitle':'Pitch (deg)',         'xmax':30,    'xmin':-30,  'xdef':0.0,  'xinc':1,     'xunit':'deg',
                                'ytitle':'Y-Velocity (m/s)',    'ymax':0.5,   'ymin':-0.5, 'ydef':0.0,  'yinc':0.01,  'yunit':'m/s',
                                'ztitle':'Yaw (deg)',           'zmax':180,   'zmin':-180, 'zdef':0.0,  'zinc':3,     'zunit':'deg'},
       'ROScopterPOS': {'idx':0,'Ftitle':'Altitude (m)',        'Fmax':1.0,   'Fmin':0.0,  'Fdef':0.0,  'Finc':0.01,  'Funit':'m',
                                'xtitle':'X-Position (m)',      'xmax':1.0,   'xmin':-1.0, 'xdef':0.0,  'xinc':0.05,  'xunit':'m',
                                'ytitle':'Y-Position (m)',      'ymax':0.0,   'ymin':-2.0,  'ydef':0.0,  'yinc':0.05,  'yunit':'m',
                                'ztitle':'Yaw (deg)',           'zmax':0.0,   'zmin':-180, 'zdef':-90,  'zinc':3,     'zunit':'deg'},
       'ROScopterVEL': {'idx':3,'Ftitle':'Altitude (m)',        'Fmax':5.0,   'Fmin':0.0,  'Fdef':0.0,  'Finc':0.05,  'Funit':'m',
                                'xtitle':'X-Velocity (m/s)',    'xmax':0.5,   'xmin':-0.5, 'xdef':0.0,  'xinc':0.01,  'xunit':'m/s',
                                'ytitle':'Y-Velocity (m/s)',    'ymax':0.5,   'ymin':-0.5, 'ydef':0.0,  'yinc':0.01,  'yunit':'m/s',
                                'ztitle':'Yawrate (deg/s)',     'zmax':20,    'zmin':-20,  'zdef':0.0,  'zinc':1,     'zunit':'deg/s'}}

class TuningGUI(QWidget):
    def __init__(self):
        super(TuningGUI, self).__init__()
        CommandPubSub.initialize()

        uifname = 'tuning_widget_nobutton.ui'
        self.ui_file = os.path.join(PWD, 'resources', uifname)
        loadUi(self.ui_file, self)
        self.setObjectName('Tuning Widget')
        self.env = ''
        self.F_val = 0.0
        self.x_val = 0.0
        self.y_val = 0.0
        self.z_val = 0.0
        self.load_environment(self.comboBox.currentText())

        self.comboBox.currentIndexChanged[str].connect(self.handleModeChange)

        self.armed = True
        self.toggleArmed() # to set armed to False immediately
        # self.ARMBUTTON.clicked.connect(self.toggleArmed) # DISABLE since we're using RC now

        self.F_slider.valueChanged[int].connect(self.handleFslider)
        self.F_up.clicked.connect(self.handleFup)
        self.F_down.clicked.connect(self.handleFdown)

        self.x_slider.valueChanged[int].connect(self.handlexslider)
        self.x_up.clicked.connect(self.handlexup)
        self.x_down.clicked.connect(self.handlexdown)

        self.y_slider.valueChanged[int].connect(self.handleyslider)
        self.y_up.clicked.connect(self.handleyup)
        self.y_down.clicked.connect(self.handleydown)

        self.z_slider.valueChanged[int].connect(self.handlezslider)
        self.z_up.clicked.connect(self.handlezup)
        self.z_down.clicked.connect(self.handlezdown)

    def toggleArmed(self):
        self.armed = not self.armed
        CommandPubSub.setArmed(self.armed)
        # if self.armed:
        #     self.ARMBUTTON.setStyleSheet("background-color: red")
        #     self.ARMBUTTON.setText('DISARM')
        # else:
        #     self.ARMBUTTON.setStyleSheet("background-color: green")
        #     self.ARMBUTTON.setText('ARM')

    def handleModeChange(self, mode):
        if self.okay_to_change():
            self.load_environment(mode)
        else:
            self.comboBox.setCurrentIndex(keymap[self.env]['idx'])

    def handleFslider(self, value):
        F_max = float(keymap[self.env]['Fmax'])
        F_min = float(keymap[self.env]['Fmin'])
        self.F_val = (F_max - F_min) * value / 1000.0 + F_min
        CommandPubSub.setF(self.F_val)
        self.F_label.setText(str(self.F_val))

    def FincrementInfo(self):
        increment = float(self.F_increment.toPlainText())
        sliderval = int(self.F_slider.value())
        F_max = float(keymap[self.env]['Fmax'])
        F_min = float(keymap[self.env]['Fmin'])
        val_increment = int(ceil(increment / (F_max - F_min) * 1000.0))
        return sliderval, val_increment

    def handleFup(self):
        sliderval, val_increment = self.FincrementInfo()
        self.F_slider.setValue(min(1000, sliderval + val_increment))

    def handleFdown(self):
        sliderval, val_increment = self.FincrementInfo()
        self.F_slider.setValue(max(0, sliderval - val_increment))

    def handlexslider(self, value):
        x_max = float(keymap[self.env]['xmax'])
        x_min = float(keymap[self.env]['xmin'])
        self.x_val = (x_max - x_min) * (value + 500.0) / 1000.0 + x_min
        CommandPubSub.setx(self.x_val)
        self.x_label.setText(str(self.x_val))

    def xincrementInfo(self):
        increment = float(self.x_increment.toPlainText())
        sliderval = int(self.x_slider.value())
        x_max = float(keymap[self.env]['xmax'])
        x_min = float(keymap[self.env]['xmin'])
        val_increment = int(ceil(increment / (x_max - x_min) * 1000.0))
        return sliderval, val_increment

    def handlexup(self):
        sliderval, val_increment = self.xincrementInfo()
        self.x_slider.setValue(min(500, sliderval + val_increment))

    def handlexdown(self):
        sliderval, val_increment = self.xincrementInfo()
        self.x_slider.setValue(max(-500, sliderval - val_increment))

    def handleyslider(self, value):
        y_max = float(keymap[self.env]['ymax'])
        y_min = float(keymap[self.env]['ymin'])
        self.y_val = (y_max - y_min) * (value + 500.0) / 1000.0 + y_min
        CommandPubSub.sety(self.y_val)
        self.y_label.setText(str(self.y_val))

    def yincrementInfo(self):
        increment = float(self.y_increment.toPlainText())
        sliderval = int(self.y_slider.value())
        y_max = float(keymap[self.env]['ymax'])
        y_min = float(keymap[self.env]['ymin'])
        val_increment = int(ceil(increment / (y_max - y_min) * 1000.0))
        return sliderval, val_increment

    def handleyup(self):
        sliderval, val_increment = self.yincrementInfo()
        self.y_slider.setValue(min(500, sliderval + val_increment))

    def handleydown(self):
        sliderval, val_increment = self.yincrementInfo()
        self.y_slider.setValue(max(-500, sliderval - val_increment))

    def handlezslider(self, value):
        z_max = float(keymap[self.env]['zmax'])
        z_min = float(keymap[self.env]['zmin'])
        self.z_val = (z_max - z_min) * (value + 500.0) / 1000.0 + z_min
        CommandPubSub.setz(self.z_val)
        self.z_label.setText(str(self.z_val))

    def zincrementInfo(self):
        increment = float(self.z_increment.toPlainText())
        sliderval = int(self.z_slider.value())
        z_max = float(keymap[self.env]['zmax'])
        z_min = float(keymap[self.env]['zmin'])
        val_increment = int(ceil(increment / (z_max - z_min) * 1000.0))
        return sliderval, val_increment

    def handlezup(self):
        sliderval, val_increment = self.zincrementInfo()
        self.z_slider.setValue(min(500, sliderval + val_increment))

    def handlezdown(self):
        sliderval, val_increment = self.zincrementInfo()
        self.z_slider.setValue(max(-500, sliderval - val_increment))

    def okay_to_change(self):
        return not CommandPubSub.getArmed()

    def load_environment(self, env):
        self.env = env
        CommandPubSub.setMode(self.env)
        F_title   = keymap[self.env]['Ftitle']
        F_max = str(keymap[self.env]['Fmax'])
        F_min = str(keymap[self.env]['Fmin'])
        F_def = keymap[self.env]['Fdef']
        F_inc = str(keymap[self.env]['Finc'])
        F_unit    = keymap[self.env]['Funit']
        x_title   = keymap[self.env]['xtitle']
        x_max = str(keymap[self.env]['xmax'])
        x_min = str(keymap[self.env]['xmin'])
        x_def = keymap[self.env]['xdef']
        x_inc = str(keymap[self.env]['xinc'])
        x_unit    = keymap[self.env]['xunit']
        y_title   = keymap[self.env]['ytitle']
        y_max = str(keymap[self.env]['ymax'])
        y_min = str(keymap[self.env]['ymin'])
        y_def = keymap[self.env]['ydef']
        y_inc = str(keymap[self.env]['yinc'])
        y_unit    = keymap[self.env]['yunit']
        z_title   = keymap[self.env]['ztitle']
        z_max = str(keymap[self.env]['zmax'])
        z_min = str(keymap[self.env]['zmin'])
        z_def = keymap[self.env]['zdef']
        z_inc = str(keymap[self.env]['zinc'])
        z_unit    = keymap[self.env]['zunit']
        self.load_data([F_title, x_title, y_title, z_title],
                       [F_max, x_max, y_max, z_max],
                       [F_min, x_min, y_min, z_min],
                       [F_def, x_def, y_def, z_def],
                       [F_inc, x_inc, y_inc, z_inc],
                       [F_unit, x_unit, y_unit, z_unit])

    def load_data(self, list_title, list_max_label, list_min_label, list_def, list_increment, list_inc_unit):
        self.F_slider.setValue(0.0 + 1000.0*(list_def[0]-float(list_min_label[0]))/(float(list_max_label[0])-float(list_min_label[0]))) #
        self.F_val = list_def[0]
        CommandPubSub.setF(self.F_val)
        self.F_label.setText(str(self.F_val))
        self.x_slider.setValue(-500.0 + 1000.0*(list_def[1]-float(list_min_label[1]))/(1.0*float(list_max_label[1])-float(list_min_label[1]))) #
        self.x_val = list_def[1]
        CommandPubSub.setx(self.x_val)
        self.x_label.setText(str(self.x_val))
        self.y_slider.setValue(-500.0 + 1000.0*(list_def[2]-float(list_min_label[2]))/(1.0*float(list_max_label[2])-float(list_min_label[2]))) #
        self.y_val = list_def[2]
        CommandPubSub.sety(self.y_val)
        self.y_label.setText(str(self.y_val))
        self.z_slider.setValue(-500.0 + 1000.0*(list_def[3]-float(list_min_label[3]))/(1.0*float(list_max_label[3])-float(list_min_label[3]))) #
        self.z_val = list_def[3]
        CommandPubSub.setz(self.z_val)
        self.z_label.setText(str(self.z_val))
        self.F_title.setText(list_title[0])
        self.x_title.setText(list_title[1])
        self.y_title.setText(list_title[2])
        self.z_title.setText(list_title[3])
        self.F_max_label.setText(list_max_label[0])
        self.x_max_label.setText(list_max_label[1])
        self.y_max_label.setText(list_max_label[2])
        self.z_max_label.setText(list_max_label[3])
        self.F_min_label.setText(list_min_label[0])
        self.x_min_label.setText(list_min_label[1])
        self.y_min_label.setText(list_min_label[2])
        self.z_min_label.setText(list_min_label[3])
        self.F_increment.setPlainText(list_increment[0])
        self.x_increment.setPlainText(list_increment[1])
        self.y_increment.setPlainText(list_increment[2])
        self.z_increment.setPlainText(list_increment[3])
        self.F_inc_unit.setText(list_inc_unit[0])
        self.x_inc_unit.setText(list_inc_unit[1])
        self.y_inc_unit.setText(list_inc_unit[2])
        self.z_inc_unit.setText(list_inc_unit[3])

    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        pass
