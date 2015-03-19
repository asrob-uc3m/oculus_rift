#!/usr/bin/env python
# -*- coding: utf-8 -*-

__author__ = "Enrique Ruiz-Medrano Garcia"
__date__ = "2014-11"
__author__ = "Raul Perula-Martinez"
__date__ = "2015-01"
__version__ = "$ Revision: 1.0 $"

import os
import math
import rospy
import rospkg
import tf

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi, QtCore
from python_qt_binding.QtGui import QWidget, QPixmap

from std_msgs.msg import Float32

class MyPlugin(Plugin):

    """
    Class to show a GUI. Could be a form.
    """

    def __init__(self, context):
        """
        Constructor of the class.
        """

        super(MyPlugin, self).__init__(context)

        # give QObjects reasonable names
        self.setObjectName('TelepresencePlugin')

        # create QWidget
        self._widget = QWidget()

        # get path to UI file which should be in the "resource" folder of this
        # package
        ui_file = os.path.join(
            rospkg.RosPack().get_path('rqt_servo'), 'resource', 'view.ui')

        # extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)

        # give QObjects reasonable names
        self._widget.setObjectName('TelepresencePlugin')
        
        # set fixed height
        self._widget.setFixedHeight(164)

        # add widget to the user interface
        context.add_widget(self._widget)
        
        # create a timer
        self.timer = QtCore.QTimer()
        self.timer.start(40)
        
        # class variables
        self.i = 0
        
        # publishers and subscribers
        self.pub_pitch_oculus = rospy.Publisher('pitch_controller/command',
                                                Float32, queue_size=100)
        self.pub_yaw_oculus = rospy.Publisher('yaw_controller/command',
                                              Float32, queue_size=100)

        # events
        QtCore.QObject.connect(self._widget.CheckManual,
                               QtCore.SIGNAL('toggled(bool)'),
                               self.on_CheckManual_toggled)

        QtCore.QObject.connect(self._widget.PitchSlider,
                               QtCore.SIGNAL('valueChanged(int)'),
                               self.on_PitchSlider_valueChanged)
        QtCore.QObject.connect(self._widget.PitchSpin,
                               QtCore.SIGNAL('valueChanged(int)'),
                               self.on_PitchSpin_valueChanged)

        QtCore.QObject.connect(self._widget.YawSlider,
                               QtCore.SIGNAL('valueChanged(int)'),
                               self.on_YawSlider_valueChanged)
        QtCore.QObject.connect(self._widget.YawSpin,
                               QtCore.SIGNAL('valueChanged(int)'),
                               self.on_YawSpin_valueChanged)
        
        QtCore.QObject.connect(self.timer,
                               QtCore.SIGNAL('timeout()'),
                               self.auto_update)

        # set manual box unnactive
        self._widget.CheckManual.setChecked(False)

    def shutdown_plugin(self):
        # unregister all
        self.pub_pitch_oculus.unregister()
        self.pub_yaw_oculus.unregister()

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    # def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog

    def auto_update(self):
        """
        Se llama cuando el timer llega a su valor de cuenta, y sirve para obtener los valores
        actuales de los sensores de las Oculus, además de para publicarlos gracias a los publishers
        específicos, así como mostrarlos en la interfaz. Además, cada vez que se ejecuta se comprueba
        si se ha pulsado el botón Reset, y si se ha hecho se pone la variable i a 0. Haciendo esto,
        se hace que la referencia se establezca en el valor actual de los servos.
        """
        
        try:
            # get the transform
            listener = tf.TransformListener()
            
            (trans, rot) = listener.lookupTransform("/camera_link", "/tf_oculus", rospy.Time(0))
            
            # convert the quaternion to Euler angles
            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(rot)

            deg_pitch = (180.0 / math.pi) * pitch
            deg_yaw = (180.0 / math.pi) * yaw
            
            # if the reset buttom is pressed
            if self._widget.ResetButton.isDown():
                self.i = 0

            deg_yaw0 = 0
            if self.i < 2:
                deg_yaw0 = deg_yaw

            # fill the messages
            msg_pitch = Float32()
            msg_yaw = Float32()

            msg_pitch.data = -deg_pitch
            msg_yaw.data = deg_yaw - deg_yaw0

            self._widget.YawSlider.setValue((deg_yaw - deg_yaw0) * 57.29577951)
            self._widget.PitchSlider.setValue(-deg_pitch * 57.29577951)

            # publish the values
            self.pub_yaw_oculus.publish(yaw)
            self.pub_pitch_oculus.publish(pitch)

            # increase i
            self.i += 1
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

    def on_CheckManual_toggled(self, checked):
        """
        Checkbox que activa el modo manual. Cuando está activo, permite el control de los servos
        con los sliders, creando los publishers de los sliders y eliminando los de las Oculus,
        además de parando el timer encargado del refresco de los datos, ya que se hará
        discretamente cada vez que se cambie la posición de los sliders. Cuando no está activo
        se pasa al modo automático, apagando los publishers de los sliders y creando los de las
        Oculus, además de iniciando el timer encargado del refresco de datos. La variable i se
        usa para establecer como origen el punto actual.
        
        @param checked: estado del checkbox
        """

        self._widget.YawSlider.setEnabled(checked)
        self._widget.YawSpin.setEnabled(checked)
        self._widget.PitchSlider.setEnabled(checked)
        self._widget.PitchSpin.setEnabled(checked)
        self._widget.ResetButton.setEnabled(not checked)

        if checked:
            self.timer.stop()
        else:
            self.i = 0

            self.timer.start(40)
        
    def on_PitchSlider_valueChanged(self, value):
        """
        Se ejecuta cuando se desplaza el slider (manual o auto), pasando el valor seleccionado
        a la casilla Spin. Si está en modo manual, se publica el valor.
        
        @param value: valor actual
        """
        
        self._widget.PitchSpin.setValue(value)
        
        msg = Float32()

        if self._widget.CheckManual.isChecked():
            msg.data = value / 57.29577951

            self.pub_pitch_oculus.publish(msg)
    
    def on_PitchSpin_valueChanged(self, value):
        """
        Se ejecuta cuando se cambia el valor del Spin, pasando el valor seleccionado al Slider
        correspondiente. Si está en modo manual, se publica el valor.
        
        @param arg1: valor actual
        """
        
        self._widget.PitchSlider.setValue(value)
        
        msg = Float32()

        if self._widget.CheckManual.isChecked():
            msg.data = value / 57.29577951

            self.pub_pitch_oculus.publish(msg)
        
    def on_YawSpin_valueChanged(self, value):
        """
        Se ejecuta cuando se cambia el valor del Spin, pasando el valor seleccionado al Slider
        correspondiente. Si está en modo manual, se publica el valor.
        
        @param arg1: valor actual
        """
        
        self._widget.YawSlider.setValue(value)
        
        msg = Float32()

        if self._widget.CheckManual.isChecked():
            msg.data = value / 57.29577951
            
            self.pub_yaw_oculus.publish(msg)
        
    def on_YawSlider_valueChanged(self, value):
        """
        Se ejecuta cuando se desplaza el slider (manual o auto), pasando el valor seleccionado
        a la casilla Spin. Si está en modo manual, se publica el valor.
        
        @param value: valor actual
        """
        
        self._widget.YawSpin.setValue(value)
        
        msg = Float32()

        if self._widget.CheckManual.isChecked():
             msg.data = value / 57.29577951

             self.pub_yaw_oculus.publish(msg)
