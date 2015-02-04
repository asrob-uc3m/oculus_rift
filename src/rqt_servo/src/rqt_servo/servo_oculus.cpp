#include "servo_oculus.h"
#include <pluginlib/class_list_macros.h>
#include <QStringList>
#include <ros/master.h>

namespace rqt_servo {

MyPlugin::MyPlugin()
  : rqt_gui_cpp::Plugin()
  , widget_(0)
{
  // Constructor is called first before initPlugin function, needless to say.

  // give QObjects reasonable names
  setObjectName("MyPlugin");
}
/**
  *Inicia el plugin, creando la interfaz, conectando los SIGNAL y SLOT de la la misma, crea el timer,
  * los publishers e inicia el modo automático.
  */
void MyPlugin::initPlugin(qt_gui_cpp::PluginContext& context)
{
  // access standalone command line arguments
  QStringList argv = context.argv();
  // create QWidget
  widget_ = new QWidget();
  // extend the widget with all attributes and children from UI file
  ui_.setupUi(widget_);
  widget_->setFixedHeight(164);
  // add widget to the user interface
  context.addWidget(widget_);
  connect(ui_.CheckManual,SIGNAL(toggled(bool)),SLOT(on_CheckManual_toggled(bool)));
  connect(ui_.PitchSlider,SIGNAL(valueChanged(int)),SLOT(on_PitchSlider_valueChanged(int)));
  connect(ui_.PitchSpin,SIGNAL(valueChanged(int)),SLOT(on_PitchSpin_valueChanged(int)));
  connect(ui_.YawSlider,SIGNAL(valueChanged(int)),SLOT(on_YawSlider_valueChanged(int)));
  connect(ui_.YawSpin,SIGNAL(valueChanged(int)),SLOT(on_YawSpin_valueChanged(int)));

  timer = new QTimer(this);
  connect(timer, SIGNAL(timeout()), this, SLOT(autoUpdate()));

  i=0;
  pub_pitch_oculus = getNodeHandle().advertise<std_msgs::Float64>("/pitch_controller/command", 100);
  pub_yaw_oculus = getNodeHandle().advertise<std_msgs::Float64>("/yaw_controller/command", 100);
  ui_.CheckManual->setChecked(false);
  timer->start(40);

}
/**
  *Cierra el plugin, apagando todos los publishers que se han creado.
  */
void MyPlugin::shutdownPlugin()
{
    // TODO unregister all publishers here
    pub_yaw.shutdown();
    pub_pitch.shutdown();
    pub_yaw_oculus.shutdown();
    pub_pitch_oculus.shutdown();
}

void MyPlugin::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
{
  // TODO save intrinsic configuration, usually using:
  // instance_settings.setValue(k, v)
}

void MyPlugin::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
{
  // TODO restore intrinsic configuration, usually using:
  // v = instance_settings.value(k)
}

/*bool hasConfiguration() const    autoUpdate();
    ui_.
{
  return true;
}

void triggerConfiguration()
{
  // Usually used to open a dialog to offer the user a set of configuration
}*/

} // namespace
//PLUGINLIB_DECLARE_CLASS(rqt_servo_oculus, MyPlugin, rqt_servo_oculus::MyPlugin, rqt_gui_cpp::Plugin)
PLUGINLIB_EXPORT_CLASS(rqt_servo::MyPlugin, rqt_gui_cpp::Plugin)

/**
  *Se llama cuando el timer llega a su valor de cuenta, y sirve para obtener los valores actuales de los
  *sensores de las Oculus, además de para publicarlos gracias a los publishers específicos, así como
  *mostrarlos en la interfaz. Además, cada vez que se ejecuta se comprueba si se ha pulsado el botón
  *Reset, y si se ha hecho se pone la variable i a 0. Haciendo esto, se hace que la referencia se establezca
  *en el valor actual de los servos.
  */
void rqt_servo::MyPlugin::autoUpdate(){
    try{
      listener.lookupTransform("/camera_link", "/tf_oculus",ros::Time(0), t);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

    tf::Matrix3x3(t.getRotation()).getRPY(deg_roll,deg_pitch,deg_yaw);

    static double deg_yaw0;
    if (ui_.ResetButton->isDown())
        i=0;
    if (i<2)
        deg_yaw0=deg_yaw;

    ////////////////////////////////////////////////////////////

    std_msgs::Float64 yaw,pitch;
    pitch.data = -deg_pitch;
    yaw.data = deg_yaw-deg_yaw0;
    ui_.YawSlider->setValue((deg_yaw-deg_yaw0)*57.29577951);
    ui_.PitchSlider->setValue(-deg_pitch*57.29577951);
    pub_yaw_oculus.publish(yaw);
    pub_pitch_oculus.publish(pitch);
    i++;
}
/**
  *Checkbox que activa el modo manual. Cuando está activo, permite el control de los servos
  *con los sliders, creando los publishers de los sliders y eliminando los de las Oculus, además de
  *parando el timer encargado del refresco de los datos, ya que se hará discretamente cada vez que
  *se cambie la posición de los sliders. Cuando no está activo se pasa al modo automático, apagando los
  *publishers de los sliders y creando los de las Oculus, además de iniciando el timer encargado del
  *refresco de datos. La variable i se usa para establecer como origen el punto actual.
  */
void rqt_servo::MyPlugin::on_CheckManual_toggled(bool checked)
{
    ui_.YawSlider->setEnabled(checked);
    ui_.YawSpin->setEnabled(checked);
    ui_.PitchSlider->setEnabled(checked);
    ui_.PitchSpin->setEnabled(checked);
    ui_.ResetButton->setEnabled(!checked);
    if(checked){
        timer->stop();
        pub_yaw=getNodeHandle().advertise<std_msgs::Float64>("/yaw_controller/command", 100);
        pub_pitch=getNodeHandle().advertise<std_msgs::Float64>("/pitch_controller/command", 100);
        pub_yaw_oculus.shutdown();
        pub_pitch_oculus.shutdown();
    }
    else{
        i=0;
        pub_pitch_oculus = getNodeHandle().advertise<std_msgs::Float64>("/pitch_controller/command", 100);
        pub_yaw_oculus = getNodeHandle().advertise<std_msgs::Float64>("/yaw_controller/command", 100);
        pub_yaw.shutdown();
        pub_pitch.shutdown();
        timer->start(40);
    }
}
/**
  *Se ejecuta cuando se desplaza el slider (manual o auto), pasando el valor seleccionado a la casilla Spin.
  *Si está en modo manual, se publica el valor.
  */
void rqt_servo::MyPlugin::on_PitchSlider_valueChanged(int value)
{
    ui_.PitchSpin->setValue(value);
    if(ui_.CheckManual->isChecked()){
        pitch.data=value/57.29577951;
        pub_pitch.publish(pitch);
    }

}
/**
  *Se ejecuta cuando se cambia el valor del Spin, pasando el valor seleccionado al Slider correspondiente.
  *Si está en modo manual, se publica el valor.
  */
void rqt_servo::MyPlugin::on_PitchSpin_valueChanged(int arg1)
{
    ui_.PitchSlider->setValue(arg1);
    if(ui_.CheckManual->isChecked()){
        pitch.data=arg1/57.29577951;
        pub_pitch.publish(pitch);
    }
}
/**
  *Se ejecuta cuando se desplaza el slider (manual o auto), pasando el valor seleccionado a la casilla Spin.
  *Si está en modo manual, se publica el valor.
  */
void rqt_servo::MyPlugin::on_YawSlider_valueChanged(int value)
{
    ui_.YawSpin->setValue(value);
    if(ui_.CheckManual->isChecked()){
        yaw.data=value/57.29577951;
        pub_yaw.publish(yaw);
    }
}
/**
  *Se ejecuta cuando se cambia el valor del Spin, pasando el valor seleccionado al Slider correspondiente.
  *Si está en modo manual, se publica el valor.
  */
void rqt_servo::MyPlugin::on_YawSpin_valueChanged(int arg1)
{
    ui_.YawSlider->setValue(arg1);
    if(ui_.CheckManual->isChecked()){
        yaw.data=arg1/57.29577951;
        pub_yaw.publish(yaw);
    }
}



