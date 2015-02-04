/**
  *@file servo_oculus.h
  *@brief Oculus Rift plugin for rqt_gui
  *
  *Read de values of the Oculus' sensors, display it in rqt_gui and publish it in ROS.
  *
  *@author Enrique Ruiz-Medrano García
  *@date 11/14
  */

#ifndef rqt_servo_oculus__servo_oculus_H
#define rqt_servo_oculus__servo_oculus_H

#include <tf/transform_listener.h>
#include <rqt_gui_cpp/plugin.h>
#include "ui_servo_oculus.h"
#include <QWidget>
#include <ros/ros.h>
#include <ros/time.h>
#include <std_msgs/Float64.h>
#include <QTimer>

namespace rqt_servo {

class MyPlugin
  : public rqt_gui_cpp::Plugin
{
  Q_OBJECT
public:
    /**
      * @brief Constructor
      */
  MyPlugin();

  /**
   * @brief Inicia el plugin
   */
  virtual void initPlugin(qt_gui_cpp::PluginContext& context);
  /**
   * @brief Cierra el plugin
   */
  virtual void shutdownPlugin();
  /**
   * @brief save intrinsic configuration, usually using:
   * instance_settings.setValue(k, v)
   */
  virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;
  /**
   * @brief save intrinsic configuration, usually using:
   * v = instance_settings.value(k)
   */
  virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);


  // Comment in to signal that the plugin has a way to configure it
  //bool hasConfiguration() const;
  //void triggerConfiguration();
private slots:  
  /**
   * @brief Checkbox del modo manual
   * @param checked estado del checkbox
   */
  void on_CheckManual_toggled(bool checked);
  /**
   * @brief Cambio del valor del slider
   * @param value valor actual
   */
  void on_PitchSlider_valueChanged(int value);
  /**
   * @brief Cambio del valor del spin
   * @param arg1 valor actual
   */
  void on_PitchSpin_valueChanged(int arg1);
  /**
   * @brief Cambio del valor del slider
   * @param value valor actual
   */
  void on_YawSlider_valueChanged(int value);
  /**
   * @brief Cambio del valor del spin
   * @param arg1 valor actual
   */
  void on_YawSpin_valueChanged(int arg1);
  /**
   * @brief función que se ejecuta con el timeout del timer
   */
  void autoUpdate();



private:
  Ui::Servo_Oculus ui_;
  QWidget* widget_;
  ros::Publisher pub_yaw,pub_pitch,pub_yaw_oculus,pub_pitch_oculus;
  std_msgs::Float64 pitch,yaw;
  tf::TransformListener listener;
  tf::StampedTransform t;
  double deg_pitch,deg_roll,deg_yaw;
  int i;
  QTimer *timer;

};
} // namespace
#endif // my_namespace__my_plugin_H
