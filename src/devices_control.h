/*
 * Copyright 2014 Instituto de Sistemas e Robotica, Instituto Superior Tecnico
 *
 * This file is part of RoAH Devices.
 *
 * RoAH Devices is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * RoAH Devices is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with RoAH Devices.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __RQT_ROAH_DEVICES_DEVICES_CONTROL_H__
#define __RQT_ROAH_DEVICES_DEVICES_CONTROL_H__

#include <QTimer>

#include <ros/ros.h>
#include <rqt_gui_cpp/plugin.h>

#include <ui_devices_control.h>
#include <roah_devices/DevicesState.h>
#include <topic_receiver.h>



namespace rqt_roah_devices
{
  class DevicesControl
    : public rqt_gui_cpp::Plugin
  {
      Q_OBJECT

    public:
      DevicesControl();
      virtual void initPlugin (qt_gui_cpp::PluginContext& context);
      virtual void shutdownPlugin();

      // Comment in to signal that the plugin has a way to configure it
      //bool hasConfiguration() const;
      //void triggerConfiguration();

    private:
      Ui::DevicesControl ui_;
      QWidget* widget_;
      QTimer update_timer_;
      TopicReceiver<roah_devices::DevicesState> rcv_;
      const ros::Duration CONTROL_DURATION;
      const ros::Duration BELL_DURATION;
      ros::Time last_control_;
      ros::Time last_bell_rcvd_;
      ros::Time last_bell_time_;

    private slots:
      void update();
      void bell();
      void switch_1();
      void switch_2();
      void switch_3();
      void dimmer (int value);
      void blinds (int value);
  };
}

#endif
