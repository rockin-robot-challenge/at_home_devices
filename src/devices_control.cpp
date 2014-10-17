/*
 * Copyright 2014 Instituto de Sistemas e Robotica, Instituto Superior Tecnico
 *
 * This file is part of RoAH RSBB.
 *
 * RoAH RSBB is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * RoAH RSBB is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with RoAH RSBB.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "devices_control.h"

#include <boost/lexical_cast.hpp>
#include <boost/date_time/posix_time/time_formatters.hpp>
#include <boost/date_time/c_local_time_adjustor.hpp>

#include <QStringList>
#include <QMessageBox>

#include <pluginlib/class_list_macros.h>

#include <std_srvs/Empty.h>
#include <roah_devices/Bool.h>
#include <roah_devices/Percentage.h>

#include <roah_utils.h>



using namespace std;
using namespace ros;
using boost::lexical_cast;



namespace rqt_roah_devices
{
  DevicesControl::DevicesControl()
    : rqt_gui_cpp::Plugin()
    , widget_ (0)
    , CONTROL_DURATION (1.0)
    , BELL_DURATION (1.0)
    , last_control_ (TIME_MIN)
    , last_bell_rcvd_ (TIME_MIN)
    , last_bell_time_ (TIME_MIN)
  {
    setObjectName ("DevicesControl");
  }

  void DevicesControl::initPlugin (qt_gui_cpp::PluginContext& context)
  {
    // access standalone command line arguments
    QStringList argv = context.argv();
    // create QWidget
    widget_ = new QWidget();
    // extend the widget with all attributes and children from UI file
    ui_.setupUi (widget_);
    // add widget to the user interface
    context.addWidget (widget_);

    connect (ui_.bell_ring, SIGNAL (clicked()), this, SLOT (bell()));
    connect (ui_.switch_1_toggle, SIGNAL (clicked()), this, SLOT (switch_1()));
    connect (ui_.switch_2_toggle, SIGNAL (clicked()), this, SLOT (switch_2()));
    connect (ui_.switch_3_toggle, SIGNAL (clicked()), this, SLOT (switch_3()));
    connect (ui_.dimmer_in, SIGNAL (valueChanged (int)), this, SLOT (dimmer (int)));
    connect (ui_.blinds_in, SIGNAL (valueChanged (int)), this, SLOT (blinds (int)));
    rcv_.start ("/devices/state", getNodeHandle());

    connect (&update_timer_, SIGNAL (timeout()), this, SLOT (update()));
    update_timer_.start (200);
  }

  void DevicesControl::shutdownPlugin()
  {
    update_timer_.stop();
    rcv_.stop();
  }

  void DevicesControl::update()
  {
    Time now = Time::now();

    roah_devices::DevicesState::ConstPtr status = rcv_.last ();

    if (! status) {
      return;
    }

    if (status->bell != TIME_MIN) {
      ui_.bell_time->setText (to_qstring (status->bell));
    }
    else {
      ui_.bell_time->setText ("--");
    }
    if (last_bell_rcvd_ != status->bell) {
      last_bell_time_ = Time::now();
      last_bell_rcvd_ = status->bell;
    }
    if ( (now - last_bell_time_) < BELL_DURATION) {
      ui_.bell_warn->setText ("!!!");
    }
    else {
      ui_.bell_warn->setText ("");
    }
    ui_.switch_1->setText (status->switch_1 ? "ON" : "OFF");
    ui_.switch_2->setText (status->switch_2 ? "ON" : "OFF");
    ui_.switch_3->setText (status->switch_3 ? "ON" : "OFF");
    ui_.dimmer->setText (QString::number (status->dimmer) + " %");
    ui_.blinds->setText (QString::number (status->blinds) + " %");

    if ( (now - last_control_) < CONTROL_DURATION) {
      return;
    }

    ui_.dimmer_in->blockSignals (true);
    ui_.dimmer_in->setValue (status->dimmer);
    ui_.dimmer_in->blockSignals (false);
    ui_.blinds_in->blockSignals (true);
    ui_.blinds_in->setValue (status->blinds);
    ui_.blinds_in->blockSignals (false);
  }

  void DevicesControl::bell()
  {
    std_srvs::Empty e;
    call_service ("/devices/bell/mock", e);
    last_control_ = Time::now();
  }

  void DevicesControl::switch_1()
  {
    roah_devices::DevicesState::ConstPtr status = rcv_.last ();
    roah_devices::Bool b;
    b.request.data = ! status->switch_1;
    call_service ("/devices/switch_1/set", b);
    last_control_ = Time::now();
  }

  void DevicesControl::switch_2()
  {
    roah_devices::DevicesState::ConstPtr status = rcv_.last ();
    roah_devices::Bool b;
    b.request.data = ! status->switch_2;
    call_service ("/devices/switch_2/set", b);
    last_control_ = Time::now();
  }

  void DevicesControl::switch_3()
  {
    roah_devices::DevicesState::ConstPtr status = rcv_.last ();
    roah_devices::Bool b;
    b.request.data = ! status->switch_3;
    call_service ("/devices/switch_3/set", b);
    last_control_ = Time::now();
  }

  void DevicesControl::dimmer (int value)
  {
    roah_devices::Percentage p;
    p.request.data = value;
    call_service ("/devices/dimmer/set", p);
    last_control_ = Time::now();
  }

  void DevicesControl::blinds (int value)
  {
    roah_devices::Percentage p;
    p.request.data = value;
    call_service ("/devices/blinds/set", p);
    last_control_ = Time::now();
  }
}



PLUGINLIB_DECLARE_CLASS (rqt_roah_devices, DevicesControl, rqt_roah_devices::DevicesControl, rqt_gui_cpp::Plugin)
