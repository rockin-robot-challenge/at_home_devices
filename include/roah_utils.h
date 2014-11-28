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

#ifndef __ROAH_DEVICES_UTILS_H__
#define __ROAH_DEVICES_UTILS_H__

#include <cstdlib>
#include <sstream>
#include <boost/date_time/posix_time/time_formatters.hpp>
#include <boost/date_time/c_local_time_adjustor.hpp>
#include <QString>
#include <ros/ros.h>



inline QString
to_qstring (ros::Time const& time)
{
  return QString::fromStdString (boost::posix_time::to_simple_string (boost::date_time::c_local_adjustor<boost::posix_time::ptime>::utc_to_local (time.toBoost())));
}



inline QString
to_qstring (ros::Duration const& duration)
{
  int mins = std::abs (duration.sec / 60);
  int secs = std::abs (duration.sec % 60);
  return (duration < ros::Duration() ? QString ("-") : QString())
         + (mins < 10 ? QString ("0") : QString())
         + QString::number (mins)
         + ":"
         + (secs < 10 ? QString ("0") : QString())
         + QString::number (secs);
}



template<typename T>
inline bool
call_service (std::string const& service,
              T& data)
{
  if (! ros::service::waitForService (service, 100)) {
    ROS_ERROR_STREAM ("Could not find service " << service);
    return false;
  }
  if (! ros::service::call (service, data)) {
    ROS_ERROR_STREAM ("Error calling service " << service);
    return false;
  }
  return true;
}

#endif
