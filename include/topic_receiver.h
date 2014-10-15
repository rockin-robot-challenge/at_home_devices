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

#ifndef __ROAH_DEVICES_TOPIC_RECEIVER_H__
#define __ROAH_DEVICES_TOPIC_RECEIVER_H__

#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>

#include <ros/ros.h>



template <class TOPIC_TYPE>
class TopicReceiver
{
  public:
    typedef typename TOPIC_TYPE::ConstPtr ConstPtrType;

    void start (std::string const& topic,
                ros::NodeHandle& nh)
    {
      sub_ = nh.subscribe (topic, 1, &TopicReceiver::callback, this);
    }

    void stop()
    {
      sub_.shutdown();
    }

    ConstPtrType last()
    {
      ConstPtrType last_msg;

      {
        boost::lock_guard<boost::mutex> _ (mutex_);
        last_msg = last_;
      }

      return last_msg;
    }

    ConstPtrType last (ros::Time& time)
    {
      ConstPtrType last_msg;

      {
        boost::lock_guard<boost::mutex> _ (mutex_);
        last_msg = last_;
        time = time_;
      }

      return last_msg;
    }

  private:
    ros::Subscriber sub_;
    boost::mutex mutex_;
    ConstPtrType last_;
    ros::Time time_;

    void callback (ConstPtrType const& msg)
    {
      boost::lock_guard<boost::mutex> _ (mutex_);
      last_ = msg;
      time_ = ros::Time::now();
    }
};

#endif
