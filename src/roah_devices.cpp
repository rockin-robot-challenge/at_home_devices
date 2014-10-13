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

#include <string>

#include <boost/thread.hpp>
#include <boost/asio.hpp>

#include <ros/ros.h>

#include <std_srvs/Empty.h>
#include <roah_devices/Bool.h>
#include <roah_devices/Percentage.h>

#include "device_ids.h"
#include "devices_socket.h"



using namespace std;
using namespace ros;
using namespace boost::asio;
using boost::asio::ip::tcp;



class BoolSwitch
{
    ServiceServer srv_;
    ServiceServer on_srv_;
    ServiceServer off_srv_;
    boost::function<void (int32_t) > setter_;
    bool state_;

    bool set (roah_devices::Bool::Request& req, roah_devices::Bool::Response& res)
    {
      state_ = req.data;
      setter_ (req.data ? 1 : 0);
      return true;
    }

    bool on (std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
    {
      state_ = true;
      setter_ (1);
      return true;
    }

    bool off (std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
    {
      state_ = false;
      setter_ (0);
      return true;
    }

  public:
    BoolSwitch (NodeHandle& nh,
                string const& name,
                boost::function<void (int32_t) > setter)
      : srv_ (nh.advertiseService ("/devices/" + name + "/set", &BoolSwitch::set, this))
      , on_srv_ (nh.advertiseService ("/devices/" + name + "/on", &BoolSwitch::on, this))
      , off_srv_ (nh.advertiseService ("/devices/" + name + "/off", &BoolSwitch::off, this))
      , setter_ (setter)
      , state_ (false)
    {
    }

    void set_current()
    {
      setter_ (state_ ? 1 : 0);
    }
};



class PercentageSwitch
{
    ServiceServer srv_;
    ServiceServer max_srv_;
    ServiceServer min_srv_;
    boost::function<void (int32_t) > setter_;
    uint8_t state_;

    bool set (roah_devices::Percentage::Request& req, roah_devices::Percentage::Response& res)
    {
      if ( (req.data < 0) || (req.data > 100)) {
        return false;
      }

      state_ = req.data;
      setter_ (req.data);
      return true;
    }

    bool max (std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
    {
      state_ = 100;
      setter_ (100);
      return true;
    }

    bool min (std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
    {
      state_ = 0;
      setter_ (0);
      return true;
    }

  public:
    PercentageSwitch (NodeHandle& nh,
                      string const& name,
                      boost::function<void (int32_t) > setter)
      : srv_ (nh.advertiseService ("/devices/" + name + "/set", &PercentageSwitch::set, this))
      , max_srv_ (nh.advertiseService ("/devices/" + name + "/max", &PercentageSwitch::max, this))
      , min_srv_ (nh.advertiseService ("/devices/" + name + "/min", &PercentageSwitch::min, this))
      , setter_ (setter)
      , state_ (0)
    {
    }

    void set_current()
    {
      setter_ (state_);
    }
};



class RoahDevices
{
    NodeHandle nh_;

    BoolSwitch switch_1_;
    BoolSwitch switch_2_;
    BoolSwitch switch_3_;
    PercentageSwitch dimmer_;
    PercentageSwitch blinds_;

    io_service io_service_;
    tcp::socket socket_;
    boost::mutex write_mutex_;
    boost::thread thread_;

    uint8_t command_;

    void
    set_int (string const& arg0,
             int32_t arg1)
    {
      boost::lock_guard<boost::mutex> _ (write_mutex_);
      try {
        if (! socket_.is_open()) {
          return;
        }

        sync_write_byte (socket_, 'I');
        sync_write_string (socket_, arg0);
        sync_write_long (socket_, arg1);
        ROS_DEBUG_STREAM ("setInt(\"" << arg0 << "\", " << arg1 << ")");
      }
      catch (...) {}
    }

    void
    start_read()
    {
      async_read (socket_,
                  buffer (&command_, 1),
                  boost::bind (&RoahDevices::handle_read, this,
                               boost::asio::placeholders::error));
    }

    void
    handle_read (const boost::system::error_code& error)
    {
      if (error) {
        cerr << "Some error code in handle_read" << endl << flush;
        io_service_.stop();
        return;
      }

      if (command_ != 'I') {
        cerr << "UNKNOWN COMMAND value " << static_cast<int> (command_) << endl << flush;
        io_service_.stop();
        return;
      }

      try {
        switch (command_) {
          case 'E': {
            string arg0 = sync_read_string (socket_);
            ROS_ERROR_STREAM ("Cannot connect to server: already a connection from: " << arg0);
            // TODO keep trying
            abort();
          }
          break;
          case '1': {
            string arg0 = sync_read_string (socket_);
            int arg1 = sync_read_long (socket_);
            int arg2 = sync_read_long (socket_);
            ROS_DEBUG_STREAM ("notifyChanges(\"" << arg0 << "\", " << arg1 << ", " << arg2 << ")");
          }
          break;
          case '2': {
            char arg0 = sync_read_byte (socket_);
            string arg1 = sync_read_string (socket_);
            string arg2 = sync_read_string (socket_);
            ROS_DEBUG_STREAM ("notifyChanges(" << (arg0 == 'N' ? "NEW" : arg0 == 'E' ? "EDIT" : arg0 == 'D' ? "DELETE" : "UNKNOWN") << ", \"" << arg1 << "\", \"" << arg2 << "\")");
          }
          break;
          default:
            ROS_FATAL_STREAM ("Received unknown command value " << ( (int) command_) << " char " << ( (char) command_));
            abort();
        }
      }
      catch (...) {
        cerr << "Some error in sync reads" << endl << flush;
        io_service_.stop();
        return;
      }

      start_read();
    }

    void run_thread()
    {
      while (ok()) {
        string smartif_host;
        param::param<string> ("~smartif_host", smartif_host, "192.168.1.56");
        tcp::endpoint endpoint (ip::address::from_string (smartif_host), 6665);
        try {
          socket_.connect (endpoint);
          ROS_DEBUG_STREAM ("Connected to " << endpoint);
          start_read();
          switch_1_.set_current();
          switch_2_.set_current();
          switch_3_.set_current();
          dimmer_.set_current();
          blinds_.set_current();
          io_service_.run();
        }
        catch (...) {}
        socket_.close();
        io_service_.reset();
        if (ok()) {
          Duration (0.2).sleep();
        }
      }
    }

  public:
    RoahDevices()
      : nh_()
      , switch_1_ (nh_, "switch_1", boost::bind (&RoahDevices::set_int, this, SWITCH_1_ID, _1))
      , switch_2_ (nh_, "switch_2", boost::bind (&RoahDevices::set_int, this, SWITCH_2_ID, _1))
      , switch_3_ (nh_, "switch_3", boost::bind (&RoahDevices::set_int, this, SWITCH_3_ID, _1))
      , dimmer_ (nh_, "dimmer", boost::bind (&RoahDevices::set_int, this, DIMMER_ID, _1))
      , blinds_ (nh_, "blinds", boost::bind (&RoahDevices::set_int, this, BLINDS_ID, _1))
      , io_service_()
      , socket_ (io_service_)
      , thread_ (&RoahDevices::run_thread, this)
    {
    }

    ~RoahDevices()
    {
      // socket_.shutdown (socket_base::shutdown_both);
      io_service_.stop();
      thread_.join();
    }
};



int main (int argc, char** argv)
{
  init (argc, argv, "roah_devices");

  RoahDevices node;

  spin();

  return 0;
}
