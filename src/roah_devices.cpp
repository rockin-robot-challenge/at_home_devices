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

    bool set (roah_devices::Bool::Request& req, roah_devices::Bool::Response& res)
    {
      setter_ (req.data ? 1 : 0);
      return true;
    }

    bool on (std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
    {
      setter_ (1);
      return true;
    }

    bool off (std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
    {
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
    {
    }
};



class PercentageSwitch
{
    ServiceServer srv_;
    ServiceServer max_srv_;
    ServiceServer min_srv_;
    boost::function<void (int32_t) > setter_;

    bool set (roah_devices::Percentage::Request& req, roah_devices::Percentage::Response& res)
    {
      if ( (req.data < 0) || (req.data > 100)) {
        return false;
      }

      setter_ (req.data);
      return true;
    }

    bool max (std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
    {
      setter_ (100);
      return true;
    }

    bool min (std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
    {
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
    {
    }
};



class RoahDevices
{
    NodeHandle nh_;

    Publisher devices_pub_;
    Publisher bell_pub_;

    BoolSwitch switch_1_;
    BoolSwitch switch_2_;
    BoolSwitch switch_3_;
    PercentageSwitch dimmer_;
    PercentageSwitch blinds_;

    io_service io_service_;
    tcp::socket socket_;
    boost::thread thread_;

    void write_byte (uint8_t val)
    {
      if (! write (socket_, buffer (&val, sizeof (val)))) {
        throw exception();
      }
    }

    void write_string (string const& val)
    {
      uint16_t length = val.size();
      length = boost::asio::detail::socket_ops::host_to_network_short (length);
      if (! write (socket_, buffer (&length, sizeof (length)))) {
        throw exception();
      }

      if (val.size() == 0) {
        return;
      }

      vector<string::value_type> content (val.begin(), val.end());
      content.resize (val.size());
      if (! write (socket_, buffer (content))) {
        throw exception();
      }
    }

    void write_int (int32_t val)
    {
      val = boost::asio::detail::socket_ops::host_to_network_long (val);
      if (! write (socket_, buffer (&val, sizeof (val)))) {
        throw exception();
      }
    }

    void set_int (string const& arg0, int32_t arg1)
    {
      try {
        write_byte ('I');
        write_string (arg0);
        write_int (arg1);
        ROS_DEBUG_STREAM ("setInt(\"" << arg0 << "\", " << arg1 << ")");
      }
      catch (exception) {}
    }

    uint8_t read_byte()
    {
      uint8_t val;
      if (! read (socket_, buffer (&val, sizeof (val)))) {
        throw exception();
      }
      return val;
    }

    string read_string()
    {
      uint16_t length;
      if (! read (socket_, buffer (&length, sizeof (length)))) {
        throw exception();
      }
      length = boost::asio::detail::socket_ops::network_to_host_short (length);

      if (length == 0) {
        return "";
      }

      vector<string::value_type> content (static_cast<size_t> (length));
      if (! read (socket_, buffer (content))) {
        throw exception();
      }

      return string (content.begin(), content.end());
    }

    int32_t read_int()
    {
      int32_t val;
      if (! read (socket_, buffer (&val, sizeof (val)))) {
        throw exception();
      }
      return boost::asio::detail::socket_ops::network_to_host_long (val);
    }

    void process_command()
    {
      uint8_t command_byte = read_byte();

      switch (command_byte) {
        case 'E': {
          string arg0 = read_string();
          ROS_ERROR_STREAM ("Cannot connect to server: already a connection from: " << arg0);
          // TODO keep trying
          abort();
        }
        break;
        case '1': {
          string arg0 = read_string();
          int arg1 = read_int();
          int arg2 = read_int();
          ROS_DEBUG_STREAM ("notifyChanges(\"" << arg0 << "\", " << arg1 << ", " << arg2 << ")");
        }
        break;
        case '2': {
          char arg0 = read_byte();
          string arg1 = read_string();
          string arg2 = read_string();
          ROS_DEBUG_STREAM ("notifyChanges(" << (arg0 == 'N' ? "NEW" : arg0 == 'E' ? "EDIT" : arg0 == 'D' ? "DELETE" : "UNKNOWN") << ", \"" << arg1 << "\", \"" << arg2 << "\")");
        }
        break;
        default:
          ROS_FATAL_STREAM ("Received unknown command value " << ( (int) command_byte) << " char " << ( (char) command_byte));
          abort();
      }
    }

    void run_thread()
    {
      string smartif_host;
      param::param<string> ("~smartif_host", smartif_host, "192.168.1.56");
      tcp::endpoint endpoint (ip::address::from_string (smartif_host), 6665);
      socket_.connect (endpoint);
      ROS_DEBUG_STREAM ("Connected to " << endpoint);
      try {
        while (ok()) {
          process_command();
        }
      }
      catch (exception) {}
      socket_.close();
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
      socket_.shutdown (socket_base::shutdown_both);
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
