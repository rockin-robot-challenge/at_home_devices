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

#include <iostream>

#include <boost/make_shared.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/asio.hpp>
#include <boost/bind.hpp>

#include "device_ids.h"
#include "devices_socket.h"



using namespace std;
using namespace boost;
using namespace boost::asio;
using boost::asio::ip::tcp;



class DummyServer
{
    io_service io_service_;
    tcp::acceptor acceptor_;
    shared_ptr<tcp::socket> acceptor_socket_;
    shared_ptr<tcp::socket> connection_socket_;

    uint8_t command_;

    void
    start_accept()
    {
      // cout << "Accepting..." << endl;
      acceptor_socket_ = make_shared<tcp::socket> (boost::ref (io_service_));
      acceptor_.async_accept (*acceptor_socket_,
                              boost::bind (&DummyServer::handle_accept,
                                           this,
                                           acceptor_socket_,
                                           placeholders::error));
    }

    void
    handle_accept (shared_ptr<tcp::socket> socket,
                   boost::system::error_code const& error)
    {
      if (! error) {
        if (! connection_socket_) {
          cout << "DUMMY SERVER: Connection accepted from " << lexical_cast<string> (socket->remote_endpoint()) << endl << flush;
          connection_socket_ = socket;
          start_read();
        }
        else {
          cout << "DUMMY SERVER: Connection REJECTED from " << lexical_cast<string> (socket->remote_endpoint()) << endl << flush;
          shared_ptr<vector<uint8_t> > b = make_shared<vector<uint8_t> >();
          serialize_byte ('E', *b);
          serialize_string (lexical_cast<string> (connection_socket_->remote_endpoint()), *b);
          async_write (*socket,
                       buffer (*b),
                       bind (&DummyServer::handle_write_and_forget,
                             socket,
                             b,
                             boost::asio::placeholders::error));
        }
      }

      start_accept();
    }

    static void
    handle_write_and_forget (boost::shared_ptr<boost::asio::ip::tcp::socket> /*socket*/,
                             boost::shared_ptr<std::vector<uint8_t> > /*buffer*/,
                             boost::system::error_code const& /*error*/)
    {
    }

    void
    start_read()
    {
      async_read (*connection_socket_,
                  buffer (&command_, 1),
                  boost::bind (&DummyServer::handle_read, this,
                               boost::asio::placeholders::error));
    }

    void
    handle_read (const boost::system::error_code& error)
    {
      if (error) {
        cerr << "DUMMY SERVER: Some error code in handle_read" << endl << flush;
        connection_socket_.reset();
        return;
      }

      if (command_ != 'I') {
        cerr << "DUMMY SERVER: UNKNOWN COMMAND value " << static_cast<int> (command_) << endl << flush;
        connection_socket_.reset();
        return;
      }

      string arg0;
      int32_t arg1;
      try {
        arg0 = sync_read_string (*connection_socket_);
        arg1 = sync_read_long (*connection_socket_);
      }
      catch (...) {
        cerr << "DUMMY SERVER: Some error in sync reads" << endl << flush;
        connection_socket_.reset();
        return;
      }

      if (arg0 == SWITCH_1_ID) {
        switch (arg1) {
          case 0:
            cout << "DUMMY SERVER: Switch 1 turned OFF" << endl << flush;
            break;
          case 1:
            cout << "DUMMY SERVER: Switch 1 turned ON" << endl << flush;
            break;
          default:
            cerr << "DUMMY SERVER: Switch 1 turned to UNKNOWN VALUE " << arg1 << endl << flush;
            break;
        }
      }
      else if (arg0 == SWITCH_2_ID) {
        switch (arg1) {
          case 0:
            cout << "DUMMY SERVER: Switch 2 turned OFF" << endl << flush;
            break;
          case 1:
            cout << "DUMMY SERVER: Switch 2 turned ON" << endl << flush;
            break;
          default:
            cerr << "DUMMY SERVER: Switch 2 turned to UNKNOWN VALUE " << arg1 << endl << flush;
            break;
        }
      }
      else if (arg0 == SWITCH_3_ID) {
        switch (arg1) {
          case 0:
            cout << "DUMMY SERVER: Switch 3 turned OFF" << endl << flush;
            break;
          case 1:
            cout << "DUMMY SERVER: Switch 3 turned ON" << endl << flush;
            break;
          default:
            cerr << "DUMMY SERVER: Switch 3 turned to UNKNOWN VALUE " << arg1 << endl << flush;
            break;
        }
      }
      else if (arg0 == DIMMER_ID) {
        if ( (arg1 >= 0) && (arg1 <= 100)) {
          cout << "DUMMY SERVER: Dimmer turned to " << arg1 << "%" << endl << flush;
        }
        else {
          cerr << "DUMMY SERVER: Dimmer turned to UNKNOWN VALUE " << arg1 << endl << flush;
        }
      }
      else if (arg0 == BLINDS_ID) {
        if ( (arg1 >= 0) && (arg1 <= 100)) {
          cout << "DUMMY SERVER: Blinds turned to " << arg1 << "%" << endl << flush;
        }
        else {
          cerr << "DUMMY SERVER: Blinds turned to UNKNOWN VALUE " << arg1 << endl << flush;
        }
      }
      else {
        cerr << "DUMMY SERVER: Received command for UNKNOWN DEVICE \"" << arg0 << "\"" << endl << flush;
      }

      start_read();
    }

  public:
    DummyServer()
      : io_service_()
      , acceptor_ (io_service_, tcp::endpoint (tcp::v4(), 6665))
    {
      start_accept();
    }

    ~DummyServer()
    {
      // shutdown sockets?
      io_service_.stop();
    }

    void
    run()
    {
      io_service_.run();
    }
};



int main (int argc, char** argv)
{
  DummyServer node;
  node.run();
  return 0;
}
