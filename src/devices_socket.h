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

#ifndef __DEVICES_SOCKET_H__
#define __DEVICES_SOCKET_H__

#include <boost/range/algorithm.hpp>
#include <boost/asio.hpp>



using namespace std;
using namespace boost::asio;
using boost::asio::ip::tcp;



inline void
serialize_byte (uint8_t val,
                vector<uint8_t>& out)
{
  out.push_back (val);
}



inline void
serialize_short (uint16_t val,
                 vector<uint8_t>& out)
{
  out.push_back ( (val >> 8) & 0xFF);
  out.push_back ( (val) & 0xFF);
}



inline void
serialize_long (uint32_t val,
                vector<uint8_t>& out)
{
  out.push_back ( (val >> 24) & 0xFF);
  out.push_back ( (val >> 16) & 0xFF);
  out.push_back ( (val >> 8) & 0xFF);
  out.push_back ( (val) & 0xFF);
}



inline void
serialize_string (string const& val,
                  vector<uint8_t>& out)
{
  serialize_short (val.size(), out);
  if (val.empty()) {
    return;
  }
  boost::copy (val, back_inserter (out));
}



inline size_t
deserialize_byte (uint8_t& val,
                  vector<uint8_t> const& in,
                  size_t offset = 0)
{
  val = in.at (offset);
  return 1;
}



inline size_t
deserialize_short (uint16_t& val,
                   vector<uint8_t> const& in,
                   size_t offset = 0)
{
  val = in.at (offset);
  val <<= 8;
  val |= in.at (offset + 1);
  return 2;
}



inline size_t
deserialize_long (uint32_t& val,
                  vector<uint8_t> const& in,
                  size_t offset = 0)
{
  val = in.at (offset);
  val <<= 8;
  val |= in.at (offset + 1);
  val <<= 8;
  val |= in.at (offset + 2);
  val <<= 8;
  val |= in.at (offset + 3);
  return 4;
}



inline size_t
deserialize_string (size_t size,
                    string& val,
                    vector<uint8_t> const& in,
                    size_t offset = 0)
{
  if (size == 0) {
    val.clear();
    return 0;
  }
  val = string (& (in.at (offset)), 1 + & (in.at (offset + size - 1)));
  return size;
}



inline size_t
deserialize_string (string& val,
                    vector<uint8_t> const& in,
                    size_t offset = 0)
{
  uint16_t size;
  size_t size_size = deserialize_short (size, in, offset);
  return size_size + deserialize_string (size, val, in, offset + size_size);
}



inline uint8_t
sync_read_byte (tcp::socket& socket)
{
  vector<uint8_t> b (1);
  if (! read (socket, buffer (b))) {
    throw exception();
  }
  uint8_t val;
  deserialize_byte (val, b);
  return val;
}



inline uint16_t
sync_read_short (tcp::socket& socket)
{
  vector<uint8_t> b (2);
  if (! read (socket, buffer (b))) {
    throw exception();
  }
  uint16_t val;
  deserialize_short (val, b);
  return val;
}



inline uint32_t
sync_read_long (tcp::socket& socket)
{
  vector<uint8_t> b (4);
  if (! read (socket, buffer (b))) {
    throw exception();
  }
  uint32_t val;
  deserialize_long (val, b);
  return val;
}



inline string
sync_read_string (tcp::socket& socket)
{
  uint16_t length = sync_read_short (socket);

  if (length == 0) {
    return "";
  }

  vector<uint8_t> b (length);
  if (! read (socket, buffer (b))) {
    throw exception();
  }
  string val;
  deserialize_string (length, val, b);
  return val;
}



inline void
sync_write_byte (tcp::socket& socket,
                 uint8_t val)
{
  vector<uint8_t> b;
  serialize_byte (val, b);
  if (! write (socket, buffer (b))) {
    throw exception();
  }
}



inline void
sync_write_short (tcp::socket& socket,
                  uint16_t val)
{
  vector<uint8_t> b;
  serialize_short (val, b);
  if (! write (socket, buffer (b))) {
    throw exception();
  }
}



inline void
sync_write_long (tcp::socket& socket,
                 uint32_t val)
{
  vector<uint8_t> b;
  serialize_long (val, b);
  if (! write (socket, buffer (b))) {
    throw exception();
  }
}



inline void
sync_write_string (tcp::socket& socket,
                   string const& val)
{
  vector<uint8_t> b;
  serialize_string (val, b);
  if (! write (socket, buffer (b))) {
    throw exception();
  }
}

#endif
