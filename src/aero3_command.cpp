#include "seed_smartactuator_sdk/aero3_command.h"
#include <iostream> // for cout/cerr
using namespace aero;
using namespace controller;

//#define DEBUG

///////////////////////////////
SerialCommunication::SerialCommunication()
: io_(),serial_(io_),timer_(io_),is_canceled_(false),comm_err_(false)
{
}

///////////////////////////////
SerialCommunication::~SerialCommunication()
{
  if(serial_.is_open())serial_.close();
}

///////////////////////////////
bool SerialCommunication::openPort(std::string _port, unsigned int _baud_rate)
{
  boost::system::error_code error_code;
  serial_.open(_port,error_code);
  if(error_code){
      return false;
  }
  else{
    serial_.set_option(serial_port_base::baud_rate(_baud_rate));
    return true;
  }
}

///////////////////////////////
void SerialCommunication::closePort()
{
  if(serial_.is_open())serial_.close();
}

///////////////////////////////
void SerialCommunication::writeAsync(std::vector<uint8_t>& _send_data)
{
  serial_.async_write_some( buffer( _send_data ), [](boost::system::error_code, std::size_t){});
  io_.reset();
  io_.run();
}

///////////////////////////////
void SerialCommunication::onReceive(const boost::system::error_code& _error, size_t _bytes_transferred)
{
  if (_error == boost::asio::error::operation_aborted){
#ifdef DEBUG
      std::cout << "Timeout " << std::endl;
#endif
  }
  else if (_error && _error != boost::asio::error::eof) {
#ifdef DEBUG
      std::cout << "receive failed: " << std::endl;
#endif
  }
  else {
    const std::string data(boost::asio::buffer_cast<const char*>(stream_buffer_.data()), stream_buffer_.size());
    receive_buffer_ = data;

    stream_buffer_.consume(stream_buffer_.size());
    timer_.cancel();
    is_canceled_ = true;
  }
}

///////////////////////////////
void SerialCommunication::onTimer(const boost::system::error_code& _error)
{
    if (!_error && !is_canceled_) serial_.cancel();
}

///////////////////////////////
void SerialCommunication::readBufferAsync(uint8_t _size=1, uint16_t _timeout=10)
{
  receive_buffer_.clear();
  is_canceled_ = false;

  boost::asio::async_read(serial_,stream_buffer_,boost::asio::transfer_at_least(_size),
      boost::bind(&SerialCommunication::onReceive, this,
          boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));

  //transfer_all() -> read until full
  //transfer_exactly(size_t size) -> read specific size
  //transfer_at_least(size_t size) -> read at leaset size
  timer_.expires_from_now(boost::posix_time::milliseconds(_timeout));
  timer_.async_wait(boost::bind(&SerialCommunication::onTimer, this, _1));
  io_.reset();
  io_.run();
}
///////////////////////////////
void SerialCommunication::readBufferAsync(std::vector<uint8_t>& _receive_data, uint8_t _size = 1, uint16_t _timeout=1)
{
  is_canceled_ = false;
  boost::asio::async_read(serial_,buffer(_receive_data,_receive_data.size()),boost::asio::transfer_at_least(_size),
      boost::bind(&SerialCommunication::onReceive, this,
          boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
  timer_.expires_from_now(boost::posix_time::milliseconds(_timeout));
  timer_.async_wait(boost::bind(&SerialCommunication::onTimer, this, _1));
  io_.reset();
  io_.run();
}

void SerialCommunication::readBuffer(std::vector<uint8_t>& _receive_data, uint8_t _length = 1)
{
  _receive_data.clear();
  _receive_data.resize(_length);
  fill(_receive_data.begin(),_receive_data.end(),0);

  Header header;
  int header_size = 4;
  int break_counter = 0;
  cosmo_cmd_.clear();
  std::vector<uint8_t> receive_buf;

  do{
    receive_buf.resize(header_size);
    readBufferAsync( receive_buf,receive_buf.size(), 1000);
    header.stx = static_cast<uint16_t>((receive_buf[0] << 8) + receive_buf[1]);
    header.data_length = receive_buf[2];
    header.command = receive_buf[3];

    if(header.stx == 0){    //command not received
      std::cerr << "Read Timeout" << std::endl;
      comm_err_ = true;
      break;
    }

    if(header.stx == 0xDFFD) {   //in case of aero command
      for(size_t i=0;i<header_size;++i) _receive_data[i] = receive_buf[i];
      receive_buf.resize(header.data_length);
      readBufferAsync(receive_buf,receive_buf.size(), 100);
      if(receive_buf.size() != header.data_length){
        std::cerr << "Buffer Error" << std::endl;
        flushPort();
        comm_err_ = true;
        break;
      }
      for(size_t i=0;i<header.data_length;++i) _receive_data[i+header_size] = receive_buf[i];
      comm_err_ = false;
    }
    else if(header.stx == 0xBFFB){   //in case of cosmo command
      break_counter ++;
      cosmo_cmd_.resize(header_size + header.data_length);

      for(size_t i=0;i<header_size;++i) cosmo_cmd_[i] = receive_buf[i];
      receive_buf.resize(header.data_length);
      readBufferAsync(receive_buf,receive_buf.size(), 100);
      if(receive_buf.size() != header.data_length){
        std::cerr << "Buffer Error" << std::endl;
        flushPort();
        comm_err_ = true;
        break;
      }
      for(size_t i=0;i<header.data_length;++i) cosmo_cmd_[i+header_size] = receive_buf[i];
    }

    else{
        break_counter ++;
        std::cerr << "Buffer Error" << std::endl;
        flushPort();
        comm_err_ = true;
        break;
    }
#ifdef DEBUG
    if(cosmo_cmd_.size() != 0){
      for(size_t i=0;i<cosmo_cmd_.size() ;++i) std::cout << std::hex << (int)(cosmo_cmd_[i] & 0x000000FF) << std::dec;
      std::cout << std::endl;
      for(size_t i=0;i<_length ;++i) std::cout << std::hex << (int)(_receive_data[i] & 0x000000FF) << std::dec;
      std::cout << std::endl;    }
    else{
      for(size_t i=0;i<_length ;++i) std::cout << std::hex << (int)(_receive_data[i] & 0x000000FF) << std::dec;
      std::cout << std::endl;
    }
#endif
  } while(header.stx != 0xDFFD & break_counter <5);

}

///////////////////////////////
void SerialCommunication::flushPort()
{
  ::tcflush(serial_.lowest_layer().native_handle(),TCIOFLUSH);
}

//*******************************************************************
//*******************************************************************
///////////////////////////////
AeroCommand::AeroCommand()
:is_open_(false),check_sum_(0),count_(0),length_(0),serial_com_()
{

}

///////////////////////////////
AeroCommand::~AeroCommand()
{
  closePort();
}

bool AeroCommand::openPort(std::string _port, unsigned int _baud_rate){
  if(serial_com_.openPort(_port, _baud_rate)) is_open_ = true;
  else is_open_ = false;

  return is_open_;
}

void AeroCommand::closePort(){
  serial_com_.closePort();
  is_open_ = false;
}

void AeroCommand::flushPort(){
  //serial_com_.flushPort();
}

///////////////////////////////
void AeroCommand::setCurrent(uint8_t _number,uint8_t _max, uint8_t _down)
{
  check_sum_ = 0;

  if(_number == 0)length_ = 68;
  else length_ = 8;

  send_data_.resize(length_);
  fill(send_data_.begin(),send_data_.end(),0);

  send_data_[0] = 0xFD;
  send_data_[1] = 0xDF;
  send_data_[2] = length_-4;
  send_data_[3] = 0x01;
  send_data_[4] = _number;

  for(unsigned int i = 0;i < (length_-6)/2; ++i){
    send_data_[i*2+5] = _max;
    send_data_[i*2+6] = _down;
  }

  //CheckSum
  for(count_ = 2;count_ < length_-1;count_++) check_sum_ += send_data_[count_];
  send_data_[length_-1] = ~check_sum_;

  //serial_com_.flushPort();
  serial_com_.writeAsync(send_data_);

}

///////////////////////////////
void AeroCommand::onServo(uint8_t _number,uint16_t _data)
{
  check_sum_ = 0;

  if(_number == 0)length_ = 68;
  else length_ = 8;

  send_data_.resize(length_);
  fill(send_data_.begin(),send_data_.end(),0);

  send_data_[0] = 0xFD;
  send_data_[1] = 0xDF;
  send_data_[2] = length_-4;
  send_data_[3] = 0x21;
  send_data_[4] = _number;

  for(unsigned int i = 0;i < (length_-6)/2; ++i){
    send_data_[i*2+5] = _data >> 8;
    send_data_[i*2+6] = _data;
  }

  //CheckSum
  for(count_ = 2;count_ < length_-1;count_++) check_sum_ += send_data_[count_];
  send_data_[length_-1] = ~check_sum_;

  //serial_com_.flushPort();
  serial_com_.writeAsync(send_data_);

}


///////////////////////////////
std::vector<int16_t> AeroCommand::getPosition(uint8_t _number)
{
  check_sum_ = 0;
  length_ = 6;

  send_data_.resize(length_);
  fill(send_data_.begin(),send_data_.end(),0);

  send_data_[0] = 0xFD;
  send_data_[1] = 0xDF;
  send_data_[2] = length_-4;
  send_data_[3] = 0x41;
  send_data_[4] = _number;

  //CheckSum
  for(count_ = 2;count_ < length_-1;count_++) check_sum_ += send_data_[count_];
  send_data_[length_-1] = ~check_sum_;

  //serial_com_.flushPort();
  serial_com_.writeAsync(send_data_);

  std::vector<uint8_t> receive_data;
  if(_number == 0) receive_data.resize(68);
  else receive_data.resize(8);
  fill(receive_data.begin(),receive_data.end(),0);

  serial_com_.readBuffer(receive_data,receive_data.size());
  comm_err_ = serial_com_.comm_err_;
  std::vector<int16_t> parse_data;
  if(_number==0) parse_data.resize(30);
  else parse_data.resize(1);
  fill(parse_data.begin(),parse_data.end(),0);
  for(size_t i=0; i < parse_data.size() ; ++i){
    parse_data[i] = static_cast<int16_t>((receive_data[i*2+5] << 8) + receive_data[i*2+6]);
  }

  return parse_data;

}

///////////////////////////////
std::vector<uint16_t> AeroCommand::getCurrent(uint8_t _number)
{
  check_sum_ = 0;
  length_ = 6;

  send_data_.resize(length_);
  fill(send_data_.begin(),send_data_.end(),0);

  send_data_[0] = 0xFD;
  send_data_[1] = 0xDF;
  send_data_[2] = length_-4;
  send_data_[3] = 0x42;
  send_data_[4] = _number;

  //CheckSum
  for(count_ = 2;count_ < length_-1;count_++) check_sum_ += send_data_[count_];
  send_data_[length_-1] = ~check_sum_;

  //serial_com_.flushPort();
  serial_com_.writeAsync(send_data_);

  std::vector<uint8_t> receive_data;
  if(_number == 0) receive_data.resize(68);
  else receive_data.resize(8);
  fill(receive_data.begin(),receive_data.end(),0);

  serial_com_.readBuffer(receive_data,receive_data.size());
  comm_err_ = serial_com_.comm_err_;
  std::vector<uint16_t> parse_data;
  if(_number==0) parse_data.resize(31);
  else parse_data.resize(1);
  fill(parse_data.begin(),parse_data.end(),0);
  for(size_t i=0; i < parse_data.size() ; ++i){
    parse_data[i] = static_cast<uint16_t>((receive_data[i*2+5] << 8) + receive_data[i*2+6]);
  }

  return parse_data;

}

///////////////////////////////
std::vector<uint16_t> AeroCommand::getTemperatureVoltage(uint8_t _number)
{
  check_sum_ = 0;
  length_ = 6;

  send_data_.resize(length_);
  fill(send_data_.begin(),send_data_.end(),0);

  send_data_[0] = 0xFD;
  send_data_[1] = 0xDF;
  send_data_[2] = length_-4;
  send_data_[3] = 0x43;
  send_data_[4] = _number;

  //CheckSum
  for(count_ = 2;count_ < length_-1;count_++) check_sum_ += send_data_[count_];
  send_data_[length_-1] = ~check_sum_;

  //serial_com_.flushPort();
  serial_com_.writeAsync(send_data_);

  std::vector<uint8_t> receive_data;
  if(_number == 0) receive_data.resize(68);
  else receive_data.resize(8);
  fill(receive_data.begin(),receive_data.end(),0);

  serial_com_.readBuffer(receive_data,receive_data.size());
  comm_err_ = serial_com_.comm_err_;
  std::vector<uint16_t> parse_data;
  if(_number==0) parse_data.resize(31);
  else parse_data.resize(1);
  fill(parse_data.begin(),parse_data.end(),0);
  for(size_t i=0; i < parse_data.size() ; ++i){
    parse_data[i] = static_cast<uint16_t>((receive_data[i*2+5] << 8) + receive_data[i*2+6]);
  }

  return parse_data;

}

///////////////////////////////
std::string AeroCommand::getVersion(uint8_t _number)
{
  check_sum_ = 0;
  length_ = 6;

  send_data_.resize(length_);
  fill(send_data_.begin(),send_data_.end(),0);

  send_data_[0] = 0xFD;
  send_data_[1] = 0xDF;
  send_data_[2] = length_-4;
  send_data_[3] = 0x51;
  send_data_[4] = _number;

  //CheckSum
  for(count_ = 2;count_ < length_-1;count_++) check_sum_ += send_data_[count_];
  send_data_[length_-1] = ~check_sum_;

  //serial_com_.flushPort();
  serial_com_.writeAsync(send_data_);

  std::vector<uint8_t> receive_data;
  receive_data.resize(11);
  fill(receive_data.begin(),receive_data.end(),0);

  serial_com_.readBuffer(receive_data,receive_data.size());
  comm_err_ = serial_com_.comm_err_;

  std::string version = "";
  char data[3];
  for(size_t i=0; i < 5 ; ++i){
    sprintf(data,"%02X", receive_data[i+5]);
    version += data;
  }

  return version;
}

///////////////////////////////
std::vector<uint16_t> AeroCommand::getStatus(uint8_t _number)
{
  check_sum_ = 0;
  length_ = 6;

  send_data_.resize(length_);
  fill(send_data_.begin(),send_data_.end(),0);

  send_data_[0] = 0xFD;
  send_data_[1] = 0xDF;
  send_data_[2] = length_-4;
  send_data_[3] = 0x52;
  send_data_[4] = _number;

  //CheckSum
  for(count_ = 2;count_ < length_-1;count_++) check_sum_ += send_data_[count_];
  send_data_[length_-1] = ~check_sum_;

  //serial_com_.flushPort();
  serial_com_.writeAsync(send_data_);

  std::vector<uint8_t> receive_data;
  if(_number == 0) receive_data.resize(68);
  else receive_data.resize(8);
  fill(receive_data.begin(),receive_data.end(),0);

  serial_com_.readBuffer(receive_data,receive_data.size());
  comm_err_ = serial_com_.comm_err_;

  std::vector<uint16_t> parse_data;    //status data
  if(_number==0) parse_data.resize(31);
  else parse_data.resize(1);
  fill(parse_data.begin(),parse_data.end(),0);
  for(size_t i=0; i < parse_data.size() ; ++i){
    parse_data[i] = static_cast<uint16_t>((receive_data[i*2+5] << 8) + receive_data[i*2+6]);
  }

  return parse_data;
}

 ///////////////////////////////
void AeroCommand::throughCAN(uint8_t _send_no,uint8_t _command,
uint8_t _data1, uint8_t _data2, uint8_t _data3, uint8_t _data4, uint8_t _data5)
{
  check_sum_ = 0;
  length_ = 12;

  send_data_.resize(length_);
  fill(send_data_.begin(),send_data_.end(),0);

  send_data_[0] = 0xFD;   //Headder
  send_data_[1] = 0xDF;   //Headder
  send_data_[2] = 8;      //Data Length
  send_data_[3] = 0x5F;   //Command
  send_data_[4] = _send_no;    //Send No.

  send_data_[5] = _command;
  send_data_[6] = _data1;
  send_data_[7] = _data2;
  send_data_[8] = _data3;
  send_data_[9] = _data4;
  send_data_[10] = _data5;

  //CheckSum
  for(count_ = 2;count_ < length_-1;count_++) check_sum_ += send_data_[count_];
  send_data_[length_-1] = ~check_sum_;

  //serial_com_.flushPort();
  serial_com_.writeAsync(send_data_);
}

///////////////////////////////
std::vector<int16_t> AeroCommand::actuateByPosition(uint16_t _time, int16_t *_data)
{
  check_sum_ = 0;
  length_ = 68;

  send_data_.resize(length_);
  fill(send_data_.begin(),send_data_.end(),0);

  send_data_[0] = 0xFD;
  send_data_[1] = 0xDF;
  send_data_[2] = length_-4;
  send_data_[3] = 0x14;
  send_data_[4] = 0x00;

  for (int i = 0; i < 30; i++){
    send_data_[i*2 + 5] = _data[i] >> 8;
    send_data_[i*2 + 6] = _data[i] ;
  }

  //time
  send_data_[65] = _time >>8;
  send_data_[66] = _time;

  //CheckSum
  for(count_ = 2;count_ < length_-1;count_++) check_sum_ += send_data_[count_];
  send_data_[length_-1] = ~check_sum_;

  //serial_com_.flushPort();
  serial_com_.writeAsync(send_data_);

  std::vector<uint8_t> receive_data;
  receive_data.resize(68);
  fill(receive_data.begin(),receive_data.end(),0);

  serial_com_.readBuffer(receive_data,receive_data.size());
  comm_err_ = serial_com_.comm_err_;

  if(serial_com_.cosmo_cmd_.size() != 0){
    std::copy(serial_com_.cosmo_cmd_.begin(),serial_com_.cosmo_cmd_.end(),std::back_inserter(cosmo_cmd_));
  }
  else{
    cosmo_cmd_.clear();
  }

  std::vector<int16_t> parse_data;    //present position & robot status
  parse_data.resize(31);
  fill(parse_data.begin(),parse_data.end(),0);
  for(size_t i=0; i < parse_data.size() ; ++i){
    parse_data[i] = static_cast<int16_t>((receive_data[i*2+5] << 8) + receive_data[i*2+6]);
  }
  return parse_data;

}

///////////////////////////////
std::vector<int16_t> AeroCommand::actuateBySpeed(int16_t *_data)
{
  check_sum_ = 0;
  length_ = 68;

  send_data_.resize(length_);
  fill(send_data_.begin(),send_data_.end(),0);

  send_data_[0] = 0xFD;
  send_data_[1] = 0xDF;
  send_data_[2] = length_-4;
  send_data_[3] = 0x15;
  send_data_[4] = 0x00;

  for (int i = 0; i < 30; i++){
    send_data_[i*2 + 5] = _data[i] >> 8;
    send_data_[i*2 + 6] = _data[i] ;
  }

  //CheckSum
  for(count_ = 2;count_ < length_-1;count_++) check_sum_ += send_data_[count_];
  send_data_[length_-1] = ~check_sum_;

  //serial_com_.flushPort();
  serial_com_.writeAsync(send_data_);

  std::vector<uint8_t> receive_data;
  receive_data.resize(68);
  fill(receive_data.begin(),receive_data.end(),0);

  serial_com_.readBuffer(receive_data,receive_data.size());
  comm_err_ = serial_com_.comm_err_;

  std::vector<int16_t> parse_data;    //present position & robot status
  parse_data.resize(31);
  fill(parse_data.begin(),parse_data.end(),0);
  for(size_t i=0; i < parse_data.size() ; ++i){
    parse_data[i] = static_cast<int16_t>((receive_data[i*2+5] << 8) + receive_data[i*2+6]);
  }

  return parse_data;
}

///////////////////////////////
void AeroCommand::runScript(uint8_t _number,uint16_t _data)
{

  check_sum_ = 0;

  if(_number == 0)length_ = 68;
  else length_ = 8;

  send_data_.resize(length_);
  fill(send_data_.begin(),send_data_.end(),0);

  send_data_[0] = 0xFD;
  send_data_[1] = 0xDF;
  send_data_[2] = length_-4;
  send_data_[3] = 0x22;
  send_data_[4] = _number;

  for(unsigned int i = 0;i < (length_-6)/2;++i) send_data_[i*2+6] = _data;

  //CheckSum
  for(count_ = 2;count_ < length_-1;count_++) check_sum_ += send_data_[count_];
  send_data_[length_-1] = ~check_sum_;

  //serial_com_.flushPort();
  serial_com_.writeAsync(send_data_);
}
