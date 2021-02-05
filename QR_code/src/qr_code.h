#include <ros/ros.h>
#include "ros/package.h"
#include <boost/thread.hpp>
#include <std_msgs/Bool.h>

#include <iostream>
#include <fstream>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <string>
#include <math.h>
#include <std_msgs/Int16.h>
#include <QR_code/send.h>

//全域變數
int baudrate;
char *dev_name;
struct SERIAL
{
    struct termios  oldtio;
    int fd,res,done;
    bool serial_ok;
};

class QRcode
{
    public:
      QRcode(char *dev_name, int Baudrate);
      ~QRcode();
      void RevProcess(double receive_period);
      //setup seril &file
      int setBaudrate(int baudrate);
      int SerialConnect(SERIAL &serial, char *port_name, int speed);
      void SendPackage(std::vector<unsigned char> recv);
      void receive_data(std::vector<unsigned char> receive);
      SERIAL mySerial;
      ///////////////////
    protected:
      boost::thread* receive_thread_;
      ros::NodeHandle handler;                     // node 的 handler
      ros::Publisher qrcode_Publisher_;
    private:

};
