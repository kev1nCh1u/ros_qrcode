#include "qr_code.h"                            // 引用 ros.h 檔
QRcode::QRcode(char *dev_name, int Baudrate)
{
    //Serial
    int baudrate = setBaudrate(Baudrate);
    if(SerialConnect(mySerial, dev_name, (speed_t)baudrate) < 0){

        ROS_INFO("Serial failed");
    }
    else
    {
        qrcode_Publisher_ = handler.advertise<QR_code::send>("qrcode", 10);
        receive_thread_ = new boost::thread(boost::bind(&QRcode::RevProcess, this, 0.01));
    }
}

QRcode::~QRcode()
{

}

void QRcode::receive_data(std::vector<unsigned char> receive)
{
    int High_X = 0, X = 0, _Tag = 0;
    int16_t High_Y = 0, Y = 0;
    int High_A = 0, A = 0;
    int Tag1 = 0, Tag2 = 0, Tag3 = 0, Tag4 = 0;

    High_X = int(receive[2] & 0x04 >> 2);
    X = int(int(receive[5] & 0x7f) | int((receive[4] & 0x7f) << 7) | int((receive[3] & 0x7f) << 14) |
        int((receive[2] & 0x07) << 21) | High_X << 24 | High_X << 25 | High_X << 26 | High_X << 27 | High_X << 28
        | High_X << 29 | High_X << 30 | High_X << 31);

    High_Y = int16_t((receive[6] & 0x40) >> 6);
    Y = int16_t(int16_t(receive[7] & 0x7f) | int16_t((receive[6] & 0x7f) << 7) | High_Y << 12 | High_Y << 13 | High_Y << 14 | High_Y << 15);

    High_A = int((receive[10] & 0x40) >> 6);
    A = int(int(receive[11] & 0x7f) | int((receive[10] & 0x7f) << 7) | High_A << 12 | High_A << 13 | High_A << 14 | High_A << 15);

    Tag1 = int((receive[14] & 0x40) >> 6);
    Tag2 = int(int(receive[15] & 0x7f) | int((receive[14] & 0x7f) << 7) | Tag1 << 12 | Tag1 << 13 | Tag1 << 14 | Tag1 << 15);

    Tag3 = int((receive[16] & 0x40) >> 6);
    Tag4 = int(int(receive[17] & 0x7f) | int((receive[16] & 0x7f) << 7) | Tag3 << 12 | Tag3 << 13 | Tag3 << 14 | Tag3 << 15);

    _Tag = int(int(Tag2 << 16) | int(Tag4));
    std::cout<<"X = "<<X<<std::endl;
    std::cout<<"Y = "<<Y<<std::endl;
    std::cout<<"A = "<<A<<std::endl;
    std::cout<<"_Tag = "<<_Tag<<std::endl;

    QR_code::send msg;
    msg.x = X;
    msg.y = Y;
    msg.A = A;
    msg.Tag = _Tag;
    qrcode_Publisher_.publish(msg);
}

void QRcode::SendPackage(std::vector<unsigned char> recv)
{
    unsigned char command[recv.size()];
    for(int i = 0;i<recv.size(); i++)
  	{
        command[i] = recv[i];
        //printf("%hhx \n",command[i]);
  	}

    int nByte = 0;
    if(mySerial.serial_ok)
    {
        nByte = write(mySerial.fd,command,recv.size());
        //std::cout<<"start sends"<<std::endl;
    }
}

void QRcode::RevProcess(double receive_period)
{
    //ros::Rate r_receive(1.0*30.0 / receive_period);
    ros::Rate r_receive(10);
    std::cout<<"start receive"<<std::endl;
    int Receive_Package_Size = 256;
    std::vector<unsigned char> rev_buf;
    std::vector<unsigned char> send_buf;
    // unsigned char input_data = 0xC8;
    // unsigned char input_data1 = 0x37;
    // send_buf.push_back(input_data);
    // send_buf.push_back(input_data1);
    // SendPackage(send_buf);
    // send_buf.clear();
    while(1)
    {
        if(mySerial.serial_ok == true)
        {
            unsigned char send_data = 0xc8;
            unsigned char send_data1 = 0x37;
            send_buf.push_back(send_data);
            send_buf.push_back(send_data1);
            SendPackage(send_buf);
            send_buf.clear();
            unsigned char buff[Receive_Package_Size];
            int readByte = 0;
            readByte = read(mySerial.fd,buff,21);
            if(readByte > 0)
            {
      					for(int i=0; i<readByte; i++)
                {
      						rev_buf.push_back(buff[i]);
                  //std::cout<<buff[i]<<std::endl;
                }
                std::cout<<"============================"<<std::endl;
                std::cout<<"rev_buf.size()"<<rev_buf.size()<<std::endl;
                if(rev_buf.size() == 21)
        				{
        					receive_data(rev_buf);
        					rev_buf.clear();
        				}
                rev_buf.clear();
    				}
        }
        r_receive.sleep();
    }
}

int QRcode::SerialConnect(SERIAL &serial, char *port_name, int speed)
{
    serial.serial_ok = false;
    serial.oldtio.c_cflag |= PARENB;//even
    serial.oldtio.c_cflag &= ~PARODD;//even
    serial.oldtio.c_cflag |= INPCK;//even
    serial.oldtio.c_cflag |= ISTRIP;//even
    cfsetospeed(&serial.oldtio, (speed_t)speed);
    cfsetispeed(&serial.oldtio, (speed_t)speed);
    serial.fd = open(port_name, O_RDWR | O_NOCTTY);
    if(serial.fd < 0)
    {
        std::cout<<"Error opening"<<std::endl;
        return -1;
    }
    if(tcgetattr(serial.fd, &(serial.oldtio)) < 0){
        std::cout<<"Error from tcgetattr"<<std::endl;
        return -1;
    }
    // serial.oldtio.c_cflag |= PARENB;//odd
    // serial.oldtio.c_cflag |= ~PARODD;//odd
    // serial.oldtio.c_cflag |= INPCK;//odd
    // serial.oldtio.c_cflag |= ISTRIP;//odd
    // serial.oldtio.c_cflag |= PARENB;//even
    // serial.oldtio.c_cflag &= ~PARODD;//even
    // serial.oldtio.c_cflag |= INPCK;//even
    // serial.oldtio.c_cflag |= ISTRIP;//even
    // cfsetospeed(&serial.oldtio, (speed_t)speed);
    // cfsetispeed(&serial.oldtio, (speed_t)speed);
    serial.serial_ok = true;

    std::cout<<"Serial Opened"<<std::endl;

    return 0;
}

int QRcode::setBaudrate(int baudrate)
{
    if(baudrate == 50)
        return B50;
    else if(baudrate == 75)
        return B75;
    else if(baudrate == 110)
        return B110;
    else if(baudrate == 134)
        return B134;
    else if(baudrate == 150)
        return B150;
    else if(baudrate == 200)
        return B200;
    else if(baudrate == 300)
        return B300;
    else if(baudrate == 600)
        return B600;
    else if(baudrate == 1200)
        return B1200;
    else if(baudrate == 1800)
        return B1800;
    else if(baudrate == 2400)
        return B2400;
    else if(baudrate == 4800)
        return B4800;
    else if(baudrate == 9600)
        return B9600;
    else if(baudrate == 19200)
        return B19200;
    else if(baudrate == 38400)
        return B38400;
		else if(baudrate == 57600)
		    return B57600;
    else if(baudrate == 115200)
        return B115200;
    else
        return B0;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "qr_code");     // 初始化 hello_cpp_node
    ros::Time::init();
    QRcode *qrcode;
    if(argc < 3)
    {
        ROS_INFO("usage: [<Device name>] [<Baud rate>]");
        return 0;
    }
    baudrate = std::atoi(argv[2]);
    dev_name = argv[1];
    qrcode = new QRcode(dev_name, baudrate);
    ros::spin();
    return 0;
}
