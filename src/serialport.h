#ifndef SERIALPORT_H
#define SERIALPORT_H

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <boost/thread.hpp>
#include <boost/shared_array.hpp>

typedef boost::function<void(const char*, int)> RecieveCallBack;

class SerialPort
{
public:
    //Constructor
    //Destructor
    SerialPort();

    ~SerialPort();

    //Initialize port
    bool init_port( const char* port, int nBaudrate = 115200);

    //Write some data to port
    void write(const char *data, size_t size);

    void write(const std::vector<char>& data);

    void setCallBack(RecieveCallBack pfunc);

    bool isOpened() const;

    void close();

private:

    //Read data from port which write data just now
    void read_from_serial();

    //The asyanc callback function of asyanc_read
    void handle_read( char buf[], boost::system::error_code ec,
        std::size_t bytes_transferred );

    void doWrite();

    void writeEnd(const boost::system::error_code& error);

    bool isopen;

    //io_service Object
    boost::asio::io_service m_ios;

    //Serial port Object
    boost::asio::serial_port *pSerialPort;

    //Serial_port function exception
    boost::system::error_code ec;

    RecieveCallBack pCallBackFunc;

    boost::thread *pthread;

    /// Data are queued here before they go in writeBuffer
    std::vector<char> writeQueue;
    boost::shared_array<char> writeBuffer; ///< Data being written
    size_t writeBufferSize; ///< Size of writeBuffer
    boost::mutex writeQueueMutex; ///< Mutex for access to writeQueue

    char m_buffer[255];
};

#endif // SERIALPORT_H
