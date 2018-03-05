#include <string>
#include <vector>
#include "serialport.h"

using namespace std;
using namespace boost;
using namespace boost::asio;

//Define Constructor function
SerialPort::SerialPort( )
    :pSerialPort( NULL )
    ,pCallBackFunc(NULL)
    ,pthread(NULL)
    ,isopen(false)
{
     pSerialPort = new serial_port(m_ios);
}

//Define destructor function
SerialPort::~SerialPort()
{
   if( pSerialPort )
   {
       if(isopen)
       {
           pSerialPort->close();
           m_ios.stop();
           pthread->join();
           m_ios.reset();
           delete pthread;
       }
        delete pSerialPort;
   }
}


//Initialize port
bool SerialPort::init_port( const char* port, int nBaudrate)
{
    //New not success
    if ( !pSerialPort )
    {
        return false;
    }

    if(isopen)
        return true;

    //Open Serial port object
    pSerialPort->open(port,  ec );


    if(ec)
    {
        std::cout << "Open port failed:   " << ec.message() << std::endl;
        return false;
    }

    //Set port argument
    pSerialPort->set_option( serial_port::baud_rate( nBaudrate ), ec );
    pSerialPort->set_option( serial_port::flow_control( serial_port::flow_control::none ), ec );
    pSerialPort->set_option( serial_port::parity( serial_port::parity::none ), ec );
    pSerialPort->set_option( serial_port::stop_bits( serial_port::stop_bits::one ), ec);
    pSerialPort->set_option( serial_port::character_size( 8 ), ec);

    read_from_serial();
    pthread = new boost::thread(boost::bind(&io_service::run, &m_ios));
    isopen = true;
    return true;
}

bool SerialPort::isOpened() const
{
    return isopen;
}

void SerialPort::close()
{
    if ( pSerialPort && isopen)
    {
        pSerialPort->close();
        m_ios.stop();
        pthread->join();

        m_ios.reset();
        delete pthread;
        isopen = false;
    }
}

void SerialPort::setCallBack(RecieveCallBack pfunc)
{
    pCallBackFunc = pfunc;
}

void SerialPort::write(const char *data, size_t size)
{
    {
        lock_guard<mutex> l(writeQueueMutex);
        writeQueue.insert(writeQueue.end(),data,data+size);
    }
    m_ios.post(boost::bind(&SerialPort::doWrite, this));
}

void SerialPort::write(const std::vector<char>& data)
{
    {
        lock_guard<mutex> l(writeQueueMutex);
        writeQueue.insert(writeQueue.end(),data.begin(),
                data.end());
    }
    m_ios.post(boost::bind(&SerialPort::doWrite, this));
}

void SerialPort::doWrite()
{
    //If a write operation is already in progress, do nothing
    if(writeBuffer==0)
    {
        lock_guard<mutex> l(writeQueueMutex);
        writeBufferSize=writeQueue.size();
        writeBuffer.reset(new char[writeQueue.size()]);
        copy(writeQueue.begin(),writeQueue.end(),
                writeBuffer.get());
        writeQueue.clear();
        async_write(*pSerialPort, buffer(writeBuffer.get(),
                writeBufferSize),
                boost::bind(&SerialPort::writeEnd, this, asio::placeholders::error));
    }
}

void SerialPort::writeEnd(const boost::system::error_code& error)
{
    if(!error)
    {
        lock_guard<mutex> l(writeQueueMutex);
        if(writeQueue.empty())
        {
            writeBuffer.reset();
            writeBufferSize=0;

            return;
        }
        writeBufferSize=writeQueue.size();
        writeBuffer.reset(new char[writeQueue.size()]);
        copy(writeQueue.begin(),writeQueue.end(),
                writeBuffer.get());
        writeQueue.clear();
        async_write(*pSerialPort,asio::buffer(writeBuffer.get(),
                writeBufferSize),
                boost::bind(&SerialPort::writeEnd, this, asio::placeholders::error));
    }
}

void SerialPort::handle_read( char *buf, boost::system::error_code ec,
    std::size_t bytes_transferred )
{
    if(this->pCallBackFunc)
    {
        this->pCallBackFunc(buf, bytes_transferred);
    }
    read_from_serial();
}


//Read data from the serial
void SerialPort::read_from_serial()
{
    pSerialPort->async_read_some(buffer(m_buffer, 255), boost::bind( &SerialPort::handle_read, this, m_buffer, _1, _2));
}

