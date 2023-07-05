#include <iostream>
#include <thread>

#include <boost/asio.hpp>
#include <boost/thread.hpp>

#include "protocol.hpp"

namespace asio = boost::asio;

typedef boost::shared_ptr<boost::asio::serial_port> serial_port_ptr;

#define SERIAL_PORT_READ_BUF_SIZE 256

namespace
{
    class SerialPortService
    {
    public:
        SerialPortService();
        ~SerialPortService();
        bool start(const char *com_port_name);
        void stop();

        int write_some(const char *buf, const int &size);
    private:
        boost::asio::io_service io_service_;
        serial_port_ptr port_;
        boost::mutex mutex_;

        char read_buf_raw_[SERIAL_PORT_READ_BUF_SIZE]{};

        void async_read_some_();
        void on_receive_(const boost::system::error_code& ec, size_t bytes_transferred);

        SerialFeedback msg{}, prev_msg{};
        uint16_t start_frame = 0;
        char prev_byte = 0;
        int msg_len = 0;
        char* p{};
    };
}

SerialPortService::SerialPortService() = default;

SerialPortService::~SerialPortService()
{
    stop();
}

bool SerialPortService::start(const char *com_port_name)
{
    boost::system::error_code ec;

    if (port_) {
        std::cout << "error : port is already opened..." << std::endl;
        return false;
    }

    port_ = serial_port_ptr(new boost::asio::serial_port(io_service_));
    port_->open(com_port_name, ec);
    if (ec) {
        std::cout << "error : port_->open() failed...com_port_name="
                  << com_port_name << ", e=" << ec.message().c_str() << std::endl;
        return false;
    }

    // option settings...
    port_->set_option(boost::asio::serial_port_base::baud_rate(115200));
    port_->set_option(boost::asio::serial_port_base::character_size(8));
    port_->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
    port_->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
    port_->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));

    boost::thread t([ObjectPtr = &io_service_] { return ObjectPtr->run(); });

    async_read_some_();

    return true;
}

void SerialPortService::stop()
{
    boost::mutex::scoped_lock look(mutex_);

    if (port_) {
        port_->cancel();
        port_->close();
        port_.reset();
    }

    io_service_.stop();
    io_service_.reset();
}

void SerialPortService::async_read_some_()
{
    if (port_.get() == nullptr || !port_->is_open()) return;

    port_->async_read_some(
            boost::asio::buffer(read_buf_raw_, SERIAL_PORT_READ_BUF_SIZE),
            boost::bind(
                    &SerialPortService::on_receive_,
                    this, boost::asio::placeholders::error,
                    boost::asio::placeholders::bytes_transferred));
}

void SerialPortService::on_receive_(const boost::system::error_code& ec, size_t bytes_transferred)
{
    boost::mutex::scoped_lock look(mutex_);

    if (port_.get() == nullptr || !port_->is_open()) return;

    if (ec) {
        async_read_some_();
        return;
    }

    for (unsigned int i = 0; i < bytes_transferred; ++i) {
        //char c = read_buf_raw_[i];

        start_frame = ((uint16_t)(read_buf_raw_[i]) << 8) | (uint8_t) prev_byte;

        if (start_frame == START_FRAME) {
            p = (char*)&msg;
            *p++ = prev_byte;
            *p++ = read_buf_raw_[i];
            msg_len = 2;
        } else if (msg_len >= 2 && msg_len < sizeof(SerialFeedback)) {
            // Otherwise just read the message content until the end
            *p++ = read_buf_raw_[i];
            msg_len++;
        }

        //std::cout << "msg_len!" << msg_len << std::endl;

        if (msg_len == sizeof(SerialFeedback))
        {
            std::cout << "Mam to! " << msg.wheelR_cnt << std::endl;

            msg_len = 0;
        }

        prev_byte = read_buf_raw_[i];
    }

    async_read_some_();
}

int SerialPortService::write_some(const char *buf, const int &size)
{
    boost::system::error_code ec;

    if (!port_) return -1;
    if (size == 0) return 0;

    return port_->write_some(boost::asio::buffer(buf, size), ec);
}

int main() {

    asio::io_context ctx;
    SerialPortService serialPortService;

    serialPortService.start("/dev/ttyUSB0");

    SerialCommand command;

    command.start = (uint16_t)START_FRAME;
    command.steer = (int16_t) 0;
    command.speed = (int16_t) 30;
    command.checksum = (uint16_t)(command.start ^ command.steer ^ command.speed);

    while (true) {
        std::this_thread::sleep_for(std::chrono::milliseconds (50));
        serialPortService.write_some((const char*) &command, sizeof(command));
    }

    serialPortService.stop();

    std::cout << "Hello, World!" << std::endl;
    return 0;
}
