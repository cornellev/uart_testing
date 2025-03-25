#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>
#include <cstring>
#include <string>
#include <thread>

#define UART_DEV "/dev/ttyUSB0"
#define BAUD_RATE B115200

int configure_uart(const char* device_path) {
    int serial_port = open(device_path, O_RDWR | O_NOCTTY);

    if (serial_port < 0) {
        perror("Error opening serial port");
        return -1;
    }

    struct termios tty;
    if (tcgetattr(serial_port, &tty) != 0) {
        perror("tcgetattr failed");
        return -1;
    }

    cfsetispeed(&tty, BAUD_RATE);
    cfsetospeed(&tty, BAUD_RATE);

    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag |= CREAD | CLOCAL;

    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_oflag &= ~OPOST;

    tty.c_cc[VMIN]  = 1;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        perror("tcsetattr failed");
        return -1;
    }

    return serial_port;
}

void receive_loop(int serial_port) {
    char ch;
    std::string line;

    while (true) {
        std::cout << "[Debug] Waiting for data...\n";
        int num_bytes = read(serial_port, &ch, 1);
        std::cout << "[Debug] Read " << num_bytes << " bytes: " << ch << "\n";
        if (num_bytes < 0) {
            std::cerr << "Error reading: " << strerror(errno) << "\n";
            break;
        } else if (num_bytes > 0) {
            if (ch == '\n') {
                std::cout << "[Received] " << line << "\n";
                line.clear();
            } else {
                line += ch;
            }
        }
    }
}

void send_message(int serial_port, const std::string& msg) {
    std::string to_send = msg + "\n";
    int written = write(serial_port, to_send.c_str(), to_send.size());
    if (written < 0) {
        std::cerr << "Error writing: " << strerror(errno) << "\n";
    }
}

int main() {
    int serial_port = configure_uart(UART_DEV);
    if (serial_port < 0) return 1;

    std::thread rx_thread(receive_loop, serial_port);

    std::string input;
    while (true) {
        unsigned char msg[] = { 'H', 'e', 'l', 'l', 'o', '\r' };
        write(serial_port, msg, sizeof(msg));
        //send_message(serial_port, input);
    }

    rx_thread.join();
    close(serial_port);
    return 0;
}
