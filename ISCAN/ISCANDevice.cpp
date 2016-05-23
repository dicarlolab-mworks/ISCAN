//
//  ISCANDevice.cpp
//  ISCAN
//
//  Created by Christopher Stawarz on 5/20/16.
//  Copyright © 2016 The MWorks Project. All rights reserved.
//

#include "ISCANDevice.hpp"


BEGIN_NAMESPACE_MW


const std::string ISCANDevice::SERIAL_PORT("serial_port");
const std::string ISCANDevice::START_COMMAND("start_command");
const std::string ISCANDevice::STOP_COMMAND("stop_command");


inline std::string ISCANDevice::getOutputParameterName(std::size_t outputNumber) {
    return (boost::format("output_0%d") % (outputNumber + 1)).str();
}


void ISCANDevice::describeComponent(ComponentInfo &info) {
    IODevice::describeComponent(info);
    
    info.setSignature("iodevice/iscan");
    
    info.addParameter(SERIAL_PORT);
    for (std::size_t outputNumber = 0; outputNumber < maxNumOutputs; outputNumber++) {
        info.addParameter(getOutputParameterName(outputNumber), false);
    }
    info.addParameter(START_COMMAND, "128");
    info.addParameter(STOP_COMMAND, "129");
}


ISCANDevice::ISCANDevice(const ParameterValueMap &parameters) :
    IODevice(parameters),
    serialPort(parameters[SERIAL_PORT].str()),
    startCommand(parameters[START_COMMAND]),
    stopCommand(parameters[STOP_COMMAND]),
    fd(-1),
    running(false)
{
    for (std::size_t outputNumber = 0; outputNumber < maxNumOutputs; outputNumber++) {
        auto &outputParameter = parameters[getOutputParameterName(outputNumber)];
        if (!outputParameter.empty()) {
            outputs.at(outputNumber) = VariablePtr(outputParameter);
        }
    }
}


ISCANDevice::~ISCANDevice() {
    if (receiveDataThread.joinable()) {
        continueReceivingData.clear();
        receiveDataThread.join();
    }
    
    if (-1 != fd) {
        // Block until all written output has been sent to the device
        if (-1 == tcdrain(fd)) {
            merror(M_IODEVICE_MESSAGE_DOMAIN, "Serial port drain failed: %s", strerror(errno));
        }
        
        disconnect();
    }
}


bool ISCANDevice::initialize() {
    if (!connect()) {
        return false;
    }
    
    continueReceivingData.test_and_set();
    receiveDataThread = std::thread([this]() {
        receiveData();
    });
    
    return true;
}


bool ISCANDevice::connect() {
    // Open the serial port read/write, with no controlling terminal, and don't wait for a connection.
    // The O_NONBLOCK flag also causes subsequent I/O on the device to be non-blocking.
    if (-1 == (fd = ::open(serialPort.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK))) {
        merror(M_IODEVICE_MESSAGE_DOMAIN, "Cannot open serial port (%s): %s", serialPort.c_str(), strerror(errno));
        return false;
    }
    
    bool shouldClose = true;
    BOOST_SCOPE_EXIT(&shouldClose, &fd) {
        if (shouldClose) {
            (void)::close(fd);
            fd = -1;
        }
    } BOOST_SCOPE_EXIT_END
    
    // open() follows POSIX semantics: multiple open() calls to the same file will succeed
    // unless the TIOCEXCL ioctl is issued.  This will prevent additional opens except by root-owned
    // processes.
    if (-1 == ioctl(fd, TIOCEXCL)) {
        merror(M_IODEVICE_MESSAGE_DOMAIN, "Cannot obtain exclusive use of serial port: %s", strerror(errno));
        return false;
    }
    
    // Now that the device is open, clear the O_NONBLOCK flag so subsequent I/O will block
    if (-1 == fcntl(fd, F_SETFL, 0)) {
        merror(M_IODEVICE_MESSAGE_DOMAIN, "Cannot restore blocking I/O on serial port: %s", strerror(errno));
        return false;
    }
    
    // Get the current options and save them, so we can restore the default settings later
    if (-1 == tcgetattr(fd, &origAttrs)) {
        merror(M_IODEVICE_MESSAGE_DOMAIN, "Cannot obtain current serial port attributes: %s", strerror(errno));
        return false;
    }
    
    struct termios attrs = origAttrs;
    cfmakeraw(&attrs);            // Set raw input (non-canonical) mode
    attrs.c_cc[VMIN] = 0;         // Reads block until a single byte has been received
    attrs.c_cc[VTIME] = 5;        //   or a 500ms timeout expires
    cfsetspeed(&attrs, B115200);  // Set speed to 115200 baud
    attrs.c_cflag |= CS8;         // Use 8-bit words
    attrs.c_cflag &= ~PARENB;     // No parity
    attrs.c_cflag &= ~CSTOPB;     // 1 stop bit
    attrs.c_cflag |= CLOCAL;      // Ignore modem status lines
    
    // Cause the new options to take effect immediately
    if (-1 == tcsetattr(fd, TCSANOW, &attrs)) {
        merror(M_IODEVICE_MESSAGE_DOMAIN, "Cannot set serial port attributes: %s", strerror(errno));
        return false;
    }
    
    shouldClose = false;
    
    return true;
}


void ISCANDevice::disconnect() {
    // Restore original options
    if (-1 == tcsetattr(fd, TCSANOW, &origAttrs)) {
        merror(M_IODEVICE_MESSAGE_DOMAIN, "Cannot restore previous serial port attributes: %s", strerror(errno));
    }
    
    if (-1 == ::close(fd)) {
        merror(M_IODEVICE_MESSAGE_DOMAIN, "Cannot close serial port: %s", strerror(errno));
    }
    
    fd = -1;
}


bool ISCANDevice::startDeviceIO() {
    lock_guard lock(mutex);
    
    if (!running) {
        if (!sendCommand(startCommand)) {
            merror(M_IODEVICE_MESSAGE_DOMAIN, "Cannot start ISCAN device");
            return false;
        }
        running = true;
    }
    
    return true;
}


bool ISCANDevice::stopDeviceIO() {
    lock_guard lock(mutex);
    
    if (running) {
        if (!sendCommand(stopCommand)) {
            merror(M_IODEVICE_MESSAGE_DOMAIN, "Cannot stop ISCAN device");
            return false;
        }
        running = false;
    }
    
    return true;
}


bool ISCANDevice::sendCommand(std::uint8_t cmd) {
    if (-1 == ::write(fd, &cmd, 1)) {
        merror(M_IODEVICE_MESSAGE_DOMAIN, "Write to ISCAN device failed: %s", strerror(errno));
        return false;
    }
    return true;
}


void ISCANDevice::receiveData() {
    boost::endian::little_uint16_t word = 0;
    std::size_t bytesRead = 0;
    int currentOutputNumber = -1;
    
    while (continueReceivingData.test_and_set()) {
        ssize_t result;
        int errnoCopy;
        {
            lock_guard lock(mutex);
            result = ::read(fd, reinterpret_cast<std::uint8_t *>(&word) + bytesRead, sizeof(word) - bytesRead);
            errnoCopy = errno;
        }
        
        if (-1 == result) {
            
            merror(M_IODEVICE_MESSAGE_DOMAIN, "Read from ISCAN device failed: %s", strerror(errnoCopy));
            
            mprintf(M_IODEVICE_MESSAGE_DOMAIN, "Attempting to reconnect to ISCAN device...");
            disconnect();
            if (!connect()) {
                merror(M_IODEVICE_MESSAGE_DOMAIN, "Cannot reconnect to ISCAN device");
                return;
            }
            
        } else if (result > 0) {
            
            bytesRead += result;
            
            if (bytesRead == sizeof(word)) {
                if (0x4444 == word) {
                    // Header received.  Restart output counter.
                    currentOutputNumber = 0;
                } else if (currentOutputNumber >= 0) {
                    if (currentOutputNumber < outputs.size()) {
                        if (auto &output = outputs.at(currentOutputNumber)) {
                            output->setValue(word / 10.0);
                        }
                    }
                    currentOutputNumber++;
                }
                
                word = 0;
                bytesRead = 0;
            }
            
        }
    }
}


END_NAMESPACE_MW




























