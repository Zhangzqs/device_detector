#include <ros/ros.h>
#include <serial/serial.h>
#include "device_detector/SerialCommunicator.h"
#include <set>
serial::Serial ser; //串口对象
serial::Timeout timeout = serial::Timeout::simpleTimeout(1000); //设置串口超时时间1s
SerialCommunicator sc(ser);

ros::NodeHandle *pnh;

void requestAddr() {
    CmdDataFrame frame{0xA7};
    sc.sendFrame(frame);
}

uint8_t getAddr(){
    int count = 0;
    CmdDataFrame frame;
    do{
        do{
            requestAddr();
            count++;
            if(count > 1000){
                return 0xff;
            }
        } while (!sc.receivable());
        frame = sc.receive();
    } while (frame.cmd != 0x7A);
    return frame.data[0];
}

bool initSerial(const std::string& serial_port,uint32_t baud_rate){

    ROS_INFO("Try Serial port: %s", serial_port.c_str());
    ROS_INFO("Serial baud rate: %d", baud_rate);

    ser.setPort(serial_port);   //设置串口端口
    ser.setBaudrate(baud_rate); //设置串口波特率
    ser.setTimeout(timeout);    //设置串口超时时间

    try {
        ser.open();
    } catch (serial::IOException &e) {
        ROS_ERROR("Unable to open port %s", serial_port.c_str());
        return false;
    }
    if (!ser.isOpen()) {
        ser.close();
        return false;
    }
    ROS_INFO("Serial port initialized");
    return true;
}


std::set<std::string> detectedPorts;

void detectedCallback(const std::string& port,uint8_t addr){
    detectedPorts.erase(port);
    ROS_INFO("Detected serial device: %s,addr: %d",port.c_str(),addr);
    switch(addr){
        case 0x00:{
            std::string chassis_name;
            pnh->getParam("chassis_name",chassis_name);
            std::stringstream ss;
            ss<<"/"<<chassis_name<<"/";

            pnh->setParam( ss.str() + "serial_port",port);
            pnh->setParam(ss.str() + "baud_rate",115200);
            break;
        }
        case 0x01:{
            std::string chassis_name;
            pnh->getParam("robotic_arm_name",chassis_name);
            std::stringstream ss;
            ss<<"/"<<chassis_name<<"/";

            pnh->setParam(ss.str()+"serial_port",port);
            pnh->setParam(ss.str()+"baud_rate",115200);
            break;
        }
        default:
            break;
    }
}

/**
 * 打开串口
 */
void openPrivatePort() {
    auto ports = serial::list_ports();
    for (const auto &port_info:ports) {
        auto port = port_info.port;
        if(port.find("ttyUSB") == std::string::npos){
            continue;   //如果不是ttyUSB就结束此轮循环
        }

        detectedPorts.insert(port);

        bool success = initSerial(port,115200);
        if(success){    //如果串口打开成功
            //尝试获取设备地址
            uint8_t addr = getAddr();
            ROS_INFO("Got addr: %d",addr);
            detectedCallback(port,addr);
            ser.close();
        }
    }
}

void openImuPort(){
    if(detectedPorts.empty()){
        //不存在其他设备
        ROS_WARN("No Imu Device");
        return;
    }
    if(detectedPorts.size() > 1){
        //过多的设备
        ROS_WARN("Too many device that cannot distinct imu");
        return;
    }

    std::string port = *detectedPorts.begin();
    ROS_INFO("Imu port: %s",port.c_str());

    //存在IMU设备
    std::string imu_name;
    pnh->getParam("imu_name",imu_name);
    std::stringstream ss;
    ss<<"/"<<imu_name<<"/";

    pnh->setParam(ss.str()+"serial_port",port);
    pnh->setParam(ss.str()+"baud_rate",115200);

}

int main(int argc,char **argv){
    ros::init(argc,argv,"device_detector_node");
    pnh = new ros::NodeHandle("~");
    openPrivatePort();
    openImuPort();
    ros::spin();
    delete pnh;
    ser.close();
    return 0;
}