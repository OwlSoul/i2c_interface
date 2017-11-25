/** @mainpage ROS I2C interface
  * This node provides a simple interface for I2C devices.
  * While there is no direct advantage over writing your own node incorporating
  * I2C, this node still may be handy in several situations.
  *
  * Node sets up a topic branch and service branch for each I2C device specified,
  * and gives you a possibility to interract with I2C devices either via topics
  * or services.
  *
  * Please, note that I2C protocol is master driven, though reading via topics
  *  is limited. You may set up periodic polling of simple I2C device by specifying
  * poll frequency and number of bytes to read in each poll. Additionally, you have an
  * option to publish all responces from services to correspoding topics, which will
  * give you an option to record your communication with I2C using rosbag.
  *
  * Only the I2C Master mode is supported for this node, since many single board
  * computers don't provide I2C slave option.
  *
  * Tested on Orange PI Lite, Orange PI Zero and Raspberry PI 2.
  *
  * In general, this thingamagick is most suitable for testing and learning purposes.
  */

//-----                             INCLUDES                             -----//
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sstream>
#include <string>
#include <atomic>
#include <thread>
#include <mutex>
#include <time.h>

#include <XmlRpcValue.h>

#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/UInt8MultiArray.h"
#include "i2c_interface/DeviceReadString.h"
#include "i2c_interface/DeviceWriteString.h"
#include "i2c_interface/DeviceReadBytes.h"
#include "i2c_interface/DeviceWriteBytes.h"
#include "i2c_interface/DeviceSetPoll.h"
#include "i2c_interface/DeviceStopPoll.h"

//----------------------------------------------------------------------------//

using namespace std;

string tag_string = "I2C INTERFACE NODE:";
#define TAG tag_string.c_str()

#define I2C_ERROR_FILE_OPEN -3
#define I2C_ERROR_SET_SLAVE -2
#define I2C_ERROR_IO        -1

#define TOPIC_QUEUE_SIZE 1000


/// Default node name, usually will be overwritten by launch file settings.
string nodeName            = "i2c_interface_node";

/// Default device name, only one device should be used per node.
string i2cDeviceName       = "/dev/i2c-0";

/// Keep I2C device opened while node is running.
bool keepOpened   = true;

/// File descriptor. Since I2C device is global the node, this can be global too.
int  file;

/// Mutex for preventing multithread issues.
std::mutex deviceMutex;

//----------------------------------------------------------------------------//

/** @brief Helper function to read string from I2C device
  *
  * This is a helper function to read string data from I2C device.
  * @param[in]  slaveAddr   I2C device address.
  * @param[in]  bytesToRead How many bytes to read from device.
  * @param[out] data        String received from I2C device.
  */
int readStringRoutine(int slaveAddr, int bytesToRead , string &data){
  int r;

  deviceMutex.lock();

  if (!keepOpened) {
    file = open(i2cDeviceName.c_str(), O_RDWR);
    if (file < 0) {
      printf("%s %s %s\n", TAG, "Error accessing I2C device.", i2cDeviceName.c_str());
      deviceMutex.unlock();
      return I2C_ERROR_FILE_OPEN;
    }
  }

  // Setting I2C slave to receive the message
  if (r = ioctl(file, I2C_SLAVE, slaveAddr) < 0) {
    printf("%s %s\n", TAG, "Setting I2C address failed.");
    if (!keepOpened) close(file);
    deviceMutex.unlock();
    return I2C_ERROR_SET_SLAVE;
  }

  char* buf = (char*) malloc((bytesToRead+1)*sizeof(char));

  r = read(file, buf, bytesToRead);

  if (r < 0) {
    if (!keepOpened) close(file);
    deviceMutex.unlock();
    return I2C_ERROR_IO;
  }
  else
  {

      buf[r + 1] = 0; //Adding zero to the end of string
      data = buf;
  }

  free(buf);
  if (!keepOpened) close(file);
  deviceMutex.unlock();

  return r;
}

/** @brief Helper function to write string to I2C device
  *
  * This is a helper function to write string data to I2C device.
  * @param[in]  slaveAddr   I2C device address.
  * @param[in]  data        String to be sent to I2C device.
  */
int writeStringRoutine(int slaveAddr, string &data) {
  int r;

  deviceMutex.lock();

  if (!keepOpened) {
    file = open(i2cDeviceName.c_str(), O_RDWR);
    if (file < 0) {
      printf("%s %s %s\n", TAG, "Error accessing I2C device.", i2cDeviceName.c_str());
      deviceMutex.unlock();
      return I2C_ERROR_FILE_OPEN;
    }
  }

  // Setting I2C slave to receive the message
  if (r = ioctl(file, I2C_SLAVE, slaveAddr) < 0) {
    printf("%s %s\n", TAG, "Setting I2C address failed.");
    if (!keepOpened) close(file);
    deviceMutex.unlock();
    return I2C_ERROR_SET_SLAVE;
  }

  r = write(file, data.c_str(), data.length());

  if (r < 0) {
    if (!keepOpened) close(file);
    deviceMutex.unlock();
    return I2C_ERROR_IO;
  }

  if (!keepOpened) close(file);

  deviceMutex.unlock();
  return r;
}

/** @brief Helper function to read bytes from I2C device
  *
  * This is a helper function to read bytes from I2C device.
  * @param[in]  slaveAddr   I2C device address.
  * @param[in]  bytesToRead How many bytes to read from device.
  * @param[out] data        Bytes received from I2C device.
  */
int readBytesRoutine(int slaveAddr, int bytesToRead , std::vector<unsigned char> &data) {
  int r;

  deviceMutex.lock();

  if (!keepOpened) {
    file = open(i2cDeviceName.c_str(), O_RDWR);
    if (file < 0) {
      printf("%s %s %s\n", TAG, "Error accessing I2C device.", i2cDeviceName.c_str());
      deviceMutex.unlock();
      return I2C_ERROR_FILE_OPEN;
    }
  }

  // Setting I2C slave to receive the message
  if (r = ioctl(file, I2C_SLAVE, slaveAddr) < 0) {
    printf("%s %s\n", TAG, "Setting I2C address failed.");
    if (!keepOpened) close(file);
    deviceMutex.unlock();
    return I2C_ERROR_SET_SLAVE;
  }

  uint8_t* buf = (uint8_t*) malloc((bytesToRead)*sizeof(uint8_t));

  r = read(file, buf, bytesToRead);

  if (r < 0) {
    if (!keepOpened) close(file);
    deviceMutex.unlock();
    return I2C_ERROR_IO;
  }
  else
  {
      data = std::vector<unsigned char>(buf, buf + bytesToRead);
  }

  free(buf);
  if (!keepOpened) close(file);
  deviceMutex.unlock();;
  return r;
}

/** @brief Helper function to write bytes to I2C device
  *
  * This is a helper function to write bytes to I2C device.
  * @param[in]  slaveAddr   I2C device address.
  * @param[in]  data        Bytes to be sent to I2C device.
  */
int writeBytesRoutine(int slaveAddr, std::vector<unsigned char> &data) {
  int r;

  deviceMutex.lock();

  if (!keepOpened) {
    file = open(i2cDeviceName.c_str(), O_RDWR);
    if (file < 0) {
      printf("%s %s %s\n", TAG, "Error accessing I2C device.", i2cDeviceName.c_str());
      deviceMutex.unlock();
      return I2C_ERROR_FILE_OPEN;
    }
  }

  // Setting I2C slave to receive the message
  if (r = ioctl(file, I2C_SLAVE, slaveAddr) < 0) {
    printf("%s %s\n", TAG, "Setting I2C address failed.");
    if (!keepOpened) close(file);
    deviceMutex.unlock();
    return I2C_ERROR_SET_SLAVE;
  }

  r = write(file, &data[0], data.size());

  if (r < 0) {
    if (!keepOpened) close(file);
    deviceMutex.unlock();
    return I2C_ERROR_IO;
  }

  if (!keepOpened) close(file);
  deviceMutex.unlock();
  return r;
}

//------------------------------ Devices class -------------------------------//
/** @brief I2C device class.
  *
  * Each device will have an instance of this class to provide its own
  * service and topic branches in ROS.
  */
class i2cDevice{
private:
  /// Polling parameters for thread creation.
  struct polling_params{
    /*@{*/
    int interval;           /**< polling interval */
    int bytesRead;          /**< bytes to read in each poll interval */
    /*@}*/
  };
  ros::NodeHandle mNodeHandle;       /// ROS node handle.

  bool isPolling;                    /// Flag to indicate if polling is enabled.
  std::thread pollingThread;         /// Thread handler.

  //                                PARAMETERS                                //
  int    address;                    /// I2C device address
  string name;                       /// I2C device name to be shown in ROS

  bool   services;                   /// If true, services will be enabled for this device
  int    request_bytes;              /// How many bytes to request in each poll
  int    request_freq;               /// Polling frequency

  string recv_topic_b;               /// Topic to publish data received from I2C device (bytes)
  string recv_topic_s;               /// Topic to publish data received from I2C device (string)
  string send_topic_b;               /// Topic for sending data to I2C device (bytes)
  string send_topic_s;               /// Topic for sending data to I2C device (string)

  bool subscribeString;              /// Will be set true if recv_topic_b is not empty
  bool subscribeBytes;               /// Will be set true if recv_topic_s is not empty
  bool publishString;                /// Will be set true if recv_topic_b is not empty
  bool publishBytes;                 /// Will be set true if recv_topic_s is not empty

  //                               SERVICES                                   //
  /// These variables exist only to provide scope, otherwise services won't work.
  ros::ServiceServer S1;
  ros::ServiceServer S2;
  ros::ServiceServer S3;
  ros::ServiceServer S4;
  ros::ServiceServer S5;
  ros::ServiceServer S6;

  //                       PUBLISHERS AND SUBSCRIBERS                         //
  ros::Publisher  mRecvStringPublisher; /// Publish string received from I2C device
  ros::Publisher  mRecvBytesPublisher;  /// Publish bytes received from I2C device

  ros::Publisher  mSendStringPublisher; /// Publish string sent to I2C device via service
  ros::Publisher  mSendBytesPublisher;  /// Publish bytes sent to I2C device via service
  ros::Subscriber mSendStringSubscriber;/// Subscriber for sending string to I2C device
  ros::Subscriber mSendBytesSubscriber; /// Subscriber for sending bytes to I2C device

public:
  /** Class conctructor
    *  @param[in] pNodeHandle    ROS node handle
    *  @param[in] pAddress       I2C device address
    *  @param[in] pName          I2C device name
    *  @param[in] pServices      Provide services for this device if true
    *  @param[in] pRequest_bytes How many bytes to request in each poll
    *  @param[in] pRequest_freq  Poll frequency
    *  @param[in] pRecv_topic_b  Topic to publish data received from I2C device (bytes)
    *  @param[in] pRecv_topic_s  Topic to publish data received from I2C device (string)
    *  @param[in] pSend_topic_b  Topic for sending data to I2C device (bytes)
    *  @param[in] pSend_topic_s  Topic for sending data to I2C device (string)
    *
    */
  i2cDevice(ros::NodeHandle pNodeHandle,
            int    pAddress,
            string pName,
            bool   pServices,
            int    pRequest_bytes,
            int    pRequest_freq,
            string pRecv_topic_b,
            string pRecv_topic_s,
            string pSend_topic_b,
            string pSend_topic_s) {
    mNodeHandle = pNodeHandle;
    address       = pAddress;
    name          = pName;
    services      = pServices;
    request_bytes = pRequest_bytes;
    request_freq  = pRequest_freq;
    recv_topic_b  = pRecv_topic_b;
    recv_topic_s  = pRecv_topic_s;
    send_topic_b  = pSend_topic_b;
    send_topic_s  = pSend_topic_s;

    // Create services
    if (services) {
      S1 = mNodeHandle.advertiseService(getName()+"/write_bytes" , &i2cDevice::devWriteBytes, this);
      S2 = mNodeHandle.advertiseService(getName()+"/read_bytes" , &i2cDevice::devReadBytes, this);
      S3 = mNodeHandle.advertiseService(getName()+"/write_string" , &i2cDevice::devWriteString, this);
      S4 = mNodeHandle.advertiseService(getName()+"/read_string" , &i2cDevice::devReadString, this);
      S5 = mNodeHandle.advertiseService(getName()+"/set_poll", &i2cDevice::devSetPoll, this);
      S6 = mNodeHandle.advertiseService(getName()+"/stop_poll", &i2cDevice::devStopPoll, this);
    }

    // Create publishers
    if (recv_topic_s.length() > 0) {
      mRecvStringPublisher = mNodeHandle.advertise<std_msgs::String>(recv_topic_s, TOPIC_QUEUE_SIZE);
    }

    if (send_topic_s.length() > 0) {
      mSendStringSubscriber = mNodeHandle.subscribe(send_topic_s, TOPIC_QUEUE_SIZE, &i2cDevice::sendStringCallback, this);
      mSendStringPublisher  = mNodeHandle.advertise<std_msgs::String>(send_topic_s, TOPIC_QUEUE_SIZE);
    }

    if (recv_topic_b.length() > 0) {
      mRecvBytesPublisher = mNodeHandle.advertise<std_msgs::UInt8MultiArray>(recv_topic_b, TOPIC_QUEUE_SIZE);
    }

    if (send_topic_b.length() > 0) {
      mSendBytesSubscriber = mNodeHandle.subscribe(send_topic_b, TOPIC_QUEUE_SIZE, &i2cDevice::sendBytesCallback, this);
      mSendBytesPublisher  = mNodeHandle.advertise<std_msgs::UInt8MultiArray>(send_topic_b, TOPIC_QUEUE_SIZE);
    }

    publishString   = (recv_topic_s.length() > 0);
    publishBytes    = (recv_topic_b.length() > 0);
    subscribeString = (send_topic_s.length() > 0);
    subscribeBytes  = (send_topic_b.length() > 0);

    isPolling = false;

    // Set polling
    if (request_bytes > 0) {
      setPoll(request_freq, request_bytes);
    }

  }
public:
  /// Get I2C device address
  int    getAddress() { return address; }
  /// Get I2C device name
  string getName()    { return name;    }

public:
  /** @brief Poll function, to be run in separate thread.
   *
   * This function is responsible for constant polling of I2C device.
   * @param polling_params Polling params
   *
   */
   void poll(polling_params params){
    string data = "";

    while (isPolling){
      readStringRoutine(address, params.bytesRead, data);
      struct timespec t;
      t.tv_sec  = (params.interval / 1000);
      t.tv_nsec = (params.interval % 1000)*1000*1000;
      if (publishString){
          std_msgs::String msg;
          msg.data = data;
          mRecvStringPublisher.publish(msg);
      }
      if (publishBytes){
         std_msgs::UInt8MultiArray msg;
         //                                                  Cut the last byte which is \0
         std::vector<uint8_t> v(data.c_str(), data.c_str() + data.length() /*+ 1*/);
         msg.data = v;
         mRecvBytesPublisher.publish(msg);
      }
      nanosleep(&t, &t);
    }
  }

  /** @brief Set polling for this I2C device
   *
   * @param interval   Polling interval.
   * @param bytesRead  How many bytes to request in each poll.
   *
   */
  void setPoll(int interval, int bytesRead) {
    isPolling = true;
    struct polling_params params;
    params.interval  = interval;
    params.bytesRead = bytesRead;
    pollingThread = std::thread(&i2cDevice::poll, this, params);
  }

  /** @brief Stop polling for this I2C device
   *
   */
  void stopPoll(){
    isPolling = false;
    pollingThread.join();
  }

  /** @brief Send string topic callback
   *
   * @param msg   ROS message (std_msgs::String).
   */
  void sendStringCallback(const std_msgs::String::ConstPtr& msg){
      string data = msg->data;
      writeStringRoutine(address, data);
  }

  /** @brief Send bytes topic callback
   *
   * @param msg   ROS message (std_msgs::UInt8MultiArray).
   */
  void sendBytesCallback(const std_msgs::UInt8MultiArray::ConstPtr& msg){
      std::vector<unsigned char> data = (std::vector<unsigned char>)msg->data;
      writeBytesRoutine(address, data);
  }

  /** @brief "set_poll" service callback
   *
   * @param req, res   Service request and responce (i2c_interface::DeviceSetPoll)
   */
  bool devSetPoll(i2c_interface::DeviceSetPoll::Request  &req, i2c_interface::DeviceSetPoll::Response &res) {
    request_bytes = req.bytesToRead;
    request_freq  = req.pollRate;
    if (isPolling) stopPoll();
    setPoll(request_freq, request_bytes);
    return true;
  }

  /** @brief "stop_poll" service callback
   *
   * @param req, res   Service request and responce (i2c_interface::DeviceStopPoll)
   */
  bool devStopPoll(i2c_interface::DeviceStopPoll::Request  &req, i2c_interface::DeviceStopPoll::Response &res) {
    stopPoll();
    return true;
  }

  /** @brief "write_bytes" service callback
   *
   * @param req, res   Service request and responce (i2c_interface::DeviceWriteBytes)
   */
  bool devWriteBytes(i2c_interface::DeviceWriteBytes::Request  &req, i2c_interface::DeviceWriteBytes::Response &res) {
    if ((req.publish) && (subscribeBytes)) {
        std_msgs::UInt8MultiArray msg;
        msg.data = req.data;
        mSendBytesPublisher.publish(msg);
        res.bytesSent = req.data.size();
    }
    else {
        res.bytesSent = writeBytesRoutine(getAddress(), req.data);
    }

    return true;
  }

  /** @brief "read_bytes" service callback
   *
   * @param req, res   Service request and responce (i2c_interface::DeviceReadBytes)
   */
  bool devReadBytes(i2c_interface::DeviceReadBytes::Request  &req, i2c_interface::DeviceReadBytes::Response &res){
      res.bytesReceived = readBytesRoutine(getAddress(), req.bytesToRead, res.data);

      if ((req.publish) && (publishBytes)) {
        std_msgs::UInt8MultiArray msg;
        msg.data = (std::vector<uint8_t>)res.data;
        mRecvBytesPublisher.publish(msg);
      }

      return true;
  }

  /** @brief "write_string" service callback
   *
   * @param req, res   Service request and responce (i2c_interface::DeviceWriteString)
   */
  bool devWriteString(i2c_interface::DeviceWriteString::Request  &req, i2c_interface::DeviceWriteString::Response &res){

      if ((req.publish) && (subscribeString)) {
        std_msgs::String msg;
        msg.data = req.data;
        mSendStringPublisher.publish(msg);
        res.bytesSent = req.data.size();
      }
      else {
          res.bytesSent = writeStringRoutine(getAddress(), req.data);
      }

      return true;
  }

  /** @brief "read_string" service callback
   *
   * @param req, res   Service request and responce (i2c_interface::DeviceReadString)
   */
  bool devReadString(i2c_interface::DeviceReadString::Request  &req, i2c_interface::DeviceReadString::Response &res){
      res.bytesReceived = readStringRoutine(getAddress(), req.bytesToRead, res.data);

      if ((req.publish) && (publishString)){
        std_msgs::String msg;
        msg.data = res.data;
        mRecvStringPublisher.publish(msg);
      }

      return true;
  }
};

std::vector<i2cDevice> i2cDevices;  /// Vector containing all I2C devices

//----------------------------------------------------------------------------//
//----                          MAIN FUNCTION                             ----//
//----------------------------------------------------------------------------//
int main(int argc, char **argv)
{
  ros::Publisher  mGlobalTopicPublisher;
  ros::Subscriber mGlobalTopicSubscriber;

  ros::init(argc, argv, nodeName);

  ros::NodeHandle mNodeHandle("~");

  tag_string = ros::this_node::getName()+":";

  // Getting parameters
  string stringParam;
  int    intParam;
  bool   boolParam;
  // I2C device parameter, which device to use
  if (ros::param::get("~i2c_device", stringParam)) i2cDeviceName = stringParam;

  // Unique Access parameter, will keep device opened if true
  if (ros::param::get("~keep_open", boolParam)) keepOpened = boolParam;

  printf("%s %s %s\n",   TAG, "I2C Device        =", i2cDeviceName.c_str());
  printf("%s %s %s\n",   TAG, "Keep device open  =", keepOpened ? "true" : "false");

  // Testing if I2C device is available
  file = open(i2cDeviceName.c_str(), O_RDWR);

  if (file < 0) {
    ROS_ERROR("%s \"%s\"\n%s", "Error accessing I2C device", i2cDeviceName.c_str(),
              "Check if device exists and access rights (use \"sudo useradd your_user i2c\")") ;
    close(file);
    exit(1);
  }

  if (!keepOpened) close(file);

  // Getting XML RPC list of possible devices from parameter server
  // This hurts a lot.
  printf("\n");
  XmlRpc::XmlRpcValue deviceList;
  if (ros::param::get("~device", deviceList)) {
    printf("%s %s %d\n\n",TAG,"Devices:", deviceList.size());
    std::map<std::string, XmlRpc::XmlRpcValue>::iterator i;
    for (i = deviceList.begin(); i != deviceList.end(); i++){
      // Figuring out device name from HEAVILY UNDOCUMENTED XML RPC mess.
      string deviceName = i->first;
      // Requesting device parameter, from server, to make our life easier.
      // Yeah, it's possible to do that from XML we've got before, but nu-uh,
      // not gonna do that.
      int deviceAddr;
      if (ros::param::get("~/device/"+deviceName+"/address", intParam)) {
        std::stringstream ss;
        std::ostringstream os;
        os << intParam;
        ss << std::hex << os.str();
        ss >> deviceAddr;
      }
      else
      // Getting this param (address) as a string, suppose string is in HEX format.
      if (ros::param::get("~/device/"+deviceName+"/address", stringParam)) {
        std::stringstream ss;

        ss << std::hex << stringParam.c_str();
        ss >> deviceAddr;
      }
      else {
        ROS_INFO("%s %s %s","I2C device",deviceName.c_str(),"has no address set.");
        continue;
      }
      // Getting param "services" - are services enabled?
      bool servicesEnabled = false;
      if (ros::param::get("~/device/"+deviceName+"/services", boolParam)) servicesEnabled = boolParam;
      // Request bytes
      int requestBytes = 0;
      if (ros::param::get("~/device/"+deviceName+"/request/bytes", intParam)) requestBytes = intParam;
      // Request rate
      int requestRate = 0;
      if (ros::param::get("~/device/"+deviceName+"/request/rate", intParam)) requestRate = intParam;
      // Request bytes
      string recvTopicBytes = "";
      if (ros::param::get("~/device/"+deviceName+"/recv_topic/bytes", stringParam)) recvTopicBytes = stringParam;
      string recvTopicString = "";
      if (ros::param::get("~/device/"+deviceName+"/recv_topic/string", stringParam)) recvTopicString = stringParam;
      // Request bytes
      string sendTopicBytes = "";
      if (ros::param::get("~/device/"+deviceName+"/send_topic/bytes", stringParam)) sendTopicBytes = stringParam;
      string sendTopicString = "";
      if (ros::param::get("~/device/"+deviceName+"/send_topic/string", stringParam)) sendTopicString = stringParam;

      printf("%s %s %s\n",  TAG, "Device :",   deviceName.c_str());
      printf("%s %s 0x%X\n",TAG, " Address       :", deviceAddr);
      printf("%s %s %s\n",  TAG, " Services      :", servicesEnabled ? "true" : "false");
      printf("%s %s %s\n",  TAG, " Send topic (b):", sendTopicBytes.c_str());
      printf("%s %s %s\n",  TAG, " Send topic (s):", sendTopicString.c_str());
      printf("%s %s %s\n",  TAG, " Recv topic (b):", recvTopicBytes.c_str());
      printf("%s %s %s\n",  TAG, " Recv topic (s):", recvTopicString.c_str());
      printf("%s %s %d\n",  TAG, " Request bytes :", requestBytes);
      printf("%s %s %d\n",  TAG, " Request rate  :", requestRate);

      if ((sendTopicBytes.compare(sendTopicString) == 0) && (sendTopicBytes.length() > 0)) {
        printf("%s %s\n",  TAG, "WARNING: String and Bytes topics for send are the same!");
      }
      if ((recvTopicBytes.compare(recvTopicString) == 0) && (recvTopicBytes.length() > 0)) {
        printf("%s %s\n",  TAG, "WARNING: String and Bytes topics for recv are the same!");
      }

      printf("\n");

      i2cDevice* dev = new i2cDevice (mNodeHandle,
                                      deviceAddr,
                                      deviceName,
                                      servicesEnabled,
                                      requestBytes,
                                      requestRate,
                                      recvTopicBytes,
                                      recvTopicString,
                                      sendTopicBytes,
                                      sendTopicString);
    }
  }

  // Node is Ret-2-Go!

  printf("%s %s\n",      TAG, "INITIALIZED!");

  ros::spin();

  if (keepOpened) close(file);

  printf("%s %s\n", TAG, "TERMINATED!");

  return 0;
}
//----                              END                                    ----//
