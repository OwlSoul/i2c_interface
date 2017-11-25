

This node provides a simple interface for I2C devices. While there is no direct advantage over writing your own node incorporating I2C, this node still may be handy in several situations.

Node sets up topic branch and service branch for each I2C device specified, and gives you a possibility to interract with I2C devices either via topics or services.

Please, note that I2C protocol is master driven, though reading via topics is limited. You may set up periodic polling of simple I2C device by specifying poll frequency and number of bytes to read in each poll. Additionally, you have an option to publish all responses from services to corresponding topics, which will give you an option to record your communication with I2C using rosbag.

Only the I2C Master mode is supported for this node, since many single board computers don't provide I2C slave option.

Tested on Orange PI Lite, Orange PI Zero and Raspberry PI 2.

In general, this thingamajig is most suitable for testing and learning purposes.
