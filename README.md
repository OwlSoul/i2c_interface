**The project is suspended, see the description below.**

This node provides a simple interface for I2C devices. While there is no direct advantage over writing your own node incorporating I2C, this node still may be handy in several situations.

Node sets up topic branch and service branch for each I2C device specified, and gives you a possibility to interract with I2C devices either via topics or services.

Please, note that I2C protocol is master driven, though reading via topics is limited. You may set up periodic polling of simple I2C device by specifying poll frequency and number of bytes to read in each poll. Additionally, you have an option to publish all responses from services to corresponding topics, which will give you an option to record your communication with I2C using rosbag.

Only the I2C Master mode is supported for this node, since many single board computers don't provide I2C slave option.

Tested on Orange PI Lite, Orange PI Zero and Raspberry PI 2.

In general, this thingamajig is most suitable for testing and learning purposes.

----

**UPD:** Since there's an occasional interest for this thing, It's better to clarify a little bit more what is does.

I made this project a while ago, to play with I2C in ROS. The idea was to use it as something like this:

- You take a Raspberry PI or some other PC which has I2C support, connect some device via I2C and then you have topics and services to send data to I2C bus and particular device and read it back (as strings or bytes).

Sounds good, right? 

Unless it isn't. This approach completely violates the ROS ideology (messages and services should be meaningful), and also I2C bus can be shared between multiple nodes, if written correctly. You can simply write a node supporting getting some data from I2C bus, no need to "expose" I2C device via topics/services (again, not the brightest idea).

If you wanted to send ALL ROS messages via I2C - now that's a pretty neat idea for ROS project, yeah. I thought about that when was writing this little thing. The problem is that practically ALL I2C-capable single Board PCs (Raspberry PI, Orange PI, Odroid etc) cannot work as I2C slaves. Master only. And things which can work a I2C slaves (Arduino devices, STM32/NXP microcontrollers or other stuff) don't support ROS. ROS2 may have a big chance for this.

**To sum up:**

A) If you want your ROS node to get a data from some I2C device - just use standard i2c library and implement getting the data from that I2C device in your ROS node.

B) If you want to use I2C as a "medium" between two ROS devices and send all ROS messages via I2C (like ROS I2C Bridge) - my node can't do that.
