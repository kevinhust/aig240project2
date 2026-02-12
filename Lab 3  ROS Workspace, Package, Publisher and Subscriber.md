# Lab 3 : ROS Workspace, Package, Publisher and Subscriber

Seneca Polytechnic
AIG240 Robotics

## Introduction

### ROS Workspace

A workspace is a directory containing ROS packages. Before using ROS, it’s necessary to source your ROS installation workspace in the terminal you plan to work in. This makes ROS’s packages available for you to use in that terminal.

### ROS Package

A package is an organizational unit for your ROS code. If you want to be able to install your code or share it with others, then you’ll need it organized in a package. With packages, you can release your ROS work and allow others to build and use it easily.

For a package to be considered a catkin package it must meet a few requirements:

- The package must contain a catkin compliant `package.xml` file.
  - That `package.xml` file provides meta information about the package.
- The package must contain a `CMakeLists.txt` which uses catkin.
  - If it is a catkin metapackage it must have the relevant boilerplate `CMakeLists.txt` file.
- Each package must have its own directory
  - This means no nested packages nor multiple packages sharing the same directory.

The simplest possible package may have a file structure that looks like:

```go
my_package/
    CMakeLists.txt
    package.xml
```

A single workspace can contain as many packages as you want, each in their own directory. You can also have packages of different build types in one workspace (CMake, Python, etc.). You cannot have nested packages.

Best practice is to have a `src` directory within your workspace, and to create your packages in there. This keeps the top level of the workspace “clean”.

A trivial workspace might look like:

```lua
workspace_directory/        -- WORKSPACE
    src/                   -- SOURCE SPACE
        CMakeLists.txt       -- 'Toplevel' CMake file, provided by catkin
        package_1/
            CMakeLists.txt     -- CMakeLists.txt file for package_1
            package.xml        -- Package manifest for package_1
        ...
        package_n/
            CMakeLists.txt     -- CMakeLists.txt file for package_n
            package.xml        -- Package manifest for package_n
```

## Procedures

### Create a Workspace directory

1. Best practice is to create a new directory for every new workspace. The name doesn’t matter, but it is helpful to have it indicate the purpose of the workspace. Let’s choose the directory name `ros_ws`, for “development workspace”. Open a new terminal and run:

   ```bash
   mkdir -p ~/ros_ws/src
   cd ~/ros_ws/
   catkin_make
   ```

   The `catkin_make` command is a convenience tool for working with catkin workspaces. When running it for the first time in your workspace, it will create a `CMakeLists.txt` link in your `src` directory.

   Another best practice is to put any packages in your workspace into the `src` directory. The above code creates a `src` directory inside `ros_ws`.

2. Additionally, if you look in your current directory you should now have a 'build' and 'devel' directory. Inside the 'devel' directory you can see that there are now several setup files. Sourcing any of these files will overlay this workspace on top of your environment. Before continuing source your new setup.sh file:

   ```bash
   source devel/setup.bash
   ```

3. To make sure your workspace is properly overlayed by the setup script, make sure `ROS_PACKAGE_PATH` environment variable includes the directory you're in.

   ```bash
   echo $ROS_PACKAGE_PATH
   ```

   You should see:

   ```bash
   /home/jetauto/ros_ws/src:/opt/ros/melodic/share
   ```

   And other path(s) if you have them added to your source.

### Create a C++ Package

1. Navigate into `ros_ws/src`, and run the package creation command to create a simple C++ publisher and subscriber:

   ```bash
   cd ~/ros_ws/src
   catkin_create_pkg cpp_pubsub std_msgs roscpp
   ```

   Your terminal will return a message verifying the creation of your package `cpp_pubsub` and all its necessary files and directories.

   `catkin_create_pkg` requires that you give it a `package_name` and optionally a list of dependencies on which that package depends: `catkin_create_pkg <package_name> [depend1] [depend2] [depend3]`

   More details on package creation can be found [here](https://wiki.ros.org/ROS/Tutorials/CreatingPackage).

   #### Write the publisher node

2. Navigate into `ros_ws/src/cpp_pubsub/src`. This is the directory in any CMake package where the source files containing executables belong.

   ```bash
   cd ~/ros_ws/src/cpp_pubsub/src
   ```

3. Download the example talker code by entering the following command:

   ```ruby
   wget -O talker.cpp https://raw.githubusercontent.com/ros/ros_tutorials/kinetic-devel/roscpp_tutorials/talker/talker.cpp
   ```

4. Now there will be a new file named `talker.cpp`. Open the file using your preferred text editor. Alternatively, create a `.cpp` file with the following:

   ```cpp
   #include "ros/ros.h"
   #include "std_msgs/String.h"
   
   #include <sstream>
   
   /**
    * This tutorial demonstrates simple sending of messages over the ROS system.
    */
   int main(int argc, char **argv)
   {
       /**
        * The ros::init() function needs to see argc and argv so that it can perform
        * any ROS arguments and name remapping that were provided at the command line.
        * For programmatic remappings you can use a different version of init() which takes
        * remappings directly, but for most command-line programs, passing argc and argv is
        * the easiest way to do it.  The third argument to init() is the name of the node.
        *
        * You must call one of the versions of ros::init() before using any other
        * part of the ROS system.
        */
       ros::init(argc, argv, "talker");
   
       /**
        * NodeHandle is the main access point to communications with the ROS system.
        * The first NodeHandle constructed will fully initialize this node, and the last
        * NodeHandle destructed will close down the node.
        */
       ros::NodeHandle n;
   
       /**
        * The advertise() function is how you tell ROS that you want to
        * publish on a given topic name. This invokes a call to the ROS
        * master node, which keeps a registry of who is publishing and who
        * is subscribing. After this advertise() call is made, the master
        * node will notify anyone who is trying to subscribe to this topic name,
        * and they will in turn negotiate a peer-to-peer connection with this
        * node.  advertise() returns a Publisher object which allows you to
        * publish messages on that topic through a call to publish().  Once
        * all copies of the returned Publisher object are destroyed, the topic
        * will be automatically unadvertised.
        *
        * The second parameter to advertise() is the size of the message queue
        * used for publishing messages.  If messages are published more quickly
        * than we can send them, the number here specifies how many messages to
        * buffer up before throwing some away.
        */
       ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
   
       ros::Rate loop_rate(10);
   
       /**
        * A count of how many messages we have sent. This is used to create
        * a unique string for each message.
        */
       int count = 0;
       while (ros::ok())
       {
           /**
            * This is a message object. You stuff it with data, and then publish it.
            */
           std_msgs::String msg;
   
           std::stringstream ss;
           ss << "hello world " << count;
           msg.data = ss.str();
   
           ROS_INFO("%s", msg.data.c_str());
   
           /**
            * The publish() function is how you send messages. The parameter
            * is the message object. The type of this object must agree with the type
            * given as a template parameter to the advertise<>() call, as was done
            * in the constructor above.
            */
           chatter_pub.publish(msg);
   
           ros::spinOnce();
   
           loop_rate.sleep();
           ++count;
       }
   
       return 0;
   }
   ```

   #### The Code Explained

   Now, let's break the code down.

   ```cpp
   #include "ros/ros.h"
   ```

   `ros/ros.h` is a convenience include that includes all the headers necessary to use the most common public pieces of the ROS system.

   ```cpp
   #include "std_msgs/String.h"
   ```

   This includes the `std_msgs/String` message, which resides in the `std_msgs` package. This is a header generated automatically from the `String.msg` file in that package. For more information on message definitions, see the [msg](https://wiki.ros.org/msg) page.

   ```css
   ros::init(argc, argv, "talker");
   ```

   Initialize ROS. This allows ROS to do name remapping through the command line -- not important for now. This is also where we specify the name of our node. Node names must be unique in a running system.

   The name used here must be a base name, i.e., it cannot have a / in it.

   ```cpp
   ros::NodeHandle n;
   ```

   Create a handle to this process' node. The first `NodeHandle` created will actually do the initialization of the node, and the last one destructed will cleanup any resources the node was using.

   ```rust
   ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
   ```

   Tell the master that we are going to be publishing a message of type `std_msgs/String` on the topic `chatter`. This lets the master tell any nodes listening on `chatter` that we are going to publish data on that topic. The second argument is the size of our publishing queue. In this case if we are publishing too quickly it will buffer up a maximum of 1000 messages before beginning to throw away old ones.

   `NodeHandle::advertise()` returns a `ros::Publisher` object, which serves two purposes: 1) it contains a `publish()` method that lets you publish messages onto the topic it was created with, and 2) when it goes out of scope, it will automatically unadvertise.

   ```css
   ros::Rate loop_rate(10);
   ```

   A `ros::Rate` object allows you to specify a frequency that you would like to loop at. It will keep track of how long it has been since the last call to `Rate::sleep()`, and sleep for the correct amount of time.

   In this case we tell it we want to run at 10Hz.

   ```cpp
   int count = 0;
   while (ros::ok())
   {
   ```

   By default roscpp will install a SIGINT handler which provides Ctrl-C handling which will cause `ros::ok()` to return false if that happens.

   `ros::ok()` will return false if:

   - a SIGINT is received (Ctrl-C)
   - we have been kicked off the network by another node with the same name
   - `ros::shutdown()` has been called by another part of the application.
   - all `ros::NodeHandles` have been destroyed

   Once `ros::ok()` returns false, all ROS calls will fail.

   ```cpp
   std_msgs::String msg;
   
   std::stringstream ss;
   ss << "hello world " << count;
   msg.data = ss.str();
   ```

   We broadcast a message on ROS using a message-adapted class, generally generated from a msg file. More complicated datatypes are possible, but for now we're going to use the standard `String` message, which has one member: "data".

   ```scss
   chatter_pub.publish(msg);
   ```

   Now we actually broadcast the message to anyone who is connected.

   ```kotlin
   ROS_INFO("%s", msg.data.c_str());
   ```

   `ROS_INFO` and friends are our replacement for `printf/cout`. See the [rosconsole documentation](https://wiki.ros.org/rosconsole) for more information.

   ```css
   ros::spinOnce();
   ```

   Calling `ros::spinOnce()` here is not necessary for this simple program, because we are not receiving any callbacks. However, if you were to add a subscription into this application, and did not have `ros::spinOnce()` here, your callbacks would never get called. So, add it for good measure.

   ```scss
   loop_rate.sleep();
   ```

   Now we use the `ros::Rate` object to sleep for the time remaining to let us hit our 10Hz publish rate.

   Here's the condensed version of what's going on:

   - Initialize the ROS system
   - Advertise that we are going to be publishing std_msgs/String messages on the chatter topic to the master
   - Loop while publishing messages to chatter 10 times a second

   #### Write the subscriber node

   Now we need to write a node to receive the messsages.

5. Return to `ros_ws/src/cpp_pubsub/src` to create the next node. Enter the following code in your terminal to download the subscriber:

   ```ruby
   wget -O listener.cpp https://raw.github.com/ros/ros_tutorials/kinetic-devel/roscpp_tutorials/listener/listener.cpp
   ```

   Check to ensure that these files exist inside the `ros_ws/src/cpp_pubsub/src` folder:

   ```undefined
   talker.cpp  listener.cpp
   ```

6. Open the `listener.cpp` with your text editor. Alternatively, create a `.cpp` file with the following:

   ```cpp
   #include "ros/ros.h"
   #include "std_msgs/String.h"
   
   /**
    * This tutorial demonstrates simple receipt of messages over the ROS system.
    */
   void chatterCallback(const std_msgs::String::ConstPtr& msg)
   {
       ROS_INFO("I heard: [%s]", msg->data.c_str());
   }
   
   int main(int argc, char **argv)
   {
       /**
        * The ros::init() function needs to see argc and argv so that it can perform
        * any ROS arguments and name remapping that were provided at the command line.
        * For programmatic remappings you can use a different version of init() which takes
        * remappings directly, but for most command-line programs, passing argc and argv is
        * the easiest way to do it.  The third argument to init() is the name of the node.
        *
        * You must call one of the versions of ros::init() before using any other
        * part of the ROS system.
        */
       ros::init(argc, argv, "listener");
   
       /**
        * NodeHandle is the main access point to communications with the ROS system.
        * The first NodeHandle constructed will fully initialize this node, and the last
        * NodeHandle destructed will close down the node.
        */
       ros::NodeHandle n;
   
       /**
        * The subscribe() call is how you tell ROS that you want to receive messages
        * on a given topic.  This invokes a call to the ROS
        * master node, which keeps a registry of who is publishing and who
        * is subscribing.  Messages are passed to a callback function, here
        * called chatterCallback.  subscribe() returns a Subscriber object that you
        * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
        * object go out of scope, this callback will automatically be unsubscribed from
        * this topic.
        *
        * The second parameter to the subscribe() function is the size of the message
        * queue.  If messages are arriving faster than they are being processed, this
        * is the number of messages that will be buffered up before beginning to throw
        * away the oldest ones.
        */
       ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
   
       /**
        * ros::spin() will enter a loop, pumping callbacks.  With this version, all
        * callbacks will be called from within this thread (the main one).  ros::spin()
        * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
        */
       ros::spin();
   
       return 0;
   }
   ```

   #### The Code Explained

   Now, let's break it down piece by piece, ignoring some pieces that have already been explained above.

   ```javascript
   void chatterCallback(const std_msgs::String::ConstPtr& msg)
   {
       ROS_INFO("I heard: [%s]", msg->data.c_str());
   }
   ```

   This is the callback function that will get called when a new message has arrived on the `chatter` topic. The message is passed in a `boost shared_ptr`, which means you can store it off if you want, without worrying about it getting deleted underneath you, and without copying the underlying data.

   ```perl
   ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
   ```

   Subscribe to the `chatter` topic with the master. ROS will call the `chatterCallback()` function whenever a new message arrives. The 2nd argument is the queue size, in case we are not able to process messages fast enough. In this case, if the queue reaches 1000 messages, we will start throwing away old messages as new ones arrive.

   `NodeHandle::subscribe()` returns a `ros::Subscriber` object, that you must hold on to until you want to unsubscribe. When the Subscriber object is destructed, it will automatically unsubscribe from the `chatter` topic.

   There are versions of the `NodeHandle::subscribe()` function which allow you to specify a class member function, or even anything callable by a `Boost.Function` object. The [roscpp](https://wiki.ros.org/roscpp/Overview) overview contains more information.

   ```css
   ros::spin();
   ```

   `ros::spin()` enters a loop, calling message callbacks as fast as possible. Don't worry though, if there's nothing for it to do it won't use much CPU. `ros::spin()` will exit once `ros::ok()` returns false, which means `ros::shutdown()` has been called, either by the default Ctrl-C handler, the master telling us to shutdown, or it being called manually.

   Again, here's a condensed version of what's going on:

   - Initialize the ROS system
   - Subscribe to the chatter topic
   - Spin, waiting for messages to arrive
   - When a message arrives, the chatterCallback() function is called

### Build and Run C++ Package

1. Now, go back to the `cpp_pubsub` package and open up `ros_ws/src/cpp_pubsub/CMakeLists.txt` and ensure the following lines are in there and not commented out. **Note:** Some of the functions are currently commented out and some are missing.

   If the folder `cpp_pubsub` does not exist, that means you have not created the C++ package. Go back to the previous section to create the package.

   ```scss
   cmake_minimum_required(VERSION 3.0.2)
   project(cpp_pubsub)
   
   ## Find catkin and any catkin packages
   find_package(catkin REQUIRED COMPONENTS roscpp std_msgs genmsg)
   
   ## Generate added messages and services
   generate_messages(DEPENDENCIES std_msgs)
   
   ## Declare a catkin package
   catkin_package()
   
   ## Build talker and listener
   include_directories(
       include ${catkin_INCLUDE_DIRS}
   )
   
   add_executable(talker src/talker.cpp)
   target_link_libraries(talker ${catkin_LIBRARIES})
   add_dependencies(talker cpp_pubsub_generate_messages_cpp)
   
   add_executable(listener src/listener.cpp)
   target_link_libraries(listener ${catkin_LIBRARIES})
   add_dependencies(listener cpp_pubsub_generate_messages_cpp)
   ```

   This will create two executables, `talker` and `listener`, which by default will go into the package directory of your `devel` space, located by default at `~/ros_ws/devel/lib/<package name>`.

   Note that you have to add dependencies for the executable targets to message the generation targets:

   ```scss
   add_dependencies(talker cpp_pubsub_generate_messages_cpp)
   ```

   This makes sure message headers of this package are generated before being used. If you use messages from other packages inside your catkin workspace, you need to add dependencies to their respective generation targets as well, because catkin builds all projects in parallel. The following variable to allow you to depend on all necessary targets:

   ```scss
   target_link_libraries(talker ${catkin_LIBRARIES})
   ```

   You can invoke executables directly or you can use `rosrun` to invoke them. They are not placed in `<prefix>/bin` because that would pollute the PATH when installing your package to the system. If you wish for your executable to be on the PATH at installation time, you can setup an install target, see: [catkin/CMakeLists.txt](https://wiki.ros.org/catkin/CMakeLists.txt)

2. Before building the package, it's always a good idea to check and see if all dependencies are met.

   ```css
   cd ~/ros_ws
   rosdep install -i --from-path src --rosdistro melodic -y
   ```

   You should get a success return:

   ```bash
   #All required rosdeps installed successfully
   ```

3. Now run `catkin_make` in your catkin workspace:

   ```bash
   cd ~/ros_ws
   catkin_make
   ```

   Note: Or if you're adding as new pkg, you may need to tell catkin to force making by `--force-cmake` option.

   Now you have written a simple publisher and subscriber.

4. Run ROS master.

   ```undefined
   roscore
   ```

5. In a new terminal, source the setup files:

   ```bash
   cd ~/ros_ws
   source ./devel/setup.bash
   ```

6. Now run the talker node from `ros_ws`:

   ```undefined
   rosrun cpp_pubsub talker
   ```

   The terminal should start publishing info messages every 0.5 seconds, like so:

   ```less
   [ INFO] [1727906572.247429209]: hello world 0
   [ INFO] [1727906572.347872260]: hello world 1
   [ INFO] [1727906572.448580826]: hello world 2
   [ INFO] [1727906572.548227290]: hello world 3
   [ INFO] [1727906572.650658485]: hello world 4
   ```

7. Open another terminal, source the setup files from inside `ros_ws` again, and then start the listener node:

   ```bash
   cd ~/ros_ws
   . devel/setup.bash
   rosrun cpp_pubsub listener
   ```

   The listener will start printing messages to the console, starting at whatever message count the publisher is on at that time, like so:

   ```less
   [ INFO] [1727906765.424901599]: I heard: [hello world 10]
   [ INFO] [1727906765.525323426]: I heard: [hello world 11]
   [ INFO] [1727906765.625240241]: I heard: [hello world 12]
   [ INFO] [1727906765.728040103]: I heard: [hello world 13]
   [ INFO] [1727906765.824066051]: I heard: [hello world 14]
   ```

8. Enter Ctrl+C in each terminal to stop the nodes from spinning.

### Create a Python Package

1. Navigate into `ros_ws/src`, and run the package creation command to create a simple Python publisher and subscriber:

   ```bash
   cd ~/ros_ws/src
   catkin_create_pkg py_pubsub std_msgs rospy
   ```

   Your terminal will return a message verifying the creation of your package `py_pubsub` and all its necessary files and directories.

   #### Write the publisher node

2. Navigate to your package `ros_ws/src/py_pubsub` and let's first create a `scripts` directories to store our Python scripts in and navigate into it:

   ```bash
   cd ~/ros_ws/src/py_pubsub
   ```

   Create a directory.

   ```bash
   mkdir scripts
   cd scripts
   ```

3. Download the example talker code and make it executable by entering the following command:

   ```bash
   wget https://raw.github.com/ros/ros_tutorials/kinetic-devel/rospy_tutorials/001_talker_listener/talker.py
   chmod +x talker.py
   ```

4. Now there will be a new file named `talker.py`. Open the file using your preferred text editor. Alternatively, create a .py file with the following:

   ```python
   #!/usr/bin/env python
   # license removed for brevity
   import rospy
   from std_msgs.msg import String
   
   def talker():
       pub = rospy.Publisher('chatter', String, queue_size=10)
       rospy.init_node('talker', anonymous=True)
       rate = rospy.Rate(10) # 10hz
       while not rospy.is_shutdown():
           hello_str = "hello world %s" % rospy.get_time()
           rospy.loginfo(hello_str)
           pub.publish(hello_str)
           rate.sleep()
   
   if __name__ == '__main__':
       try:
           talker()
       except rospy.ROSInterruptException:
           pass
   ```

   #### The Code Explained

   Now, let's break the code down.

   ```shell
   #!/usr/bin/env python
   ```

   Every Python ROS Node will have this declaration at the top. The first line makes sure your script is executed as a Python script.

   ```javascript
   import rospy
   from std_msgs.msg import String
   ```

   You need to import `rospy` if you are writing a ROS Node. The `std_msgs.msg` import is so that we can reuse the `std_msgs/String` message type (a simple string container) for publishing.

   ```rust
   pub = rospy.Publisher('chatter', String, queue_size=10)
   rospy.init_node('talker', anonymous=True)
   ```

   This section of code defines the talker's interface to the rest of ROS.

   `pub = rospy.Publisher("chatter", String, queue_size=10)` declares that your node is publishing to the `chatter` topic using the message type `String`. `String` here is actually the class `std_msgs.msg.String`. The `queue_size` argument limits the amount of queued messages if any subscriber is not receiving them fast enough.

   The next line, `rospy.init_node(NAME, ...)`, is very important as it tells `rospy` the name of your node -- until `rospy` has this information, it cannot start communicating with the ROS Master. In this case, your node will take on the name `talker`. NOTE: the name must be a base name, i.e. it cannot contain any slashes "/".

   `anonymous = True` ensures that your node has a unique name by adding random numbers to the end of NAME. Refer to [Initialization and Shutdown - Initializing your ROS Node](https://wiki.ros.org/rospy/Overview/Initialization and Shutdown#Initializing_your_ROS_Node) in the `rospy` documentation for more information about node initialization options.

   ```ini
   rate = rospy.Rate(10) # 10hz
   ```

   This line creates a `Rate` object `rate`. With the help of its method `sleep()`, it offers a convenient way for looping at the desired `rate`. With its argument of `10`, we should expect to go through the loop 10 times per second (as long as our processing time does not exceed 1/10th of a second!)

   ```scss
   while not rospy.is_shutdown():
       hello_str = "hello world %s" % rospy.get_time()
       rospy.loginfo(hello_str)
       pub.publish(hello_str)
       rate.sleep()
   ```

   This loop is a fairly standard `rospy` construct: checking the `rospy.is_shutdown()` flag and then doing work. You have to check `is_shutdown()` to check if your program should exit (e.g. if there is a Ctrl-C or otherwise). In this case, the "work" is a call to `pub.publish(hello_str)` that publishes a string to our `chatter` topic. The loop calls `rate.sleep()`, which sleeps just long enough to maintain the desired rate through the loop.

   (You may also run across `rospy.sleep()` which is similar to `time.sleep()` except that it works with simulated time as well (see [Clock](https://wiki.ros.org/Clock)).)

   This loop also calls `rospy.loginfo(str)`, which performs triple-duty: the messages get printed to screen, it gets written to the Node's log file, and it gets written to `rosout`. `rosout` is a handy tool for debugging: you can pull up messages using `rqt_console` instead of having to find the console window with your Node's output.

   `std_msgs.msg.String` is a very simple message type, so you may be wondering what it looks like to publish more complicated types. The general rule of thumb is that *constructor args are in the same order as in the .msg file*. You can also pass in no arguments and initialize the fields directly, e.g.

   ```ini
   msg = String()
   msg.data = str
   ```

   or you can initialize some of the fields and leave the rest with default values:

   ```scss
   String(data=str)
   ```

   You may be wondering about the last little bit:

   ```python
   try:
       talker()
   except rospy.ROSInterruptException:
       pass
   ```

   In addition to the standard Python `__main__` check, this catches a `rospy.ROSInterruptException` exception, which can be thrown by `rospy.sleep()` and `rospy.Rate.sleep()` methods when Ctrl-C is pressed or your Node is otherwise shutdown. The reason this exception is raised is so that you don't accidentally continue executing code after the `sleep()`.

   #### Write the subscriber node

   Now we need to write a node to receive the messages.

5. Return to `ros_ws/src/py_pubsub/scripts` to create the next node. Enter the following code in your terminal:

   ```bash
   wget https://raw.github.com/ros/ros_tutorials/kinetic-devel/rospy_tutorials/001_talker_listener/listener.py
   chmod +x listener.py
   ```

   Now the directory should have these files:

   ```undefined
   listener.py  talker.py
   ```

6. Open the `listener.py` with your text editor. Alternatively, create a .py file with the following:

   ```python
   #!/usr/bin/env python
   import rospy
   from std_msgs.msg import String
   
   def callback(data):
       rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
   
   def listener():
   
       # In ROS, nodes are uniquely named. If two nodes with the same
       # name are launched, the previous one is kicked off. The
       # anonymous=True flag means that rospy will choose a unique
       # name for our 'listener' node so that multiple listeners can
       # run simultaneously.
       rospy.init_node('listener', anonymous=True)
   
       rospy.Subscriber("chatter", String, callback)
   
       # spin() simply keeps python from exiting until this node is stopped
       rospy.spin()
   
   if __name__ == '__main__':
       listener()
   ```

7. Add the following to your `ros_ws/src/py_pubsub/CMakeLists.txt` in the `py_pubsub` package. This makes sure the python scripts get installed properly, and uses the right python interpreter. You may just add this to the end of the `CMakeLists.txt`.

   ```bash
   catkin_install_python(PROGRAMS scripts/talker.py scripts/listener.py
       DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
   )
   ```

   #### The Code Explained

   The code for `listener.py` is similar to `talker.py`, except we've introduced a new callback-based mechanism for subscribing to messages.

   ```python
   rospy.init_node('listener', anonymous=True)
   
   rospy.Subscriber("chatter", String, callback)
   
   # spin() simply keeps python from exiting until this node is stopped
   rospy.spin()
   ```

   This declares that your node subscribes to the `chatter` topic which is of type `std_msgs.msgs.String`. When new messages are received, `callback` is invoked with the message as the first argument.

   We also changed up the call to `rospy.init_node()` somewhat. We've added the `anonymous=True` keyword argument. ROS requires that each node have a unique name. If a node with the same name comes up, it bumps the previous one. This is so that malfunctioning nodes can easily be kicked off the network. The `anonymous=True` flag tells `rospy` to generate a unique name for the node so that you can have multiple `listener.py` nodes run easily.

   The final addition, `rospy.spin()` simply keeps your node from exiting until the node has been shutdown. Unlike `roscpp`, `rospy.spin()` does not affect the subscriber callback functions, as those have their own threads.

### Build and run Python Package

We use CMake as our build system and, yes, you have to use it even for Python nodes. This is to make sure that the autogenerated Python code for messages and services is created.

1. Before building the package, it's always a good idea to check and see if all dependencies are met.

   ```css
   cd ~/ros_ws
   rosdep install -i --from-path src --rosdistro melodic -y
   ```

   You should get a success return:

   ```bash
   #All required rosdeps installed successfully
   ```

2. Go to your catkin workspace and run `catkin_make`:

   ```bash
   cd ~/ros_ws
   catkin_make
   ```

3. Run ROS master.

   ```undefined
   roscore
   ```

4. Open a new terminal, navigate to `ros_ws`, and source the setup files:

   ```bash
   cd ~/ros_ws
   source ./devel/setup.bash
   ```

5. Now run the talker node:

   ```undefined
   rosrun py_pubsub talker.py
   ```

   The terminal should start publishing info messages every 0.5 seconds, like so:

   ```less
   [INFO] [1727910097.196673]: hello world 1727910097.2
   [INFO] [1727910097.293260]: hello world 1727910097.29
   [INFO] [1727910097.394363]: hello world 1727910097.39
   [INFO] [1727910097.493223]: hello world 1727910097.49
   [INFO] [1727910097.592866]: hello world 1727910097.59
   ```

6. Open another terminal, source the setup files from inside `ros_ws` again, and then start the listener node:

   ```bash
   cd ~/ros_ws
   . devel/setup.bash
   rosrun py_pubsub listener.py
   ```

   The listener will start printing messages to the console, starting at whatever message count the publisher is on at that time, like so:

   ```less
   [INFO] [1727910166.665430]: /listener_5173_1727910161732I heard hello world 1727910166.66
   [INFO] [1727910166.761804]: /listener_5173_1727910161732I heard hello world 1727910166.76
   [INFO] [1727910166.870095]: /listener_5173_1727910161732I heard hello world 1727910166.87
   [INFO] [1727910166.963474]: /listener_5173_1727910161732I heard hello world 1727910166.96
   [INFO] [1727910167.071514]: /listener_5173_1727910161732I heard hello world 1727910167.07
   ```

7. Stop the listener and try to run the C++ listener from earlier:

   ```undefined
   rosrun cpp_pubsub listener
   ```

   You should see a similar output.

8. Press `Ctrl+C` in each terminal to stop the nodes from spinning.

## Lab Exercise ([Project 1](https://seneca-bsa.github.io/bsa/aig240/project1/))

1. Write a new controller (C++ or Python) for turtlesim that replaces `turtle_teleop_key`. Since the turtlesim node is the subscriber in this example, you’ll only need to write a single publisher node.

   Create a new package called `lab3_turtlesim`. You can create a new workspace called `lab3_ws` or use your existing workspace.

   The command to create the packages are given below depending on your preferred programming language. You'll need the `geometry_msgs` dependency to use the `twist` object.

   ```undefined
   catkin_create_pkg lab3_turtlesim roscpp geometry_msgs
   ```

   or

   ```undefined
   catkin_create_pkg lab3_turtlesim rospy geometry_msgs
   ```

   Your node should do the following:

   - Accept a command line argument specifying the name of the turtle it should control.
     - i.e., running `rosrun lab3_turtlesim turtle_controller turtle1` will start a controller node that controls `turtle1`.
   - Use `w`, `a`, `s`, `d` (and `q`, `e`, `c`, and `z`) to control the turtle by publish velocity control messages on the appropriate topic whenever the user presses those keys on the keyboard, as in the original `turtle_teleop_key`. **The turtle should ONLY move when is key is pressed. When the key is released, the turtle should STOP moving.**
     - Option 1 (Easy) Single keypress: In addition to just forward/backward and turns, the turtle should move forward and turn left in a circular path if `q` is pressed and similar for `e`, `c`, and `z` in their corresponding direction.
     - Option 2 (Hard) Multiple keypress: If you want to challenge your Python skills, make the controller so it listen to multiple keys. i.e., if `w + a` are pressed, the turtle should move forward and turn left in a circular path. If the keys pressed are contracdicting, there should be no movement. You'll need to install additional Python package to achieve this.

   **Hint:** You'll need to use the `Twist` message type in the `geometry_msgs` package.

   **Hint:** Find the `turtle_teleop_key` source code as a reference. [teleop_turtle_key.cpp](https://docs.ros.org/en/kinetic/api/turtlesim/html/teleop__turtle__key_8cpp_source.html)

   To test, spawn multiple turtles and open multiple instances of your new turtle controller node, each linked to a different turtle.