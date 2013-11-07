#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "brics_actuator/JointPositions.h"

#include <sstream>

float juntas[5];
bool recibido_juntas;

//Callback para el mensaje JointPositions
void positionsCallback(const brics_actuator::JointPositions::ConstPtr& msg)
{
    ROS_INFO("Mensaje de juntas recibido");
    ROS_INFO("Valores de las juntas:");
    for(int i=0; i<5; i++)
    {
        ROS_INFO("Junta %d - %f", i, msg->positions[i].value);
        juntas[i]=msg->positions[i].value;
    }
    recibido_juntas = true;
}


int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line. For programmatic
   * remappings you can use a different version of init() which takes remappings
   * directly, but for most command-line programs, passing argc and argv is the easiest
   * way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "interface");

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
  ros::Publisher juntas_pub[5];

//  ros::Publisher junta0_pub = n.advertise<std_msgs::Float64>("/vrep/junta0", 1000);
//  ros::Publisher junta1_pub = n.advertise<std_msgs::Float64>("/vrep/junta1", 1000);
//  ros::Publisher junta2_pub = n.advertise<std_msgs::Float64>("/vrep/junta2", 1000);
//  ros::Publisher junta3_pub = n.advertise<std_msgs::Float64>("/vrep/junta3", 1000);
//  ros::Publisher junta4_pub = n.advertise<std_msgs::Float64>("/vrep/junta4", 1000);

  juntas_pub[0] = n.advertise<std_msgs::Float64>("/vrep/junta0", 1000);
  juntas_pub[1] = n.advertise<std_msgs::Float64>("/vrep/junta1", 1000);
  juntas_pub[2] = n.advertise<std_msgs::Float64>("/vrep/junta2", 1000);
  juntas_pub[3] = n.advertise<std_msgs::Float64>("/vrep/junta3", 1000);
  juntas_pub[4] = n.advertise<std_msgs::Float64>("/vrep/junta4", 1000);

  ros::Subscriber sub = n.subscribe("/arm_1/arm_controller/position_command", 1000, positionsCallback);

  ros::Rate loop_rate(10);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  //int count = 0;
  while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
      if(recibido_juntas)
      {
        ROS_INFO("Hay valores a enviar");
        std_msgs::Float64 msg;

        for(int i=0; i<5; i++)
        {
            msg.data = juntas[i];
            ROS_INFO("Junta %d - %f", i, msg.data);
		
	    //EXTRA - Para trasladar mejor al V-Rep basandose en los valores de applications/hello_world_demo/main.cpp
	    if (i==0) msg.data = msg.data - 2.56244;
	if (i==1) msg.data = msg.data - 1.04883;
	if (i==2) msg.data = msg.data + 2.43523;
	if (i==3) msg.data = msg.data - 1.73184;

            juntas_pub[i].publish(msg);

            ros::spinOnce();
        }

        recibido_juntas=false;
      }

    loop_rate.sleep();
    //++count;
    //  ros::spin();
    ros::spinOnce();
  }


  return 0;
}
