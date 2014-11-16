#include <iostream>
#include <string>
#include <sstream>
#include <dirent.h>
#include <termios.h>
// ROS
#include "ros/ros.h"
#include <std_msgs/Char.h>

#define QUEUE_SIZE 1

#define IMG_ROWS 480
#define IMG_COLS 640
#define SIZE     200

int getch();
class Keyboard_Listener{


public:
    Keyboard_Listener(const ros::NodeHandle& n);
    void run();

private:
    ros::NodeHandle n_;

    ros::Publisher char_pub_;

};

// =============================================================================
// =============================================================================

int main(int argc, char* argv[])
{
    // ** Init node
    ros::init(argc, argv, "vision_training2D");
    ros::NodeHandle n;

    // ** Create object recognition object
    Keyboard_Listener k(n);

    k.run();

    return 0;
}


Keyboard_Listener::Keyboard_Listener(const ros::NodeHandle& n)
    : n_(n)
{
    // ** Publisher
    char_pub_ = n_.advertise<std_msgs::Char>("/keyboard_listener/char",5);
}

void Keyboard_Listener::run()
{
    while(ros::ok())
    {
        char c = getch();
        std_msgs::Char msg;
        msg.data = c;
        char_pub_.publish(msg);
        ros::spinOnce();
    }
}


int getch()
{
//    http://answers.ros.org/question/63491/keyboard-key-pressed/
  static struct termios oldt, newt;
  tcgetattr( STDIN_FILENO, &oldt);           // save old settings
  newt = oldt;
  newt.c_lflag &= ~(ICANON);                 // disable buffering
  tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

  int c = getchar();  // read character (non-blocking)

  tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
  return c;
}
