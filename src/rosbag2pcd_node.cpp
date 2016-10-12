#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <boost/filesystem.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>
#include <stdio.h>
#include <termios.h>

#include <opencv2/opencv.hpp>

#include <boost/algorithm/string.hpp>
#define BOOST_NO_CXX11_SCOPED_ENUMS
#include <boost/filesystem.hpp>
#undef BOOST_NO_CXX11_SCOPED_ENUMS
#include <boost/regex.hpp>

namespace bf = boost::filesystem;

size_t message_counter;
ros::Subscriber sub;
std::string output_folder;

bool take_snapshot;
bool record;

bool
existsFile ( const std::string &rFile )
{
    bf::path dir_path = rFile;
    return (bf::exists ( dir_path ) && bf::is_regular_file(dir_path));
}

std::string
createFilename()
{
    std::stringstream filename;
    do
    {
        filename.str("");
        filename << output_folder << "/cloud_" << std::setfill ('0') << std::setw (10) << message_counter << ".pcd";
        message_counter++;
    }while ( existsFile(filename.str() ) );
    return filename.str();
}

void
pclCallback(const sensor_msgs::PointCloud2& msg)
{
    if(take_snapshot || record)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::fromROSMsg(msg, *cloud);
        const std::string filename = createFilename();
        std::cout << "Saving PCD to " << filename << std::endl;
        pcl::io::savePCDFileBinaryCompressed(filename, *cloud);
    }

    if(take_snapshot)
        take_snapshot = false;
}

char getch()
{
    fd_set set;
    struct timeval timeout;
    int rv;
    char buff = 0;
    int len = 1;
    int filedesc = 0;
    FD_ZERO(&set);
    FD_SET(filedesc, &set);

    timeout.tv_sec = 0;
    timeout.tv_usec = 1000;

    rv = select(filedesc + 1, &set, NULL, NULL, &timeout);

    struct termios old = {0};
    if (tcgetattr(filedesc, &old) < 0)
        ROS_ERROR("tcsetattr()");
    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN] = 1;
    old.c_cc[VTIME] = 0;
    if (tcsetattr(filedesc, TCSANOW, &old) < 0)
        ROS_ERROR("tcsetattr ICANON");

    if(rv != -1 && rv !=0)
        read(filedesc, &buff, len );

    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;
    if (tcsetattr(filedesc, TCSADRAIN, &old) < 0)
        ROS_ERROR ("tcsetattr ~ICANON");
    return (buff);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rosbag2pcd");
  ros::NodeHandle nh;
  output_folder = "/tmp/clouds/";
  take_snapshot = false;
  record = false;

  std::string topic = "/kinect2/hd/points";
  std::cout << "Subscribing to topic: " << topic << std::endl
	    << "  SPACE... start/stop exporting .pcd file from stream" << std::endl
	    << "  s... export .pcd snapshot" << std::endl; 

  boost::filesystem::create_directories( boost::filesystem::path( output_folder ) );

  sub = nh.subscribe(topic, 1, &pclCallback);
  ros::Rate loop_rate(1);
  message_counter = 0;
  
  while( ros::ok() )
  { 
      char c = getch();   // call your non-blocking input function

      if (c == 's' && record)
      {
          std::cerr << "Cannot take snapshot and record at the same time. Please stop recording first." << std::endl;
      }
      else if (c == 's')
      {
          std::cout << " Taking snapshot..." << std::endl;
          take_snapshot = true;
      }
      else if(c == ' ')
      {
          if( !record)
          {
              std::cout << " Start recording..." << std::endl;
              record = true;
          }
          else
          {
              std::cout << " Stop recording." << std::endl;
              record = false;
          }
      }

      ros::spinOnce();
      loop_rate.sleep();
  }
}

