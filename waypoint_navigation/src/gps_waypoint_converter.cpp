#include <ros/ros.h>
#include <ros/package.h>
#include <fstream>
#include <utility>
#include <vector>
#include <waypoint_navigation/navsat_conversions.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <tf/transform_listener.h>
#include <math.h>




// initialize variables
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

std::vector <std::pair<double, double>> waypointVect;
std::vector<std::pair < double, double> > ::iterator iter; //init. iterator
geometry_msgs::PointStamped UTM_point, map_point, UTM_next, map_next;
int count = 0, waypointCount = 0, wait_count = 0;
double numWaypoints = 0;
double latiGoal, longiGoal, latiNext, longiNext;
std::string utm_zone;
std::string path_local, path_abs;
bool new_goal = true;


void goalCallback(const std_msgs::Bool::ConstPtr& msg){

    if (msg->data) { new_goal = true;  }


}


int countWaypointsInFile(std::string path_local)
{
    path_abs = ros::package::getPath("waypoint_navigation") + path_local;
    std::ifstream fileCount(path_abs.c_str());
    if(fileCount.is_open())
    {
        double lati = 0;
        while(!fileCount.eof())
        {
            fileCount >> lati;
            ++count;
        }
        //count = count - 1;
        numWaypoints = count / 2;
        ROS_INFO("%.0f GPS waypoints were read", numWaypoints);
        fileCount.close();
    }
    else
    {
        std::cout << "Unable to open waypoint file" << std::endl;
        ROS_ERROR("Unable to open waypoint file");
    }
    return numWaypoints;
}

std::vector <std::pair<double, double>> getWaypoints(std::string path_local)
{
    double lati = 0, longi = 0;

    path_abs = ros::package::getPath("waypoint_navigation") + path_local;
    std::ifstream fileRead(path_abs.c_str());
    for(int i = 0; i < numWaypoints; i++)
    {
        fileRead >> lati;
        fileRead >> longi;
        waypointVect.push_back(std::make_pair(lati, longi));
    }
    fileRead.close();

    //Outputting vector
    ROS_INFO("The following GPS Waypoints have been set:");
    for(std::vector < std::pair < double, double >> ::iterator iterDisp = waypointVect.begin(); iterDisp != waypointVect.end();
    iterDisp++)
    {
        ROS_INFO("%.9g %.9g", iterDisp->first, iterDisp->second);
    }
    return waypointVect;
}

geometry_msgs::PointStamped latLongtoUTM(double lati_input, double longi_input)
{
    double utm_x = 0, utm_y = 0;
    geometry_msgs::PointStamped UTM_point_output;

    //convert lat/long to utm
    RobotLocalization::NavsatConversions::LLtoUTM(lati_input, longi_input, utm_y, utm_x, utm_zone);

    //Construct UTM_point and map_point geometry messages
    UTM_point_output.header.frame_id = "utm";
    UTM_point_output.header.stamp = ros::Time(0);
    UTM_point_output.point.x = utm_x;
    UTM_point_output.point.y = utm_y;
    UTM_point_output.point.z = 0;

    return UTM_point_output;
}

geometry_msgs::PointStamped UTMtoMapPoint(geometry_msgs::PointStamped UTM_input)
{
    geometry_msgs::PointStamped map_point_output;
    bool notDone = true;
    tf::TransformListener listener; //create transformlistener object called listener
    ros::Time time_now = ros::Time::now();
    while(notDone)
    {
        try
        {
            UTM_point.header.stamp = ros::Time::now();
            listener.waitForTransform("map", "utm", time_now, ros::Duration(3.0));
            listener.transformPoint("map", UTM_input, map_point_output);
            notDone = false;
        }
        catch (tf::TransformException& ex)
        {
            ROS_WARN("%s", ex.what());
            ros::Duration(0.01).sleep();
            //return;
        }
    }
    return map_point_output;
}

geometry_msgs::PoseStamped  buildGoal(geometry_msgs::PointStamped map_point, geometry_msgs::PointStamped map_next, bool last_point)
{
    geometry_msgs::PoseStamped goal;

    //Specify what frame we want the goal to be published in
    goal.header.frame_id = "map";
    goal.header.stamp = ros::Time::now();

    // Specify x and y goal
    goal.pose.position.x = map_point.point.x; //specify x goal 
    goal.pose.position.y = map_point.point.y; //specify y goal
    ROS_INFO("goalxy %f    %f", map_point.point.x , map_point.point.y);

    // Specify heading goal using current goal and next goal (point robot towards its next goal once it has achieved its current goal)
    if(last_point == false)
    {
        tf::Matrix3x3 rot_euler;
        tf::Quaternion rot_quat;

        // Calculate quaternion
        float x_curr = map_point.point.x, y_curr = map_point.point.y; // set current coords.
        float x_next = map_next.point.x, y_next = map_next.point.y; // set coords. of next waypoint
        float delta_x = x_next - x_curr, delta_y = y_next - y_curr;   // change in coords.
        float yaw_curr = 0, pitch_curr = 0, roll_curr = 0;
        yaw_curr = atan2(delta_y, delta_x);

        // Specify quaternions
        rot_euler.setEulerYPR(yaw_curr, pitch_curr, roll_curr);
        rot_euler.getRotation(rot_quat);

        goal.pose.orientation.x = rot_quat.getX();
        goal.pose.orientation.y = rot_quat.getY();
        goal.pose.orientation.z = rot_quat.getZ();
        goal.pose.orientation.w = rot_quat.getW();
    }
    else
    {
        goal.pose.orientation.w = 1.0;
    }

    return goal;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "gps_waypoint"); //initiate node called gps_waypoint
    ros::NodeHandle n;
    ROS_INFO("Initiated gps_waypoint node");
    std::vector<geometry_msgs::PoseStamped> list;
    int i = 0;

    MoveBaseClient ac("move_base", true);
    while(!ac.waitForServer(ros::Duration(5.0))){
     ROS_INFO("Waiting for the move_base action server to come up");
     }

    move_base_msgs::MoveBaseGoal goal_m;
    // Initiate publisher to send end of node message
    ros::Publisher pubWaypointNodeEnded = n.advertise<std_msgs::Bool>("/waypoint_navigation/waypoint_following_status", 100);
    ros::Publisher waypoint_pub = n.advertise<geometry_msgs::PoseStamped >("/goal",100);

    ros::Subscriber sub_state_ = n.subscribe("goal_reached", 1000, goalCallback);



    //Get Longitude and Latitude goals from text file

    //Count number of waypoints
    ros::param::get("/waypoint_navigation/coordinates_file", path_local);
    numWaypoints = countWaypointsInFile(path_local);

    //Reading waypoints from text file and output results
    waypointVect = getWaypoints(path_local);


    // Iterate through vector of waypoints for setting goals
    for(iter = waypointVect.begin(); iter < waypointVect.end(); iter++)
    {
        //Setting goal:
        latiGoal = iter->first;
        longiGoal = iter->second;
        bool final_point = false;

        //set next goal point if not at last waypoint
        if(iter < (waypointVect.end() - 1))
        {   
            iter++;
            latiNext = iter->first;
            longiNext = iter->second;
            iter--;
        }
        else //set to current
        {
            latiNext = iter->first;
            longiNext = iter->second;
            final_point = true;
        }

        ROS_INFO("Received Latitude goal:%.8f", latiGoal);
        ROS_INFO("Received longitude goal:%.8f", longiGoal);

        //Convert lat/long to utm:
        UTM_point = latLongtoUTM(latiGoal, longiGoal);
        UTM_next = latLongtoUTM(latiNext, longiNext);

        //Transform UTM to map point in odom frame
        map_point = UTMtoMapPoint(UTM_point);
        map_next = UTMtoMapPoint(UTM_next);

        //Build goal to send to move_base
        // Send Goal
        geometry_msgs::PoseStamped goal = buildGoal(map_point, map_next, final_point);
        list.push_back(goal); 


    } // End for loop iterating through waypoint vector

    // Notify joy_launch_control that waypoint following is complete
    //std_msgs::Bool node_ended;
    //node_ended.data = true;
    //pubWaypointNodeEnded.publish(node_ended);


     ros::Rate loop_rate(10);
     while( ros::ok() && (i < list.size()) ){ 
        goal_m.target_pose = list[i];
      // if (new_goal){
        ac.sendGoal(goal_m);
        ROS_INFO("Sending goal"); 
        ac.waitForResult();
        // waypoint_pub.publish(list[i]); 
        // new_goal = false ;
        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
           ROS_INFO("goal reached");
           i++; 

        }

      ros::spinOnce();
      }

    ros::shutdown();

    return 0;
}
