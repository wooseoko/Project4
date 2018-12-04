//state definition
#define INIT 0
#define PATH_PLANNING 1
#define RUNNING 2
#define FINISH -1

#include <unistd.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Time.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <project2/rrtTree.h>
#include <tf/transform_datatypes.h>
#include <project2/pid.h>
#include <math.h>
#include <pwd.h>

#include "geometry_msgs/PoseWithCovarianceStamped.h"

PID pid_ctrl;
//map spec
cv::Mat map;
double res;
int map_y_range;
int map_x_range;
double map_origin_x;
double map_origin_y;
double world_x_min;
double world_x_max;
double world_y_min;
double world_y_max;

//parameters we should adjust : K, margin, MaxStep
int margin = 3;
int K = 2000;
double MaxStep = 2;
int waypoint_margin = 24;

//way points
std::vector<point> waypoints;

//path
std::vector<traj> path_RRT;

//entering theta
std::vector<double> entering_theta;

//path length 
std::vector<int> path_length;

//robot
point robot_pose;
ackermann_msgs::AckermannDriveStamped cmd;

//FSM state
int state;

//function definition
void setcmdvel(double vel, double deg);
void callback_state(geometry_msgs::PoseWithCovarianceStampedConstPtr msgs);
void set_waypoints();
void generate_path_RRT();

int main(int argc, char** argv){
    ros::init(argc, argv, "slam_main");
    ros::NodeHandle n;
	int i=0;
    // Initialize topics
    ros::Publisher cmd_vel_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("/vesc/high_level/ackermann_cmd_mux/input/nav_0",1);
    
    ros::Subscriber gazebo_pose_sub = n.subscribe("/amcl_pose", 100, callback_state);

    printf("Initialize topics\n");
    
    // FSM
    state = INIT;
    bool running = true;
	int look_ahead_idx;
    ros::Rate control_rate(60);

    while(running){
        switch (state) {
        case INIT: {
            
            // Load Map
            char* user = getpwuid(getuid())->pw_name;
            cv::Mat map_org = cv::imread((std::string("/home/") + std::string(user) +
                              std::string("/catkin_ws/src/project4/src/slam_map.pgm")).c_str(), CV_LOAD_IMAGE_GRAYSCALE);

            cv::transpose(map_org,map);
            cv::flip(map,map,1);

            map_y_range = map.cols;
            map_x_range = map.rows;
            map_origin_x = map_x_range/2.0 - 0.5;
            map_origin_y = map_y_range/2.0 - 0.5;
            world_x_min = -4.5;
            world_x_max = 4.5;
            world_y_min = -13.5;
            world_y_max = 13.5;
            res = 0.05;
            printf("Load map\n");

             if(! map.data )                              // Check for invalid input
            {
                printf("Could not open or find the image\n");
                return -1;
            }
            state = PATH_PLANNING;
        } break;

        case PATH_PLANNING:
            
            // Set Way Points
            set_waypoints();
            printf("Set way points\n");

            // RRT
            generate_path_RRT();
            printf("Generate RRT\n");

            ros::spinOnce();
            ros::Rate(0.33).sleep();

            printf("Initialize ROBOT\n");
            state = RUNNING;

        case RUNNING:{
                //TODO 1
                while(ros::ok()){
//          printf("Running PID \n");
/*              printf("start while & i : %d\n",i);
//              printf("look_ahead_idx %d",look_ahead_idx);
//              printf(" path x , y , th %2f, %2f, %2f \n", path_RRT[i].x,path_RRT[i].y,path_RRT[i].th);*/
//              printf("robot pose x, y, th %2f, %2f, %2f\n",robot_pose.x,robot_pose.y,robot_pose.th);
                point path_now;
                path_now.x = path_RRT[i].x;
                path_now.y = path_RRT[i].y;
                path_now.th = path_RRT[i].th;

                float ctrl = pid_ctrl.get_control(robot_pose,path_now);
                float speed= 0.9 + 1/fabs(ctrl)/5;
                if(speed>1.0) speed=1.0;
                float max_steering = (0.45/speed + 0.6 < 1.18)? 0.45/speed + 0.6 : 1.18;
                float steering = ctrl*max_steering/3;
//              printf("ctrl %f \n", steering);
//              printf("error , error_sum , error_diff :  %.2f  %.2f  %.2f \n",pid_ctrl.error,pid_ctrl.error_diff,pid_ctrl.error_sum  );
//              printf("path_RRT[i].x , y %.2f  %.2f \n ", path_RRT[i].x,path_RRT[i].y);
                setcmdvel(speed,steering);
                cmd_vel_pub.publish(cmd);
                if(pow(path_RRT[i].x-robot_pose.x,2)+pow(path_RRT[i].y-robot_pose.y,2)<pow(0.3,2)) {
                    i++;
                }
//              printf("look_ahead_idx %d\n",look_ahead_idx); 
                if(pow(waypoints[look_ahead_idx].x-robot_pose.x,2)+pow(waypoints[look_ahead_idx].y-robot_pose.y,2)<pow(0.45,2)) look_ahead_idx++;
                if(look_ahead_idx==waypoints.size())
                {
                    state=FINISH;
                    break;
                }


				ros::spinOnce();
				control_rate.sleep();
				}

        } break;

        case FINISH: {
            setcmdvel(0,0);
            cmd_vel_pub.publish(cmd);
            running = false;
            ros::spinOnce();
            control_rate.sleep();
        } break;
        default: {
        } break;
        }
    }
    return 0;
}

void setcmdvel(double vel, double deg){
    cmd.drive.speed = vel;
    cmd.drive.steering_angle = deg;
}

void callback_state(geometry_msgs::PoseWithCovarianceStampedConstPtr msgs){
    robot_pose.x = msgs->pose.pose.position.x;
    robot_pose.y = msgs->pose.pose.position.y;
    robot_pose.th = tf::getYaw(msgs->pose.pose.orientation);
    //printf("x,y : %f,%f \n",robot_pose.x,robot_pose.y);
}

void set_waypoints()
{
    std::srand(std::time(NULL));
    point waypoint_candid[5];
    waypoint_candid[0].x = -3.5;
    waypoint_candid[0].y = 12.0;

    cv::Mat map_margin = map.clone();
    int jSize = map.cols; // the number of columns
    int iSize = map.rows; // the number of rows

    for (int i = 0; i < iSize; i++) {
        for (int j = 0; j < jSize; j++) {
            if (map.at<uchar>(i, j) < 125) {
                for (int k = i - waypoint_margin; k <= i + waypoint_margin; k++) {
                    for (int l = j - waypoint_margin; l <= j + waypoint_margin; l++) {
                        if (k >= 0 && l >= 0 && k < iSize && l < jSize) {
                            map_margin.at<uchar>(k, l) = 0;
                        }
                    }
                }
            }
        }
    }

    //TODO 2
	//4th quadrant
	    while(true)
	    {
    	    int rand_i = ((1+iSize)/2) + std::rand() % (iSize/2);
	        int rand_j = ((1+jSize)/2) + std::rand() % (jSize/2);
	        if(map_margin.at<uchar>(rand_i,rand_j)>125)
	        {
	          waypoint_candid[1].x = res*(rand_i-map_origin_x);
	          waypoint_candid[1].y = res*(rand_j-map_origin_y);
	          break;
	        }
	    }
	
	    //3th quadrant
	    while(true)
	    {
	        int rand_i = ((1+iSize)/2) + std::rand() % (iSize/2);
	        int rand_j = std::rand() % (jSize/2);
	        if(map_margin.at<uchar>(rand_i,rand_j)>125)
	        {
	          waypoint_candid[2].x = res*(rand_i-map_origin_x);
	          waypoint_candid[2].y = res*(rand_j-map_origin_y);
	          break;
	        }
	    }
	
	    //2th quadrant
	    while(true)
	    {
	        int rand_i = std::rand() % (iSize/2);
	        int rand_j = std::rand() % (jSize/2);
	        if(map_margin.at<uchar>(rand_i,rand_j)>125)
	        {
	          waypoint_candid[3].x = res*(rand_i-map_origin_x);
	          waypoint_candid[3].y = res*(rand_j-map_origin_y);
	          break;
	        }
	}

    waypoint_candid[4].x = -3.5;
    waypoint_candid[4].y = 12.0;

    int order[] = {0,1,2,3,4};
    int order_size = 5;

    for(int i = 0; i < order_size; i++){
        waypoints.push_back(waypoint_candid[order[i]]);
    }
}

void generate_path_RRT()
{
    //TODO 1
		for(int i=0;i<waypoints.size();i++) 
		{
		    entering_theta.push_back(0.0);
		    path_length.push_back(0);
		}	
//      printf("RRT\n");
        point lastp=waypoints[0];
		entering_theta[0]=waypoints[0].th;
        for(int i=0; i<waypoints.size()-1; i++){
			lastp.x=waypoints[i].x;
            lastp.y=waypoints[i].y;
            rrtTree *tree = new rrtTree(lastp, waypoints[i+1], map, map_origin_x, map_origin_y, res, margin);
            tree->generateRRT(world_x_max, world_x_min, world_y_max, world_y_min, K, MaxStep);
            std::vector<traj> vec = tree->backtracking_traj();
			if( (vec.begin()->x-waypoints[i+1].x)*(vec.begin()->x-waypoints[i+1].x)+(vec.begin()->y-waypoints[i+1].y)*(vec.begin()->y-waypoints[i+1].y) > 0.04 && i>0 ){
   			    printf("!!!REFIND THE WAY\n");
			    printf(" x y %.2f %.2f waypoints %.2f %.2f\n", vec.begin()->x, vec.begin()->y,waypoints[i+1].x, waypoints[i+1].y);
			    if(i>0){ 
					i=i-2; 
			 		lastp.th = entering_theta[i+1];
 					for(int j=0;j<path_length[i+1];j++) path_RRT.pop_back();
		    	}
			}
//                std::reverse(vec.begin(),vec.end());
			else{
		    	lastp.x = vec.begin()->x;
                lastp.y = vec.begin()->y;
                lastp.th = vec.begin()->th;
		    	entering_theta[i+1]= lastp.th;		
                tree->visualizeTree();
	    	    std::reverse(vec.begin(),vec.end());
                tree->visualizeTree(vec);
		    	path_length[i] = vec.size();
                for(int j=0; j<vec.size();j++)
                path_RRT.push_back(vec[j]);
			}
            delete tree;
        }
        traj lastpoint;
        lastpoint.x=waypoints[waypoints.size()-1].x;
        lastpoint.y=waypoints[waypoints.size()-1].y;
        lastpoint.th=waypoints[waypoints.size()-1].th; 
        path_RRT.push_back(lastpoint);


}



