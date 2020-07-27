#include <Eigen/Dense>
#include <iostream>
#include <math.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/core.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Header.h>


using namespace std;

#define M_PI 3.14159265358979323846 

void event_update(Eigen::MatrixXd & input_matrix, int x_mean, double x_var, int x_window, int y_mean, double y_var, int y_window)
{
    int x_start_index = std::max(x_mean-x_window, 0);
    int x_end_index = std::min(x_mean+x_window, (int)input_matrix.rows()-1);
    int y_start_index = std::max(y_mean-y_window, 0);
    int y_end_index = std::min(y_mean+y_window, (int)input_matrix.cols()-1);

    for (int i = x_start_index ; i <= x_end_index ; i++)
    {
        for (int j = y_start_index ; j <= y_end_index ; j++)
        {
            input_matrix(i,j) += std::exp(-0.5 * ((std::pow(i-x_mean, 2)/x_var) + (std::pow(j-y_mean, 2)/y_var)));// / (2*M_PI*std::sqrt(x_var*y_var));
        }
    }
}

// void event_update(cv::Mat & input_matrix, int x_mean, double x_var, int x_window, int y_mean, double y_var, int y_window)
// {
//     int x_start_index = std::max(x_mean-x_window, 0);
//     int x_end_index = std::min(x_mean+x_window, (int)input_matrix.rows-1);
//     int y_start_index = std::max(y_mean-y_window, 0);
//     int y_end_index = std::min(y_mean+y_window, (int)input_matrix.cols-1);

//     for (int i = x_start_index ; i <= x_end_index ; i++)
//     {
//         for (int j = y_start_index ; j <= y_end_index ; j++)
//         {
//             input_matrix(i,) += (int) (255 * std::exp(-0.5 * ((std::pow(i-x_mean, 2)/x_var) + (std::pow(j-y_mean, 2)/y_var))));// / (2*M_PI*std::sqrt(x_var*y_var));
//         }
//     }
// }

void matrix_update(Eigen::MatrixXd & input_matrix, double L , double timestamp_new, double timestamp_old)
{
    //double temp = timestamp;
    //double Alpha = exp(-L * (timestamp - temp)) ;
    double Alpha = exp(-L * (timestamp_new - timestamp_old)) ;
    input_matrix = Alpha * input_matrix;

}


int main(int argc, char **argv){
    
    ros::init(argc, argv, "servoing");
	ros::NodeHandle pnh_;

    Eigen::MatrixXd test_matrix(10,10);
    test_matrix = Eigen::MatrixXd::Zero(10,10);

    event_update(test_matrix, 5, 3, 100, 5, 3, 100);
    //cout << test_matrix << endl;

    cv::Mat float_image(10, 10, CV_64FC1);
    cv::Mat CV_image(10, 10, CV_8UC1);
    
	cv::eigen2cv(test_matrix, float_image); 


    cv::convertScaleAbs(float_image, CV_image, 255, 0.0);



    // matrix_update(test_matrix, 10, 1000010, 1000000);
    // cout << test_matrix << endl;

    // matrix_update(test_matrix, 0.3, 1, 0);
    // cout << test_matrix << endl;
    //while(1);


    std_msgs::Header header; // empty header
	header.seq = 1; // user defined counter
	header.stamp = ros::Time::now(); // time
    header.frame_id = "test";
    
    sensor_msgs::Image image_ros;


    cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );// Create a window for display.
    cv::imshow( "Display window", float_image );                   // Show our image inside it.

    cv::waitKey(0);                                          // Wait for a keystroke in the window

    
    cv_bridge::CvImage img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, CV_image);

	image_ros = *(img_bridge.toImageMsg()); // from cv_bridge to sensor_msgs::Image
    cout << CV_image.channels() << endl;
    cout << CV_image.size() << endl;
    cout << CV_image << endl;

    cout << image_ros.height << image_ros.width << "," << image_ros.is_bigendian << endl;

    
    ros::Publisher pub_heatmap =pnh_.advertise<sensor_msgs::Image>("/corner_heatmap", 1);
    ros::Rate loop_rate(10);

    while(ros::ok())
    {
        pub_heatmap.publish(image_ros); // ros::Publisher pub_img = node.advertise<sensor_msgs::Image>("topic", queuesize);
        
        loop_rate.sleep();

    }
	
    return 0;

}  