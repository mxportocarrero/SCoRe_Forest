/*

g++ trash.cpp src/dataset.cpp src/linear_algebra_functions.cpp src/utilities.cpp -o trash `pkg-config opencv --cflags --libs` -mavx2 -O3
*
*/

#include "inc/forest.hpp"

#define DATABASE_NAME "data/7_scenes/chess"


int main(int argc, char const *argv[])
{

	// Settings
	Settings *settings = new Settings(640, 480, 5000, 525.0f, 525.0f, 319.5f, 239.5f); // Default

	// Dataset
	Dataset *myDataset = new Dataset(DATABASE_NAME,1);

	cv::namedWindow("Display RGB",cv::WINDOW_AUTOSIZE);
	cv::namedWindow("Display Depth",cv::WINDOW_AUTOSIZE);

	for (int i = 0; i < myDataset->getNumFrames(); ++i)
	{
		cv::imshow("Display RGB",myDataset->getRgbImage(i));
		cv::imshow("Display Depth",myDataset->getDepthImage(i));

		cv::waitKey(40);
	}

	//Pose pose = read_pose("data/7_scenes/chess/seq-01/frame-000000.pose.txt");

	//printMat44(pose, "pose");

}