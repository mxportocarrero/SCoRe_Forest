// g++ trash2.cpp src/dataset.cpp src/linear_algebra_functions.cpp src/utilities.cpp -o trash2 `pkg-config opencv --cflags --libs` -mavx2 -O3

#include "inc/forest.hpp"

#define ds1 "data/7_scenes/chess/seq-01/frame-000000.depth.png"
#define ds2 "data/rgbd_dataset_freiburg11_desk/depth/1305031453.374112.png"

int main(int argc, char const *argv[])
{
	cv::namedWindow("ds1",cv::WINDOW_AUTOSIZE);
	//cv::namedWindow("ds2",cv::WINDOW_AUTOSIZE);

	cv::Mat img_depth1 = cv::imread(ds1, CV_LOAD_IMAGE_ANYDEPTH);
	//cv::Mat img_depth2 = cv::imread(ds2, CV_LOAD_IMAGE_ANYDEPTH);

	cv::imshow("ds1",img_depth1);
	cv::waitKey();
	//cv::imshow("ds2",img_depth2);

	for (int row = 0; row < 640; ++row)
	{
		for (int col = 0; col < 480; ++col)
		{
			if (img_depth1.at<ushort>(col,row) == 65535)
			{
				img_depth1.at<ushort>(col,row) = 0;
			}
		}
	}

	cv::imshow("ds1",img_depth1);
	cv::waitKey();

	return 0;
}