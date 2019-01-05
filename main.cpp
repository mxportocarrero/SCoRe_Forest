/**
Implementacion de Regression Forest
Esta es una adaptacion del codigo propuesto por

@software{brooks16,
author        = {Conner Brooks},
title         = {C++ Implementation of SCORE Forests for Camera Relocalization},
howpublished  = {\url{https://github.com/isue/relocforests}},
year          = {2016}
}

en referencia al paper

@Inprceedings {export:184826,
author       = {Jamie Shotton and Ben Glocker and Christopher Zach and Shahram Izadi and Antonio
                Criminisi and Andrew Fitzgibbon},
booktitle    = {Proc. Computer Vision and Pattern Recognition (CVPR)},
month        = {June},
publisher    = {IEEE},
title        = {Scene Coordinate Regression Forests for Camera Relocalization in RGB-D Images},
url          = {http://research.microsoft.com/apps/pubs/default.aspx?id=184826},
year         = {2013},
} 

Comando para compilacion:

g++ main.cpp src/dataset.cpp src/linear_algebra_functions.cpp src/utilities.cpp -o reg_forest `pkg-config opencv --cflags --libs` -mavx2 -O3

Comando para ejecucion:

./reg_forest

*/

#include "inc/dataset.hpp"

#include "inc/node.hpp"

#define DATABASE_NAME "data/rgbd_dataset_freiburg11_room"

int main(int argc, char const *argv[])
{
	/**
	Dataset myDataset(DATABASE_NAME);

	cv::namedWindow("Display RGB",cv::WINDOW_AUTOSIZE);
	cv::namedWindow("Display Depth",cv::WINDOW_AUTOSIZE);

	for (int i = 0; i < myDataset.getNumFrames(); ++i)
	{
		cv::imshow("Display RGB",myDataset.getRgbImage(i));
		cv::imshow("Display Depth",myDataset.getDepthImage(i));

		cv::waitKey(30);
	}
	*/

	std::cout << "Hola Mundo!2\n";

	/** // Test para el generador de numeros random
	Random myRandGenerator;
	for (int i = 0; i < 5; ++i)
	{
		std::cout << myRandGenerator.Next(-130,130) << std::endl;
	}
	*/ // Fin de Comentario

	/** // Testeando la clase feature
	Random *myRandGenerator = new Random();

	DepthAdaptiveRGB<ushort, cv::Vec3b> feature = DepthAdaptiveRGB<ushort, cv::Vec3b>::CreateRandom(myRandGenerator);

	std::cout << feature.GetThreshold() << std::endl;
	*/

	// Testeando la clase Node
	


	return 0;
}