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

#include "inc/tree.hpp"

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

	// Testeando la Clase Tree

	// SETTINGS
	// --------
	// los argumentos representan los siquientes parámetros
	// width / heigth / depth_factor / fx / fy / cx / cy

	// También hay otros parametros que estan definidos aqui
	// num_trees = 5 / max_tree_depth = 16 / num_frames_per_tree = 500
	// num_px_per_frame = 5000
	Settings *settings = new Settings(640, 480, 5000, 525.0f, 525.0f, 319.5f, 239.5f);

	// DATA
	Dataset *myDataset = new Dataset(DATABASE_NAME);

	Tree<ushort, cv::Vec3b> * tree = new Tree<ushort, cv::Vec3b>();

	return 0;
}