/**
CODIGO PARA VERIFICAR LOS RESULTADOS DE LAS ESTRUCTURAS

Compilacion:
g++ test.cpp src/dataset.cpp src/linear_algebra_functions.cpp src/utilities.cpp -o test `pkg-config opencv --cflags --libs` -mavx2 -O3

Ejecucion:
./test

*/

#include "inc/forest.hpp"

#define DATABASE_NAME "data/rgbd_dataset_freiburg11_desk"
// Esta base de datos tiene:
//	-> 576 frames de sincronizados RGBD
//	-> 2338 frames de groundtruth

int main(int argc, char const *argv[])
{
	// Configuraciones
	Settings *settings = new Settings(640, 480, 5000, 525.0f, 525.0f, 319.5f, 239.5f);

	// Leyendo la Data
	Dataset *data = new Dataset(DATABASE_NAME);

	// 1er Test: Lectura de Datos --- DESARROLLO
	cv::namedWindow("Display RGB",cv::WINDOW_AUTOSIZE);
	cv::namedWindow("Display Depth",cv::WINDOW_AUTOSIZE);

	std::cout << "Total number of Frames: " << data->getNumFrames() << std::endl;

	for (int i = 0; i < data->getNumFrames(); ++i)
	{
		std::cout << data->getTimestamp(i) << " ";

		cv::imshow("Display RGB",data->getRgbImage(i));
		cv::imshow("Display Depth",data->getDepthImage(i));
		Pose pose = data->getPose(i);

		printMat44(pose, "pose");

		printEigenVector3d( posePosition(pose) );		

		cv::waitKey();
	}

	// 2do Test: Entrenamiento de 1 arbol --- DESARROLLO

	return 0;
}