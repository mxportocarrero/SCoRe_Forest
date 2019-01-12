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

./reg_forest (train|test) <forest_file_name>

*/

#include "inc/forest.hpp"

#define DATABASE_NAME "data/rgbd_dataset_freiburg11_desk"
//#define DATABASE_NAME "data/rgbd_dataset_freiburg22_pioneer_slam"

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

	// EL programa se ejecuta con algunas opciones
	// opc1 ( train | test ) // opcion que indica si entrenar o testear
	// opc2 <forest_file_name> // archivo para guardar el entrenamiento del bosque

	std::string opc1(argv[1]);
	std::string opc2(argv[2]);
	std::cout.precision(4);

	bool isTraining = (opc1 == "train");
	std::string forest_file_name = opc2;

	// Testeando la Clase Forest

	// SETTINGS
	// --------
	// los argumentos representan los siquientes parámetros
	// width / heigth / depth_factor / fx / fy / cx / cy

	// También hay otros parametros que estan definidos aqui
	// num_trees = 5 / max_tree_depth = 16 / num_frames_per_tree = 500
	// num_px_per_frame = 5000

	Settings *settings = new Settings(640, 480, 5000, 525.0f, 525.0f, 319.5f, 239.5f); // Default
	//Settings *settings = new Settings(640, 480, 5000, 517.3f, 516.5f, 318.6f, 255.3f); // Freiburg 1
	//Settings *settings = new Settings(640, 480, 5000, 535.4f, 539.2f, 320.1f, 247.6f); // Freiburg 3

	// DATA
	Dataset *myDataset = new Dataset(DATABASE_NAME);

	// Tree<ushort, cv::Vec3b> * tree = new Tree<ushort, cv::Vec3b>();
	Forest<ushort, cv::Vec3b> *forest = nullptr;

	// Seleccionando tarea
	if(isTraining)
	{
		forest = new Forest<ushort, cv::Vec3b>(myDataset,settings);
		forest->Train();
		forest->Serialize(forest_file_name);

		std::cout << "Is forest valid:" << forest->IsValid() << std::endl;
	}
	else // Esta en modo testing
	{
		// Cargando el arbol
		forest = new Forest<ushort, cv::Vec3b>(myDataset,settings,forest_file_name);

		// Revisar si el arbol es válido
		if (forest->IsValid())
			std::cout << "Forest is Valid" << std::endl;
		else
		{
			std::cout << "Forest is NOT valid" << std::endl;
			return 1;
		}		

	//**	// TESTING

		// Todo: test each image in test dataset and provide relevent statistics about accuracy
		Random randGen;
		int correct_predictions = 0;
		int num_of_tests = 100;
	    cv::namedWindow("Display RGB",cv::WINDOW_AUTOSIZE);
	    // eval forest at random frames
	    for (int i = 0; i < num_of_tests; ++i)
	    {
	    	int frame = randGen.Next(0,myDataset->getNumFrames());

	    	std::cout << "TEST FRAME - " << frame << " ts: " << myDataset->getTimestamp(frame) << std::endl;

		    std::clock_t start;
		    double duration;
		    start = std::clock();

		    Eigen::Affine3d pose = forest->Test(myDataset->getRgbImage(frame), myDataset->getDepthImage(frame));
		    cv::imshow("Display RGB",myDataset->getRgbImage(frame));


		    duration = (std::clock() - start) / (double)CLOCKS_PER_SEC;
		    std::cout << "Test time: " << duration << " Seconds \n";

		    // compare pose to groundtruth value 
		    auto ground_truth = myDataset->getPose(frame);

		    std::cout.precision(5);
		    std::cout << "Evaluated Pose:" << std::endl;
		    //std::cout << pose.rotation() << std::endl << std::endl;
		    Eigen::Vector3d rot_eval = pose.rotation().eulerAngles(0, 1, 2) * 180.0 / M_PI;
		    Eigen::Vector3d pos_eval = pose.translation();
		    std::cout << rot_eval << std::endl;
		    std::cout << pos_eval << std::endl;

		    std::cout << "Ground Truth:" << std::endl;
		    //std::cout << poseRotation(ground_truth) << std::endl;
		    Eigen::Vector3d rot_gt = poseRotation(ground_truth).eulerAngles(0, 1, 2) * 180.0 / M_PI;
		    Eigen::Vector3d pos_gt = posePosition(ground_truth);
		    std::cout << rot_gt << std::endl;
		    std::cout << pos_gt << std::endl;

		    Eigen::Vector3d verror; double error_pos, error_rot;
		    // Error Rotacional
		    verror = rot_eval - rot_gt;
		    error_rot = pow(verror(0),2.0) + pow(verror(1),2.0) + pow(verror(2),2.0);
		    error_rot = sqrt(error_rot);

		    // Error traslacional
		    verror = pos_eval - pos_gt;
		    error_pos = sqrt( pow(verror(0),2.0) + pow(verror(1),2.0) + pow(verror(2),2.0));

		    //if (error_pos < 0.05 && error_rot < 5.0)
		    if (error_pos < 0.10 && error_rot < 5.0)
		    	correct_predictions++;

		    printf("Rotational Error: %.4f Traslational Error: %.4f\n", error_rot, error_pos );


		    if(correct_predictions == 0)
		    {
		    	std::cout << "no correct predictions could be made\n" << std::endl;
		    }
		    else
		    {
		    	std::cout << "Accuracy (%) : " << (float)correct_predictions / (float)(i+1) * 100.0 << std::endl;
		    }

		    cv::waitKey(1000);

	    } // Fin de FOR


//*/
	} // Fin del else (TESTING)

	delete forest;
	delete settings;

	return 0;
}
























