#ifndef DATASET_HPP
#define DATASET_HPP

// Esta clase se encarga de leer el archivo que contiene
// los archivos de los pares RGB-D sincronizados

#include "general_includes.hpp"
#include "linear_algebra_functions.hpp"
#include "utilities.hpp"
#include <map>

/**
LabeledPixel
Esta clase para representar a un solo pixel y su correspondiente valor en world coordinates
*/
class LabeledPixel{
public:
	// Constructor del Labeled Pixel
    LabeledPixel(uint32_t frame, cv::Point2i pos, cv::Point3f label):
    	frame_(frame), pos_(pos), label_(label){}
    uint32_t frame_;
    cv::Point2i pos_;
    cv::Point3f label_;	
};

/**
Dataset
Esta clase se encarga de manejar todos los datos de entrada.
Toma como base el formato propuesto por el Dataset del TUM Sturm 2012

La estructura de los datos es como sigue 
	-> data
		-> DatasetName_Folder
			->rgb 				// Carpeta con las imagenes RGB (8-bit values)
			->depth 			// Carpeta con las imagenes de Profundidad (16-bit values)

			->rgb.txt			// Nombre de todas las imagenes Depth con sus respectivos Timestamps (30Hz)
			->depth.txt			// Nombre de todas las imagenes RGB con sus respectivos Timestamps (30Hz)
			->groundtruth.txt   // Lista de todas las poses y sus respectivos timestamps captados por el Motion Capture System (100Hz)

			->rgb_t.txt			// Lista Pre-procesada con las correspondencias para los Timestamps
			->depth_t.txt		// Lista Pre-procesada con las correspondencias para los Timestamps

Debido a que los datos tomados por el sensor RGB, Depth y Groundtruth no estan sincronizados, en muchas
ocasiones es necesario preprocesar la data de modo que sea posible encontrar correspondencias.
Ello se logra con el archivo 'associate_corrected.py'
*/

class Dataset
{
private:

	// Nombre del 
    std::string dataset_path_;

    std::vector<std::string> rgb_filenames_;
    std::vector<std::string> depth_filenames_;
    // En estos vectores iran los datos RGBD que usaremos
    // Tienen la misma cantidad de elementos
    // No necesariamente son iguales a los datos groundtruth
    std::vector<std::string> timestamp_rgbd_;
    std::vector<Pose> poses_;

    // Loaded images 
    // Vamos a cargar toda la secuencia de imagenes a la RAM
    // Para acelerar su procesamiento
    std::map<uint32_t, cv::Mat> rgb_images_;
    std::map<uint32_t, cv::Mat> depth_images_;

    // En estos vectores almacenaremos los timestamps y Poses del groundtruth
    // Estos vectores tienen la misma cantidad de elementos
    std::vector<std::string> timestamp_groundtruth_;
    std::vector<Pose> poses_gt_;

    // Hace referencia al numero de frames validos que pueden usarse para el entrenamiento
    int num_frames_ = 0;

public:
    Dataset(std::string dataset_path):
    	dataset_path_{dataset_path};

    // Estas funciones se encargan de devolver los cv::Mat y Poses correspondientes
    cv::Mat getRgbImage(int frame);
    cv::Mat getDepthImage(int frame);
    Pose getPose(int frame);

    int getNumFrames();

    // Se encarga de  agregar datos a los diccionarios y vectores que almacenan todo
    void addFrame(cv::Mat rgb_frame, cv::Mat depth_frame, Pose pose);

}; // Fin de Clase de Dataset

// Implementacion de Funciones

/**
Class Constructor
@param dataset_path Un string que indica la ubicacion de la secuencia a utilizarse
*/
Dataset::Dataset(std::string dataset_path):
    dataset_path_(dataset_path){

    std::ifstream myRgbFile;
    std::ifstream myDepthFile;
    std::ifstream myGroundtruthFile;

    myRgbFile.open(dataset_path_ + "/rgb_t.txt");
    myDepthFile.open(dataset_path_ + "/depth_t.txt");
    myGroundtruthFile.open(dataset_path_ + "/groundtruth.txt");

    std::string RGBline;
    std::string DEPTHline;
    std::string Groundtruthline;

    // Validamos que hayamos abierto los archivos
    if (myRgbFile.is_open() && myDepthFile.is_open() && myGroundtruthFile.is_open())
    {
    	std::vector<double> tempStamps,tempStamps_gt; // Vectores para almacenar los timestamps
    	// Lectura de los datos RGBD
    	// *************************
    	while(std::getline(myRgbFile,RGBline) && std::getline(myDepthFile,DEPTHline)){
    		if(RGBline[0] != '#'){ // Omitimos los comentarios, y solo usamos uno porq ya estan preprocesados
	    		timestamp_rgbd_.push_back( split(RGBline,' ')[0] );
	    		rgb_filenames_.push_back( split(RGBline,' ')[1] );
	    		depth_filenames_.push_back( split(DEPTHline,' ')[1] );

	    		tempStamps.push_back( std::stod(split(RGBline,' ')[0]) );
    		}
    	} // Fin de Bucle While

    	// Lectura del Groundtruth
    	// ***********************
    	std::vector<std::string> temp;
    	while(std::getline(myGroundtruthFile,Groundtruthline)){
    		if(Groundtruthline[0] != '#'){
    			temp = split(Groundtruthline,' ');
    			timestamp_groundtruth_.push_back( temp[0] );
    			pose_gt_.push_back(Quaternion_2_Mat44(std::stof(temp[1]),std::stof(temp[2]),std::stof(temp[3]),std::stof(temp[4]),std::stof(temp[5]),std::stof(temp[6]),std::stof(temp[7])));

    			tempStamps_gt.push_back( std::stod( temp[0] ) );
    		}
    	}

    	// Time Alignment
    	// **************
    	
    	for

    	// Declaramos un vector de ternas para almacenar todas las combinaciones de timestamps
    	// que tengan diferencias menores a un max_difference en tiempo
    	double max_difference = 0.02 // en segundos es 2 decimas de segundo
    	std::vector< std::tuple<double,double,double> > ternas;
    	for (int i = 0; i < timestamp_rgbd_.size(); ++i){
    		for (int j = 0; j < timestamp_groundtruth_.size(); ++j){
    			double a = timestamp_rgbd_[i], b = timestamp_groundtruth_[j];
    			if(abs(a - b) < max_difference){
    				std::tuple<double,double,double> t(a-b,a,b);
    				ternas.push_back(tuple);
    			}
    		}
    	}

    	// Ordenamos las tuplas de acuerdo al primer valor
    	std::sort(ternas.begin(),ternas.end());

    	for (int i = 0; i < ternas.size(); ++i){
    		/* code */
    	}




    } else {
    	std::cout << "Could not Open a File\n";
    }
} // Fin del constructor

cv::Mat Dataset::getRgbImage(int frame){
	if (rgb_images_.count(frame)) {
    	return rgb_images_.at(frame);
    }
    else {
    	cv::Mat img = cv::imread(data_path_ + rgb_names_.at(frame));
        rgb_images_.insert(std::pair<uint32_t, cv::Mat>(frame, img));
        return img;
    }
}

cv::Mat Dataset::getDepthImage(int frame){
	if (depth_images_.count(frame)) {
    	return depth_images_.at(frame);
    }
    else {
    	cv::Mat img = cv::imread(data_path_ + depth_names_.at(frame), CV_LOAD_IMAGE_ANYDEPTH);
    	depth_images_.insert(std::pair<uint32_t, cv::Mat>(frame, img));
        return img;
    }
}

Pose Dataset::getPose(int frame){
	return poses_.at(frame);
}

int Dataset::getNumFrames(){
    return num_frames_;
}

void Dataset::addFrame(cv::Mat rgb_frame, cv::Mat depth_frame, Pose pose){
	rgb_images_.insert(std::pair<uint32_t, cv::Mat>(num_frames_, rgb_frame));
    depth_images_.insert(std::pair<uint32_t, cv::Mat>(num_frames_, depth_frame));
    poses_.push_back(pose);
    num_frames_++;
}

#endif // DATASET_HPP