#ifndef DATASET_HPP
#define DATASET_HPP

// Esta clase se encarga de leer el archivo que contiene
// los archivos de los pares RGB-D sincronizados

#include "general_includes.hpp"
#include "linear_algebra_functions.hpp"
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
    std::vector<std::string> timestamp_rgbd_;
    std::vector<std::string> timestamp_groundtruth_;

    // Loaded images 
    // Vamos a cargar toda la secuencia de imagenes a la RAM
    // Para acelerar su procesamiento
    std::map<uint32_t, cv::Mat> rgb_images_;
    std::map<uint32_t, cv::Mat> depth_images_;
    std::vector<Pose> poses_;

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

}

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