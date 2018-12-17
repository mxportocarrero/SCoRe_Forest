#ifndef DATASET_HPP
#define DATASET_HPP

// Esta clase se encarga de leer el archivo que contiene
// los archivos de los pares RGB-D sincronizados

#include "general_includes.hpp"
#include "linear_algebra_functions.hpp"
#include <map>

class Dataset
{
private:
    std::string database_name_;

    std::vector<std::string> timestamp_filenames_;
    std::vector<std::string> rgb_filenames_;
    std::vector<std::string> depth_filenames_;

    // Loaded images 
    // Vamos a cargar toda la secuencia de imagenes a la RAM
    // Para acelerar su procesamiento
    std::map<uint32_t, cv::Mat> rgb_images_;
    std::map<uint32_t, cv::Mat> depth_images_;
    std::vector<Mat44> poses_;

    int num_frames_ = 0;

public:
    Dataset(std::string folder_name);

    cv::Mat getRGB_filename(int frame);
    cv::Mat getDEPTH_filename(int frame);
     getTimestamp_filename(int frame);

    int NoFrames();

}; // Fin de Clase de Dataset

#endif // DATASET_HPP