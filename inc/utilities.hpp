#ifndef UTILITIES_HPP
#define UTILITIES_HPP

/**
Aquí pondremos algunas funciones útiles
**/

#include "general_includes.hpp"

// Función para mostrar
void show_depth_image(const cv::String & win_name, const cv::Mat & depth_img);

// Leer una imagen normal y la transforma a escala de grises [0,1]
bool read_image(cv::Mat & img,const cv::String & img_name);

// Leer una imagen de profundidad de datos en 16bit unsigned int
bool read_depth_image(cv::Mat & depth,const cv::String & img_name, const float & scalar_factor);

void writeMat2File(const cv::Mat & M, const char* file);

void writeEigenMat2File(const Eigen::MatrixXd & Mat, const char* file);

void writeEigenVec2File(const Eigen::VectorXd& Vec, const char* file);

// Funcion split para leer los datos

std::vector<std::string> split(const std::string& s, char separator);


#endif //UTILITIES_HPP
