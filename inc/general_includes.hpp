#ifndef GENERAL_INCLUDES_HPP
#define GENERAL_INCLUDES_HPP

// Import Standar Libraries
#include <string>
#include <iostream>
#include <fstream>
#include <vector>

// Import Intrinsic Directives
#include <immintrin.h>

// Import OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>






// Import Eigen
#include <eigen3/Eigen/Dense>
#include <eigen3/unsupported/Eigen/MatrixFunctions> // Para hacer uso de esta libreria
// tuve que modificar algunos includes en el source code que utiliza por q referenciaba
// #include <Eigen/Core> pero deberia ser en mi caso #include <eigen3/Eigen/Core>





// Precision de las imagenes, podemos alternar entre floats o dobles
//#define Double_Precision

#ifdef Double_Precision
typedef double myNum;
#else
typedef float myNum;
#endif // Double_Precision



/** MACROS **/

#define FOR(i,n) for(int i = 0; i < n; ++i)



#endif // GENERAL_INCLUDES_HPP
