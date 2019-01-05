#ifndef LINEAR_ALGEBRA_FUNCTIONS_HPP
#define LINEAR_ALGEBRA_FUNCTIONS_HPP

#include "general_includes.hpp"

/*  // Incluir estas lineas para testear las funciones de transformacion
    Eigen::Vector3d w(4,5,6);
    Eigen::Matrix3d R = vector2rotation(w);
    w = rotation2vector(R);
    vector2rotation(w);

    Eigen::VectorXd xi(6);
    xi << 1,2,3,4,5,6;
    Eigen::Matrix4d g = twistcoord2rbm(xi);
    xi = rbm2twistcoord(g);
    twistcoord2rbm(xi);
*/

Eigen::Matrix3d hat(const Eigen::Vector3d & w);

// Transformaciones de Rotacion
Eigen::Matrix3d vector2rotation(const Eigen::Vector3d & w);

Eigen::Vector3d rotation2vector(const Eigen::Matrix3d & R);

// Transformaciones de Rigid-Body Motions

Eigen::Matrix4d twistcoord2rbm(const Eigen::VectorXd & xi);

Eigen::VectorXd rbm2twistcoord(const Eigen::Matrix4d & g);

// p & q must have same size
Eigen::Matrix4d QuickTransformation(const std::vector<Eigen::Vector3d> &p,const std::vector<Eigen::Vector3d> &q);

//#################################################################################
//#################################################################################
//#################################################################################


// Declaring some Matrix Arithmetic.
// examples taken from https://gist.github.com/rygorous/4172889

// La razón por la que declaramos nuestra matrix como union, es para poder interpretar nuestra
// data tanto como un doble arreglo de floats o un arreglo de vectores intrinsecos para SIMD
union Mat44 {
    float m[4][4];
    __m128 row[4];
};

typedef Mat44 Pose; // Para mejorar el readibility

void printMat44(const Mat44 &mat, const char* name);

// Tengamos en cuenta que una multiplicación de matrices puede llevarse a cabo de diferentes formas
// en nuestro caso, expresamos este proceso mediante combinaciones lineales(linear combination). Ver agenda
static inline __m128  lincomb_SSE(const __m128 &a, const Mat44 &B);

void matmul_SSE(Mat44 &out, const Mat44 &A, const Mat44 &B);

// Esta funcion usa los intrinsecos AVX, aprovechando la extension a 256 bits de los registros YMM
// Como son matrices 4x4, podemos aprovechar ello y hacer calculos duales
// Es decir de 2 en 2 filas
static inline __m256 dual_lincomb_AVX8(__m256 A01, const Mat44 &B);

void dual_matmul_AVX8(Mat44 &out, const Mat44 &A, const Mat44 &B);

//Realizamos un enum para tener un mejor acceso a las variables
struct Quaternion{
    float data[4];
    float x() const {return data[0];}
    float y() const {return data[1];}
    float z() const {return data[2];}
    float w() const {return data[3];}

    Quaternion(const float &qx,const float &qy,const float &qz,const float &qw){
        data[0] = qx;
        data[1] = qy;
        data[2] = qz;
        data[3] = qw;
    }
};

// Conversion functions
// Funcion que devuelve el objeto Mat44 a partir de un arreglo de 4 floats(unit quaternion)
Mat44 Quaternion_2_Mat44(const Quaternion &q);

// Funcion sobrecargada que acepta tambien un vector de traslación
Mat44 Quaternion_2_Mat44(const float &tx, const float &ty, const float &tz, const float &qx,const float &qy,const float &qz,const float &qw);

Mat44 TwistCoord_2_Mat44(const float &v1, const float &v2, const float &v3, const float &w1,const float &w2,const float &w3);

#endif // LINEAR_ALGEBRA_FUNCTIONS_HPP