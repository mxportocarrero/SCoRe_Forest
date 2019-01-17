#pragma once

#include "general_includes.hpp"
#include "settings.hpp"
#include "random.hpp"
#include "utilities.hpp"


// Clase Feature
// -------------
// 		->random
// 		->settings
// 		->general_includes // No hay conflicto por que tambien solo son cabeceras

// Esta forma parte de los nodos


class Feature
{
//protected:
public:
	cv::Point2i offset_1_;
	cv::Point2i offset_2_;
public:
	Feature()
	{
		offset_1_ = cv::Point2i(0,0);
		offset_2_ = cv::Point2i(0,0);
	}
	Feature(cv::Point2i offset_1, cv::Point2i offset_2) : offset_1_(offset_1), offset_2_(offset_2){};
	virtual float GetResponse(cv::Mat depth_image, cv::Mat rgb_image, cv::Point2i pos, Settings &settings, bool &valid) = 0;	
};

// Especializacion de funcionces

// DEPTH FEATURE RESPONSE FUNCTIONS
// --------------------------------

// La razon por la que lo declaramos en forma de template
// Es por q se hace mas facil si quisieras cambiar y usar otro tipo de features
template <typename D, typename RGB>
class Depth : public Feature
{
public:
	Depth(cv::Point2i offset_1, cv::Point2i offset_2) : Feature(offset_1, offset_2){};

	/** Funcion encargada de devolver el valor response
	Input:
		el par RGBD, punto en pixel coordinates y una referencia a un bool para validar la operacion
	Output:
		double con el valor calculado, segun formula. revisar el paper
	*/
	virtual float GetResponse(cv::Mat depth_image, cv::Mat rgb_image, cv::Point2i pos, bool &valid)
		{
		D depth_at_pos = depth_image.at<D>(pos);
		cv::Point2i depth_inv_1(offset_1_.x / depth_at_pos, offset_2_.y / depth_at_pos);
		cv::Point2i depth_inv_2(offset_2_.x / depth_at_pos, offset_2_.y / depth_at_pos);

		if (depth_at_pos == 0)
			valid = false;

		D D_1 = depth_image.at<D>(pos + depth_inv_1);
		D D_2 = depth_image.at<D>(pos + depth_inv_2);

		return D_1 - D_2;
	}      
}; // Fin de la definicion de Clase Depth

template <typename D, typename RGB>
class DepthAdaptiveRGB : public Feature
{
//protected:
public:
	int color_channel_1_, color_channel_2_;
	float tau_;	
public:	
	DepthAdaptiveRGB()
	{
		color_channel_1_ = 0;
		color_channel_2_ = 0;
		tau_ = 0;
	} // Fin de Contructor Default

	DepthAdaptiveRGB(cv::Point2i offset_1, cv::Point2i offset_2, int color_channel_1, int color_channel_2, float tau)
		: Feature(offset_1, offset_2), color_channel_1_(color_channel_1), color_channel_2_(color_channel_2), tau_(tau){};

	static DepthAdaptiveRGB CreateRandom(Random *random)
	{
		cv::Point2i offset_1(random->Next(-130, 130), random->Next(-130, 130)); // Value from the paper -- +/- 130 pixel meters
		cv::Point2i offset_2(random->Next(-130, 130), random->Next(-130, 130));
		int color_channel_1 = random->Next(0, 2);
		int color_channel_2 = random->Next(0, 2);
		int tau = random->Next(-128, 128); // Revisar esto en el paper !!!!
		return DepthAdaptiveRGB(offset_1, offset_2, color_channel_1, color_channel_2, tau);
	}

	virtual float GetResponse(cv::Mat depth_img, cv::Mat rgb_img, cv::Point2i pos, Settings &settings, bool &valid) override
	{
		D depth_at_pos = depth_img.at<D>(pos);
		float depth = (float)depth_at_pos;
		//std::cout << "pixel depth: " << depth << std::endl;
		//std::cout << "depth factor: " << settings.depth_factor_ << ":" << (float)settings.depth_factor_ << std::endl;

		if (depth <= 0) {
			valid = false;
			return 0.0;
		} else {
			depth /= (float)settings.depth_factor_; // scale value
		}

		cv::Point2i depth_inv_1(offset_1_.x / depth, offset_2_.y / depth);
		cv::Point2i depth_inv_2(offset_2_.x / depth, offset_2_.y / depth);

		cv::Point2i pos1 = pos + depth_inv_1;
		cv::Point2i pos2 = pos + depth_inv_2;

		int width = settings.image_width_;
		int height = settings.image_height_;

		/*// depth invariance
		// Este codigo sirve para observar como se comportan los offsets
		std::cout << "pixel depth / depth factor: " << depth << std::endl;

		std::cout << "Central pixel: " << pos.x << "," << pos.y << "\n";

		std::cout << "Offset 1: " << offset_1_.x << "," << offset_1_.y << " Offset 2: " << offset_2_.x << "," << offset_2_.y << std::endl;
		std::cout << "Depth_inv_1: " << depth_inv_1.x << "," << depth_inv_1.y << " Depth_inv_2: " << depth_inv_2.x << "," << depth_inv_2.y << std::endl;
		std::cout << "pos1: " << pos1.x << "," << pos1.y << " pos2: " << pos2.x << "," << pos2.y << std::endl;

		cv::namedWindow("Display Depth",cv::WINDOW_AUTOSIZE);

		cv::Mat m = depth_img.clone();
		// Centro
		cv::circle(m,pos,5,cv::Scalar(255,255,255));
		// Offsets
		cv::circle(m,pos1,10,cv::Scalar(255,255,255));
		cv::circle(m,pos2, 10,cv::Scalar(255,255,255));
		cv::circle(m,pos,130,cv::Scalar(255,255,255));

		show_depth_image("Display Depth",m);
		cv::waitKey();
		
		*/

		// check bounds
		if (pos1.x >= width || pos1.y >= height ||
			pos1.x < 0.0    || pos1.y < 0.0 ) {
			valid = false;
			return 0.0f;
		}
		if (pos2.x >= width || pos2.y >= height ||
			pos2.x < 0.0    || pos2.y < 0.0) {
			valid = false;
			return 0.0f;
		}

		float I_1 = rgb_img.at<RGB>(pos1)[this->color_channel_1_];
		float I_2 = rgb_img.at<RGB>(pos2)[this->color_channel_2_];

		return I_1 - I_2;
	} // Fin de la Funcion GetResponse

	float GetThreshold()
	{
		return tau_;
	} // Fin de la funcion Get Threshold

	//no deja espacio cuando imprime
	void printOffsets()
	{
		std::cout << "offset1: " << offset_1_.x << ","<< offset_1_.y
				  << " offset2: " << offset_2_.x << ","<< offset_2_.y;
	}


}; // Fin de la Declaracion de la Clase Depth AdaptiveRGB