#pragma once

#include "general_includes.hpp"
#include "settings.hpp"
#include "random.hpp"


// Clase Feature
// -------------
// 		->random
// 		->settings
// 		->general_includes // No hay conflicto por que tambien solo son cabeceras

// Esta forma parte de los nodos


class Feature
{
protected:
	cv::Point2i offset_1_;
	cv::Point2i offset_2_;
public:
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
protected:
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
		int tau_ = random->Next(-128, 128);
		return DepthAdaptiveRGB(offset_1, offset_2, color_channel_1, color_channel_2, tau_);
	}

	virtual float GetResponse(cv::Mat depth_img, cv::Mat rgb_img, cv::Point2i pos, Settings &settings, bool &valid) override
	{
		D depth_at_pos = depth_img.at<D>(pos);
		float depth = (float)depth_at_pos;

		if (depth <= 0) {
			valid = false;
			return 0.0;
		} else {
			depth /= settings.depth_factor_; // scale value
		}

		// depth invariance
		cv::Point2i depth_inv_1(offset_1_.x / depth, offset_2_.y / depth);
		cv::Point2i depth_inv_2(offset_2_.x / depth, offset_2_.y / depth);

		cv::Point2i pos1 = pos + depth_inv_1;
		cv::Point2i pos2 = pos + depth_inv_2;

		int width = settings.image_width_;
		int height = settings.image_height_;

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
}; // Fin de la Declaracion de la Clase Depth AdaptiveRGB