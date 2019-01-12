/**
Estructura de clase Forest (includes)
//FOREST
	-> Kabsch Algorithm
	-> Tree
 		->Dataset
 			->general_includes
 			->linear_algebra_functions
 			->utilities
 		->Mean Shift
 		->Nodo
	 		->Feature // Solo es necesario agregar esta libreria para tener las otras
 				->random
 				->settings
 				->general_includes // No hay conflicto por que tambien solo son cabeceras
*/

#include "tree.hpp"
#include "kabsch.hpp"
#include <ctime>

template<typename D, typename RGB>
class Forest
{
private:
	// Datos preprocesados (sincronizados)
	Dataset * data_;
	// Configuracion del algoritmo
	Settings *settings_;
	// Bosque, vector de arboles
	std::vector< Tree<D,RGB>* > forest_;
	// Generador de numeros aleatorios
	Random *random_;
	// Archivo que guarda el entrenamiento del Regresion Forest
	const char* binaryFileHeader_ = "ISUE.RelocForest.Forest";

public:
	// Constructor basico de la clase Forest
	Forest(Dataset * data, Settings *settings)
		: data_(data), settings_(settings)
	{
		for (int i = 0; i < settings->num_trees_; ++i)
		{
			forest_.push_back(new Tree<D, RGB>());
		}
		random_ = new Random();
	} // Fin del Constructor

	// Init forest from binary file
	Forest(Dataset *data, Settings *settings, const std::string& path) 
		: data_(data), settings_(settings)
	{
		random_ = new Random();

		std::ifstream i(path, std::ios_base::binary);
		if (!i.is_open()) {
			throw std::runtime_error("Unable to open file");
		}
		this->Deserialize(i);
	} // Fin del Constructor

	~Forest()
	{
		delete random_;
		for (auto t : forest_)
			delete t;
		forest_.clear();
	} // Fin del destructor

	void Serialize(const std::string& path)
	{
		std::ofstream o(path, std::ios_base::binary);
		Serialize(o);
	} // Fin de Serialize

	void Serialize(std::ostream& stream)
	{
		const int majorVersion = 0, minorVersion = 0;

		stream.write(binaryFileHeader_, strlen(binaryFileHeader_));
		stream.write((const char*)(&majorVersion), sizeof(majorVersion));
		stream.write((const char*)(&minorVersion), sizeof(minorVersion));

		int treeCount = settings_->num_trees_;
		stream.write((const char*)(&treeCount), sizeof(treeCount));

		for (auto tree : forest_)
			tree->Serialize(stream);

		if (stream.bad())
			throw std::runtime_error("Forest serialization failed");
	} // Fin de Serialize

	void Deserialize(std::istream& stream)
	{
		std::vector<char> buffer(strlen(binaryFileHeader_) + 1);
		stream.read(&buffer[0], strlen(binaryFileHeader_));
		buffer[buffer.size() - 1] = '\0';

		if (strcmp(&buffer[0], binaryFileHeader_) != 0)
			throw std::runtime_error("Unsupported forest format.");


		const int majorVersion = 0, minorVersion = 0;
		stream.read((char*)(&majorVersion), sizeof(majorVersion));
		stream.read((char*)(&minorVersion), sizeof(minorVersion));

		int treeCount = 0;
		stream.read((char*)(&treeCount), sizeof(treeCount));

		int i = 0;
		for (i = 0; i < treeCount; ++i) {
			Tree<D, RGB> *t = new Tree<D, RGB>();
			t->Deserialize(stream);
			forest_.push_back(t);
		}
	} // Fin de Deserialize

	bool IsValid()
	{
		bool ret = true;
		for (auto tree : forest_) {
			ret = ret && tree->IsValid();
		}

		return ret;
	}

	// Train time
	/////////////

	std::vector<LabeledPixel> LabelData()
	{
		std::vector<LabeledPixel> labeled_data;

		// randomly choose frames
		for (int i = 0; i < settings_->num_frames_per_tree_; ++i) {
			int curr_frame = random_->Next(0, data_->getNumFrames());
			// randomly sample pixels per frame
			for (int j = 0; j < settings_->num_pixels_per_frame_; ++j) {
				int row = random_->Next(0, settings_->image_height_);
				int col = random_->Next(0, settings_->image_width_);

				// calc label
				cv::Mat rgb_image = data_->getRgbImage(curr_frame);
				cv::Mat depth_image = data_->getDepthImage(curr_frame);

				float Z = (float)depth_image.at<D>(row, col) / (float)settings_->depth_factor_;
				float Y = (row - settings_->cy) * Z / settings_->fy;
				float X = (col - settings_->cx) * Z / settings_->fx;

				Eigen::Vector3d p_camera(X, Y, Z);
				Pose pose = data_->getPose(curr_frame);
				Eigen::Vector3d label_e = poseRotation(pose) * p_camera + posePosition(pose);

				cv::Point3f label(label_e.x(), label_e.y(), label_e.z());

				// store labeled pixel
				LabeledPixel pixel(curr_frame, cv::Point2i(col, row), label);
				labeled_data.push_back(pixel);
			}
		}
		return labeled_data;
	} // Fin de la funcion LabelData


	void Train()
	{
		// train each tree individually
		int index = 1;
		for (auto t : forest_) {
			std::cout << "[Tree " << index << "] " << "Generating Training Data.\n";
			std::vector<LabeledPixel> labeled_data = LabelData();

			// train tree with set of pixels recursively
			std::cout << "[Tree " << index << "] " << "Training.\n";

			std::clock_t start;
			double duration;
			start = std::clock();

			t->Train(data_, labeled_data, random_, settings_);

			duration = (std::clock() - start) / (double)CLOCKS_PER_SEC;
			std::cout << "[Tree " << index << "] "
			        << "Train time: " << duration << " Seconds \n";
			index++;
		} // Fin de FOR

	} // Fin de la Funcion Train

	// Test time
	////////////

	class Hypothesis {
		public:
		//Eigen::Affine3d pose;
		Eigen::Transform<double,3,Eigen::Affine,Eigen::DontAlign> pose; // Tuvimos que redeclarar esta clase por q tenia error en ejecucion
		Eigen::Vector3d camera_space_point;
		Eigen::Matrix3Xd input;
		Eigen::Matrix3Xd output;
		uint32_t energy;

		bool operator < (const Hypothesis& h) const
		{
			return (energy < h.energy);
		}
	}; // Fin de la Declaracion de Hipotesis

	// Funcion Eval encargada de obtener la moda a partir de un solo pixel coordiante
	std::vector<Eigen::Vector3d> Eval(int row, int col, cv::Mat rgb_image, cv::Mat depth_image)
	{
		std::vector<Eigen::Vector3d> modes;

		for (auto t : forest_) {
		  bool valid = true;
		  Eigen::Vector3d m = t->Eval(row, col, rgb_image, depth_image, valid);
		  if (valid)
		    modes.push_back(m);
		}

		return modes;
	} // Fin de la Funcion

	// Funcion de error Top hat, que sirve para encontrar las modas
	uint8_t tophat_error(double val)
	{
		return !(val > 0 && val < 0.1);
	}

	// Funcion que se encarga de crear las hipotesis iniciales
	std::vector<Hypothesis> CreateHypotheses(int K_init, cv::Mat rgb_frame, cv::Mat depth_frame)
	{
		std::vector<Hypothesis> hypotheses;

		// sample initial hypotheses
		for (uint16_t i = 0; i < K_init; ++i) {
			Hypothesis h;
			// 3 points
			h.input.resize(3, 3);
			h.output.resize(3, 3);

			for (uint16_t j = 0; j < 3; ++j) {
				int col = 0, row = 0;
				double Z = 0.0;
				D test = 0;

				do {
					col = random_->Next(0, settings_->image_width_);
					row = random_->Next(0, settings_->image_height_);
					test = depth_frame.at<D>(row, col);
					Z = (double)test / (double)settings_->depth_factor_;
				} while (test == 0);

				double Y = (row - settings_->cy) * Z / settings_->fy;
				double X = (col - settings_->cx) * Z / settings_->fx;

				Eigen::Vector3d p_camera(X, Y, Z);
				h.camera_space_point = p_camera;
				// add to input
				h.input.col(j) = p_camera;

				std::vector<Eigen::Vector3d> modes = Eval(row, col, rgb_frame, depth_frame);

				if (modes.empty()) {
					--j;
				} else {
					Eigen::Vector3d point;
					if (modes.size() > 1)
						point = modes.at(random_->Next(0, modes.size() - 1));
					else
						point = modes.front();

					// add point to output
					h.output.col(j) = point;

				} // Fin de if else clause

			} // Fin de bucle FOR interior (Buscando 3 puntos validos)

			// kabsch algorithm
			Eigen::Affine3d transform = Find3DAffineTransform(h.input, h.output);
			h.pose = transform;
			h.energy = 0;
			hypotheses.push_back(h);
		} // Fin de bucle FOR de las Hipotesis

		return hypotheses;

	} // Fin de CreateHypothesis

	// Get pose hypothesis from depth and rgb frame
	Eigen::Affine3d Test(cv::Mat rgb_frame, cv::Mat depth_frame)
	{
		int K_init = 1024;
		int K = K_init;

		std::vector<Hypothesis> hypotheses = CreateHypotheses(K_init, rgb_frame, depth_frame);

		while (K > 1) {
			// sample set of B test pixels
			std::vector<cv::Point2i> test_pixels;
			int batch_size = 500; // todo put in settings

			// sample points in test image
			for (int i = 0; i < batch_size; ++i)  {
				int col = random_->Next(0, settings_->image_width_);
				int row = random_->Next(0, settings_->image_height_);
				test_pixels.push_back(cv::Point2i(col, row));
			}

			for (auto p : test_pixels) {
				// evaluate forest to get modes (union)
				auto modes = Eval(p.y, p.x, rgb_frame, depth_frame);

				D test = depth_frame.at<D>(p);
				double Z = (double)test / (double)settings_->depth_factor_;
				double Y = (p.y - settings_->cy) * Z / settings_->fy;
				double X = (p.x - settings_->cx) * Z / settings_->fx;

				Eigen::Vector3d p_camera(X, Y, Z);

				for (int i = 0; i < K; ++i) {
					double e_min = DBL_MAX; // Valor maximo del double
					Eigen::Vector3d best_mode;
					Eigen::Vector3d best_camera_p;

					for (auto mode : modes) {
						Eigen::Vector3d e = mode - hypotheses.at(i).pose * p_camera;
						double e_norm = e.norm();

						if (e_norm < e_min) {
							e_min = e_norm;
							best_mode = mode;
							best_camera_p = p_camera;
						}
					} // Fin del bucle de las modas

					// update energy
					hypotheses.at(i).energy += tophat_error(e_min);

					// inlier
					if (tophat_error(e_min) == 0) {
						if (test != 0) {
							// add to kabsch matrices
							hypotheses.at(i).input.conservativeResize(3, hypotheses.at(i).input.cols() + 1);
							hypotheses.at(i).input.col(hypotheses.at(i).input.cols() - 1) = best_camera_p;

							hypotheses.at(i).output.conservativeResize(3, hypotheses.at(i).output.cols() + 1);
							hypotheses.at(i).output.col(hypotheses.at(i).output.cols() - 1) = best_mode;
						}
					} // Fin del Condicional

				}// Fin de bucle FOR, testeo de las hipotesis

			} // Fin de bucle FOR testeo de los pixeles

			// sort hypotheses 
			std::sort(hypotheses.begin(), hypotheses.begin() + K);

			K = K / 2; // discard half

			// refine hypotheses (kabsch)
			for (int i = 0; i < K; ++i) {
				hypotheses.at(i).pose = Find3DAffineTransform(hypotheses.at(i).input, hypotheses.at(i).output);
			}
		} // Fin del bucle WHILE

		// return best pose and energy        
		return hypotheses.front().pose;

	} // Fin de la Funcion Test
	
}; // Fin de la Clase FOREST