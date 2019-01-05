#include "../inc/dataset.hpp"

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
        // Lectura de los datos RGBD
        // *************************
        while(std::getline(myRgbFile,RGBline) && std::getline(myDepthFile,DEPTHline)){
            if(RGBline[0] != '#'){ // Omitimos los comentarios, y solo usamos uno porq ya estan preprocesados
                timestamp_rgbd_.push_back( split(RGBline,' ')[0] );
                rgb_filenames_.push_back( split(RGBline,' ')[1] );
                depth_filenames_.push_back( split(DEPTHline,' ')[1] );
            }
        } // Fin de Bucle While

        // Lectura del Groundtruth
        // ***********************
        std::vector<std::string> temp;
        while(std::getline(myGroundtruthFile,Groundtruthline)){
            if(Groundtruthline[0] != '#'){
                temp = split(Groundtruthline,' ');
                timestamp_groundtruth_.push_back( temp[0] );
                poses_gt_.push_back(Quaternion_2_Mat44(std::stof(temp[1]),std::stof(temp[2]),std::stof(temp[3]),std::stof(temp[4]),std::stof(temp[5]),std::stof(temp[6]),std::stof(temp[7])));
            }
        }

        // Time Alignment
        // **************
        // Declaramos un vector de ternas para almacenar todas las combinaciones de timestamps
        // que tengan diferencias menores a un max_difference en tiempo
        double max_difference = 0.02; // en segundos es 2 decimas de segundo
        std::vector< std::tuple<double,int,int> > ternas;
        for (int i = 0; i < timestamp_rgbd_.size(); ++i){
            for (int j = 0; j < timestamp_groundtruth_.size(); ++j){
                double a = std::stod(timestamp_rgbd_[i]);
                double b = std::stod(timestamp_groundtruth_[j]);
                if(abs(a - b) < max_difference){
                    std::tuple<double,int,int> t(a-b,i,j);
                    ternas.push_back(t);
                }
            }
        } // Fin de bucle FOR

        // Ordenamos las tuplas de acuerdo al primer valor
        std::sort(ternas.begin(),ternas.end());

        // Guardaremos los indices para acceder a los vectores de poses
        std::vector< std::tuple<int,int> > matches;
        std::vector<int> tempStamps,tempStamps_gt;
        for (int i = 0; i < ternas.size(); ++i){
            double diff = std::get<0>(ternas[i]);
            int a = std::get<1>(ternas[i]);
            int b = std::get<2>(ternas[i]);
            auto it_a = std::find(tempStamps.begin(),tempStamps.end(),a);
            auto it_b = std::find(tempStamps_gt.begin(),tempStamps_gt.end(),b);

            // Este alineamiento necesariamente va a estar limitado por el vector mas pequeño
            if( it_a == tempStamps.end() &&  it_b == tempStamps_gt.end()){
                tempStamps.push_back(a);
                tempStamps_gt.push_back(b);
                matches.push_back( std::tuple<int,int>(a,b) );
            }
        } // Fin del bucle FOR

        // Ordenamos los matches (respecto a las imagenes RGBD)
        // Despues de esta linea el vector matches deberia estar ordenado
        std::sort(matches.begin(),matches.end());

        // Saving Data
        // ***********
        // Agregamos los datos a las estructuras finales
        for (int frame = 0; frame < matches.size(); ++frame){
            cv::Mat img_rgb = cv::imread(dataset_path_ + "/" + rgb_filenames_.at(frame));
            cv::Mat img_depth = cv::imread(dataset_path_ + "/" + depth_filenames_.at(frame), CV_LOAD_IMAGE_ANYDEPTH);
            addFrame(img_rgb,img_depth,poses_gt_[ std::get<1>( matches[frame] ) ] );
            if(frame % 100 == 0){
                std::cout.precision(2);
                std::cout << (float) frame / matches.size() * 100.0f << " percent Loaded\n";
            }
        }

    } else {

        std::cout << "Could not Open a File\n";

    } // Fin de la lectura y del relleno de los datos

} // Fin del constructor

cv::Mat Dataset::getRgbImage(int frame){
    if (rgb_images_.count(frame)) {
        return rgb_images_.at(frame);
    }
    else {
        cv::Mat img = cv::imread(dataset_path_ + rgb_filenames_.at(frame));
        rgb_images_.insert(std::pair<uint32_t, cv::Mat>(frame, img));
        return img;
    }
}

cv::Mat Dataset::getDepthImage(int frame){
    if (depth_images_.count(frame)) {
        return depth_images_.at(frame);
    }
    else {
        cv::Mat img = cv::imread(dataset_path_ + depth_filenames_.at(frame), CV_LOAD_IMAGE_ANYDEPTH);
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