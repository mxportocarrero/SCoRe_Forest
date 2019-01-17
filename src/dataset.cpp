#include "../inc/dataset.hpp"

// Funcion especial para leer un archivo pose
Pose read_pose(const std::string &pose_file)
{
    Pose pose;

    std::ifstream myFile;

    myFile.open(pose_file);

    // La forma en que es almacenada los poses es como una matriz de 4 x 4
    if (!myFile.is_open())
    {
        std::cout << "File couldnt be opened\n" << std::endl;
    }

    std::string t1, t2; double d; int cont = 0;
    while(myFile >> t1)
    {

        t2 = t1.substr( t1.size() - 4, 4);
        t1 = t1.erase( t1.size() - 5);

        d = std::stof(t1);

        //std::cout << t1 << " " << t2 << std::endl;

        if (t2[0] == '+')
        {
            d = d * pow(10.0, (int)(t2[3] - '0') );
        }
        else
        {
            d = d / pow(10.0, (int)(t2[3] - '0') );
        }

        //std::cout.precision(6);
        //std::cout << d << std::endl;

        pose.m[cont / 4][ cont % 4] = d;
        cont++;
    }

    return pose;
}

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
        // Los datos RGBD ya se encuentran alineados
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
        // estas ternas se conforman de la siguiente manera
        // < diff_in_time , indice_rgbd , indice_gt >
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
        std::vector<int> tempStamps,tempStamps_gt; // Estos vectores es para no agregar valores repetidos
        for (int i = 0; i < ternas.size(); ++i){
            double diff = std::get<0>(ternas[i]);
            int a = std::get<1>(ternas[i]);
            int b = std::get<2>(ternas[i]);

            // buscando agregar matches no repetidos
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

            addFrame(img_rgb,
                     img_depth,
                     timestamp_groundtruth_[ std::get<1>(matches[frame]) ],
                     poses_gt_[ std::get<1>(matches[frame]) ]);

            if(frame % 100 == 0){
                std::cout.precision(2);
                std::cout << (float) frame / matches.size() * 100.0f << " percent Loaded\n";
            }
        }

    } else {

        std::cout << "Could not Open a File\n";

    } // Fin de la lectura y del relleno de los datos

} // Fin del constructor // Lector del Dataset del TUM

// Constructor para el 7 scenes
Dataset::Dataset(std::string dataset_path, int dataset_type):
    dataset_path_(dataset_path)
{
    // Una entrada seria del tipo "data/7_scenes/chess"

    // Tenemos que leer los archivos TrainSplit.txt y TestSplit.txt
    std::ifstream trainFile;
    std::ifstream testFile;

    trainFile.open(dataset_path_ + "/TrainSplit.txt");
    testFile.open(dataset_path_ + "/TestSplit.txt");

    if(trainFile.is_open() && testFile.is_open())
    {
        std::string temp_line; // String temporal para guardar los datos

        // Leemos las secuencias de entrenamiento
        // De estos archivos solo nos interesa capturar el indice que esta en el
        // ultimo caracter de las lineas
        while( std::getline(trainFile,temp_line) )
        {
            //std::cout << temp_line << std::endl << " " << temp_line[ temp_line.size() -2 ] << std::endl;
            train_sequences.push_back( (int)( temp_line[ temp_line.size() - 2 ] - '0' ) );
        }

        while( std::getline(testFile,temp_line) )
        {
            test_sequences.push_back( (int)( temp_line[ temp_line.size() - 2 ] - '0' ) );
        }

        std::cout << "Training Sequences: ";
        for (int i = 0; i < train_sequences.size(); ++i)
            std::cout << train_sequences[i] << ",";
        std::cout << std::endl;

        std::cout << "Testing Sequences: ";
        for (int i = 0; i < test_sequences.size(); ++i)
            std::cout << test_sequences[i] << ",";
        std::cout << std::endl;

        // Cada secuencia de imagenes esta separado en sub-secuencias de 1000
        // En cada frame se encuentran anotados < RGB, Depth, Pose >
        // de la sgte forma:
        //      -> frame-000000.color.png
        //      -> frame-000000.depth.png
        //      -> frame-000000.pose.txt

        // Lo primero que haremos sera leer toda la secuencia sin importar si son
        // frames para training o para test
        std::string s,s_rgb,s_depth,s_pose;
        for (int i = 0; i < train_sequences.size() + test_sequences.size(); ++i)
        {
            // Leemos los paquetes de los frames
            for (int frame = 0; frame < 1000; ++frame)
            {
                // Construyendo los strings para la lectura
                s = std::to_string(frame);

                if(frame < 10)
                {
                    s = "00000" + s;
                }
                else if(frame < 100)
                {
                    s = "0000" + s;
                }
                else
                {
                    s = "000" + s;
                }

                s_rgb = dataset_path_ + "/seq-0"+ std::to_string(i+1) +"/frame-" + s + ".color.png";
                s_depth = dataset_path_ + "/seq-0"+ std::to_string(i+1) +"/frame-" + s + ".depth.png";
                s_pose = dataset_path_ + "/seq-0"+ std::to_string(i+1) +"/frame-" + s + ".pose.txt";

                //std::cout << s_rgb << std::endl << s_depth << std::endl << s_pose << std::endl;
                

                cv::Mat img_rgb = cv::imread( s_rgb );
                cv::Mat img_depth = cv::imread( s_depth, CV_LOAD_IMAGE_ANYDEPTH );

                // Tenemos que corregir las imagenes depth
                for (int row = 0; row < 640; ++row)
                {
                    for (int col = 0; col < 480; ++col)
                    {
                        if (img_depth.at<ushort>(col,row) == 65535)
                        {
                            img_depth.at<ushort>(col,row) = 0;
                        }
                    }
                }

                Pose pose = read_pose(s_pose);

                addFrame(img_rgb,
                         img_depth,
                         "00000000", // Este Dataset no tiene timestamps
                         pose);
                

            } // Fin de FOR

            std::cout << "Secuencia " << i << " Cargada\n";

        } // Fin de FOR // Lectura de todas subsecuencias

    } // Fin de IF

} // Fin del Constructor // Lector del 7scenes

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

std::string Dataset::getTimestamp(int frame)
{
    return timestamp_.at(frame);
}

Pose Dataset::getPose(int frame){
    return poses_.at(frame);
}

int Dataset::getNumFrames(){
    return num_frames_;
}

void Dataset::addFrame(cv::Mat rgb_frame, cv::Mat depth_frame, std::string timestamp, Pose pose){
    rgb_images_.insert(std::pair<uint32_t, cv::Mat>(num_frames_, rgb_frame));
    depth_images_.insert(std::pair<uint32_t, cv::Mat>(num_frames_, depth_frame));
    timestamp_.push_back(timestamp);
    poses_.push_back(pose);
    num_frames_++;
}
