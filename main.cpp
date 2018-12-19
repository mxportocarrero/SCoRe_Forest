/**
Implementacion de Regression Forest

g++ main.cpp src/dataset.cpp -o reg_forest `pkg-config opencv --cflags --libs` && ./reg_forest

**/

#include "inc/utilities.hpp"
#include "inc/dataset.hpp"

#define DATABASE_NAME "data/rgbd_dataset_freiburg11_room"

int main(int argc, char const *argv[])
{
	Dataset myDataset(DATABASE_NAME);

	std::cout << "Hola Mundo!\n";

	return 0;
}