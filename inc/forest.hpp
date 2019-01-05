/**
Estructura de clase Forest
FOREST
	-> 
*/

#include "dataset.h"
#include "tree.h"


class Forest
{

private:
	Data * data_;
	Settings *settings;
	std::vecto<Tree<D,RGB>*> forest_;
	Random *random_;
	const char* binaryFileHeader = "ISUE.RelocForest.Forest"

public:
	Forest();
	~Forest();
	
};