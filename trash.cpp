#include <string>
#include <iostream>

int main(int argc, char const *argv[])
{
	for (int i = 0; i < 1000; ++i)
	{
		std::string s;
		s = std::to_string(i);

		if(i < 10)
		{
			std::cout << "00000" + s << std::endl;
		}
		else if(i < 100)
		{
			std::cout << "0000" + s << std::endl;
		}
		else
		{
			std::cout << "000" + s << std::endl;
		}
	}
	return 0;
}