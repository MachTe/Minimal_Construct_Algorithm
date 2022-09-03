#include "MCA.h"

int main()
{
	std::vector<PolygonMCA> polygons;
	PolygonMCA CurrentPolygon;
	PolygonMCA EmptyPolygon;
	
	//LOADING POLYGONS FROM TXT BEGIN
	std::string tmp;
	std::string tmp2;
	std::string filename = "../polygons/TestPolygons.txt";

	std::ifstream mapPolygons(filename);

	while (getline(mapPolygons, tmp)) {
		if (tmp == "MAIN")
		{
			std::cout << "main\n";
		}
		else if (tmp == "HOLE")
		{
			CurrentPolygon.vertices.push_back(CurrentPolygon.vertices[0]);
			polygons.push_back(CurrentPolygon);
			CurrentPolygon = EmptyPolygon;
		}
		else {
			getline(mapPolygons, tmp2);
			Point x = { std::stoi(tmp) / 100.0, std::stoi(tmp2) / 100.0 };
			CurrentPolygon.vertices.push_back(x);
		}
	}
	CurrentPolygon.vertices.push_back(CurrentPolygon.vertices[0]);
	polygons.push_back(CurrentPolygon);

	mapPolygons.close();
	//LOADING POLYGONS FROM TXT END

	VertexInGraph S, T;
	
	T.posx = -33.45;
	T.posy = -17.36;
	S.posx = 15.0;
	S.posy = 80.0;
	
	std::vector<Point> l = FindDirectionOfTravel(polygons, S, T);

	std::ofstream outFile;
	outFile.open(filename, std::ios_base::app);
	outFile << "\nPATH\n";

	while (!l.empty())
	{
		outFile << l.back().x() * 100.0 << "\n";
		outFile << l.back().y() * 100.0 << "\n";
		l.pop_back();
	}

	outFile.close();
	
	return 0;
}
