#include "MCA.h"

int main()
{
	std::vector<PolygonMCA> polygons;
	PolygonMCA CurrentPolygon;
	PolygonMCA EmptyPolygon;
	std::string tmp;
	std::string tmp2;
	std::string filename = "../polygons/TestPolygons.txt";

	//LOAD CITIES
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

	VertexInGraph S, T;
	
	T.posx = -33.45;
	T.posy = -17.36;
	S.posx = 15.0;
	S.posy = 80.0;
	
	auto start = std::chrono::high_resolution_clock::now();
	std::vector<Point> l = FindDirectionOfTravel(polygons, S, T);
	auto stop = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);

	std::cout << "Time taken by function: " << duration.count() << " microseconds" << std::endl;

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

	//input just to force the command prompt to stay opened
	int f;
	std::cin >> f;
	return 0;
}