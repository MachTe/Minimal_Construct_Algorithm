#include <iostream>
#include <queue>
#include <map>
#include <deque>
#include <typeinfo>
#include <fstream>
#include <chrono>

#include <boost/geometry/geometry.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/adapted/c_array.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/multi/geometries/multi_polygon.hpp>
#include <boost/geometry/core/point_type.hpp>


//Geometry models based on the boost librarary
typedef boost::geometry::model::d2::point_xy<double> Point;
typedef boost::geometry::model::linestring<Point> linestring;
typedef boost::geometry::model::polygon<Point> polygon;


//Vertex structure used to store data
struct VertexInGraph
{
	bool closed = false;
	bool open = false;
	double posx = NULL;
	double posy = NULL;
	double cost = DBL_MAX;
	double heruistic = 0.0;
	double priority = DBL_MAX;

	VertexInGraph* parent = NULL;
	std::vector<VertexInGraph*> neighbors;

	//deleting specific neigbour
	void RemoveNeighbor(VertexInGraph* v)
	{
		std::vector<VertexInGraph*>::iterator position = std::find(neighbors.begin(), neighbors.end(), v);
		if (position != neighbors.end())
		{
			neighbors.erase(position);
		}
	}

	//setting distance from the vertex to the final destination whose coordinates have to be inserted
	void setHeruistic(double x, double y) {
		heruistic = sqrt(pow(x - posx, 2) + pow(y - posy, 2));
	}
};


//Graph as deffined in the source pdf
struct Graph
{
	std::vector<VertexInGraph> vertices;
};


//Used to compare priority of items in a priority queue of type:
//std::priority_queue<VertexInGraph*, std::vector<VertexInGraph*>, ComparePriority>
struct ComparePriority {
	bool operator()(VertexInGraph* p1, VertexInGraph* p2)
	{
		return (*p1).priority > (*p2).priority;
	}
};


//Measuring distance between vertices
double VertexDistance(VertexInGraph* a, VertexInGraph* b)
{
	double dis = sqrt(pow((*a).posx - (*b).posx, 2) + pow((*a).posy - (*b).posy, 2));
	return dis;
}


//Polygon typer used to store polygons as described in the source pdf
struct PolygonMCA
{
	bool closed = false;
	std::vector<Point> vertices;
};


//Function used to compare two doubles
bool AreSame(double a, double b)
{
	return fabs(a - b) < 0.000001;
}


//Function which finds whether the inserted line (u, v) intersects any of the polygons
//returns the position of the intersected polygon
unsigned short LineIntersectionTest(VertexInGraph* u, VertexInGraph* v, std::vector<PolygonMCA>* polygons)
{
	polygon barrier;
	polygon line;
	std::deque<Point> output;
	double minDistance = DBL_MAX;
	double ax = NULL;
	double ay = NULL;
	unsigned short intersectedPolygon = USHRT_MAX;
	unsigned short intersectionCount;

	//sifts through all of the polygons
	for (unsigned short i = 0; i < (*polygons).size(); i++)
	{
		std::vector<Point> lineVec;
		Point x = { (*u).posx, (*u).posy };
		Point y = { (*v).posx, (*v).posy };
		lineVec.push_back(x);
		lineVec.push_back(y);
		lineVec.push_back(x);
		boost::geometry::assign_points(line, lineVec);

		boost::geometry::assign_points(barrier, (*polygons)[i].vertices);

		boost::geometry::intersection(barrier, line, output);

		intersectionCount = output.size();

		//goes through all of the intersected points and finds the closest one
		//due to a few quirks in the boost library we have to delete unwanted vertices
		while (!output.empty())
		{
			ax = output.front().x();
			ay = output.front().y();

			//the only intersection lies on the border of the polygon
			if (intersectionCount == 1)
			{
				if ((AreSame(ax, x.x()) && AreSame(ay, x.y())) || (AreSame(ax, y.x()) && AreSame(ay, y.y())))
				{
					output.pop_front();
					break;
				}
			}

			//the only two intersections lie on the the same edge of the polygon
			if ((intersectionCount == 2) && (output.size() == 2))
			{
				for (int l = 0; l < (*polygons)[i].vertices.size() - 1; l++)
				{
					bool statement1 = ((AreSame(barrier.outer()[l].x(), x.x()) && AreSame(barrier.outer()[l].y(), x.y()))) || ((AreSame(barrier.outer()[l].x(), y.x()) && AreSame(barrier.outer()[l].y(), y.y())));
					bool statement2 = ((AreSame(barrier.outer()[l + 1].x(), x.x()) && AreSame(barrier.outer()[l + 1].y(), x.y()))) || ((AreSame(barrier.outer()[l + 1].x(), y.x()) && AreSame(barrier.outer()[l + 1].y(), y.y())));
					if (statement1 && statement2)
					{
						output.pop_front();
						output.pop_front();
						break;
					}
				}
			}

			//if the intersection lies on ne of the vertices of the polygon
			if (((AreSame(ax, x.x()) && AreSame(ay, x.y())) || (AreSame(ax, y.x()) && AreSame(ay, y.y()))) && !output.empty())
			{
				output.pop_front();
				continue;
			}

			//selects the shortest distance to the intersection of the polygon
			if (sqrt(pow(ax - (*u).posx, 2) + pow(ay - (*u).posy, 2)) < minDistance && !output.empty())
			{
				minDistance = sqrt(pow(ax - (*u).posx, 2) + pow(ay - (*u).posy, 2));
				intersectedPolygon = i;
			}
			if (!output.empty())
			{
				output.pop_front();
			}
		}
	}
	return intersectedPolygon;
}


//Method Find Parent as described in the source pdf
void FindParent(VertexInGraph* v, std::priority_queue<VertexInGraph*, std::vector<VertexInGraph*>, ComparePriority>*& q)
{
	double minPathCost = DBL_MAX;
	VertexInGraph* u = NULL;
	VertexInGraph* v_i = NULL;

	for (unsigned short i = 0; i < (*v).neighbors.size(); i++)
	{
		v_i = (*v).neighbors[i];
		if ((*v_i).closed)
		{
			if ((*v_i).cost + VertexDistance(v, v_i) < minPathCost)
			{

				minPathCost = (*v_i).cost + VertexDistance(v, v_i);
				u = v_i;

			}
		}
	}
	if (u != NULL) {

		(*v).parent = u;
		(*v).cost = (*u).cost + VertexDistance(v, u);
		(*v).priority = (*v).cost + (*v).heruistic;
		(*q).push(v);
		(*v).open = true;

	}
	else
	{
		(*v).open = false;
	}
	
}


//Method which connects all vertices with their parents and adds them to the Path
void ExtractPath(VertexInGraph* v, std::vector<Point>* PathPtr, Graph* G, std::priority_queue<VertexInGraph*, std::vector<VertexInGraph*>, ComparePriority>*& q, std::vector<PolygonMCA> polygons)
{
	VertexInGraph* oldParent;
	unsigned short p;
	Point x;

	while (v != &((*G).vertices[0]))
	{
		oldParent = (*v).parent;
		FindParent(v, q);
		p = LineIntersectionTest((*v).parent, v, &polygons);
		if (p == USHRT_MAX)
		{
			x = { (*v).posx, (*v).posy };
			(*PathPtr).push_back(x);
			v = (*v).parent;
		}
		else {
			(*v).RemoveNeighbor((*v).parent);
			(*((*v).parent)).RemoveNeighbor(v);

			(*v).parent = NULL;
			FindParent(v, q);
		}
	}
	x = { (*G).vertices[0].posx, (*G).vertices[0].posy };
	(*PathPtr).push_back(x);
}


//Function which determines whether vertex P[i] si convex
bool IsConvex(unsigned short i, PolygonMCA* P)
{
	unsigned short a, c;
	if (i == 0) {
		a = (*P).vertices.size() - 2;
		c = 1;
	}
	else if (i == (*P).vertices.size() - 1)
	{
		c = 1;
		a = i - 1;
	}
	else
	{
		a = i - 1;
		c = i + 1;
	}

	double ax = (*P).vertices[a].x();
	double ay = (*P).vertices[a].y();
	double bx = (*P).vertices[i].x();
	double by = (*P).vertices[i].y();
	double cx = (*P).vertices[c].x();
	double cy = (*P).vertices[c].y();

	if (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by) >= 0.0)
	{
		return true;
	}
	else
	{
		return false;
	}
}


//Function determines whether v_i is tangential to the vertex (corner) P[i]
bool IsTangential(VertexInGraph* v_i, unsigned short i, PolygonMCA* P)
{
	unsigned short a, c;
	if (i == 0) {
		a = (*P).vertices.size() - 2;
		c = 1;
	}
	else if (i == (*P).vertices.size() - 1)
	{
		c = 1;
		a = i - 1;
	}
	else
	{
		a = i - 1;
		c = i + 1;
	}

	double ax = (*P).vertices[a].x();
	double ay = (*P).vertices[a].y();
	double bx = (*P).vertices[i].x();
	double by = (*P).vertices[i].y();
	double cx = (*P).vertices[c].x();
	double cy = (*P).vertices[c].y();
	double dx = (*v_i).posx;
	double dy = (*v_i).posy;

	if (dx * (by - cy) + bx * (cy - dy) + cx * (dy - by) >= -0.01 && dx * (by - ay) + bx * (ay - dy) + ax * (dy - by) >= -0.01)
	{
		return true;
	}
	else if (dx * (by - cy) + bx * (cy - dy) + cx * (dy - by) <= 0.01 && dx * (by - ay) + bx * (ay - dy) + ax * (dy - by) <= 0.01)
	{
		return true;
	}
	else
	{
		return false;
	}
}


//Method Connect Obstacle as described in the source pdf
void ConnectObstacle(PolygonMCA* P, Graph* G, std::priority_queue<VertexInGraph*, std::vector<VertexInGraph*>, ComparePriority>*& q, std::vector<PolygonMCA>* polygons)
{
	for (unsigned short i = 0; i < (*P).vertices.size() - 1; i++)
	{
		VertexInGraph* v_ii = NULL;
		VertexInGraph* v_j = NULL;
		VertexInGraph newVertex;
		VertexInGraph emptyVertex;
		newVertex.posx = (*P).vertices[i].x();
		newVertex.posy = (*P).vertices[i].y();

		if (IsConvex(i, P))
		{
			newVertex.setHeruistic((*G).vertices[1].posx, (*G).vertices[1].posy);
			
			(*G).vertices.push_back(newVertex);
			newVertex = emptyVertex;
			v_ii = &((*G).vertices.back());
			
			for (unsigned short k = 0; k < (*G).vertices.size() -1; k++)
			{
				v_j = &((*G).vertices[k]);
				if (IsTangential(v_j, i, P)) {
					(*v_j).neighbors.push_back(v_ii);
					(*v_ii).neighbors.push_back(v_j);
				}
			}

			FindParent(&((*G).vertices.back()), q);

		}
	}
}


//Function implementing the Minimal Construct Algorith as described in the source pdf
//returns the shortest path from the start vertex S to the vertex T
std::vector<Point> FindDirectionOfTravel(std::vector<PolygonMCA> polygons, VertexInGraph S, VertexInGraph T)
{
	unsigned short p = NULL;

	VertexInGraph* u = NULL;
	VertexInGraph* v = NULL;
	VertexInGraph* v_i = NULL;

	std::vector<Point> Path;

	std::priority_queue<VertexInGraph*, std::vector<VertexInGraph*>, ComparePriority> q;	//We are using a priority queue q
	std::priority_queue<VertexInGraph*, std::vector<VertexInGraph*>, ComparePriority>* qptr = &q;

	Graph G;																				//Start with an empty graph
	G.vertices.reserve(2000);

	G.vertices.push_back(S);																//Add start and target vertices
	G.vertices.push_back(T);

	VertexInGraph* Sptr = &(G.vertices[0]);
	VertexInGraph* Tptr = &(G.vertices[1]);
	(*Sptr).neighbors.push_back(Tptr);														//Add T to the neighbours of S
	(*Tptr).neighbors.push_back(Sptr);
	//
	(*Sptr).heruistic = VertexDistance(Sptr, Tptr);
	(*Tptr).heruistic = 0.0;

	(*Tptr).parent = Sptr;																	//Set S as the parent of T
	(*Sptr).closed = true;																	//Close the start vertex
	(*Sptr).cost = 0.0;

	q.push(&(G.vertices[1]));																//Push the target into the q

	while (!q.empty())																		//While q is not empty
	{ 
		v = q.top();																		//pop the vertex with top priority 
		q.pop();

		u = (*v).parent;																	//Get the parent of v
		
		if (u == NULL ||  v == NULL)
		{
			continue;
		}
		p = LineIntersectionTest(u, v, &polygons);
		if (p == USHRT_MAX) {																//if no polygon has been intersected
			if (v == &(G.vertices[1]))														//if the target has been reached
			{

				ExtractPath(v, &Path, &G, qptr, polygons);									//Follow parents to start
				break;																		//Finished!
			}

			(*v).closed = true;																//Close the vertex v

			for (unsigned short i = 0; i < (*v).neighbors.size(); i++)                      //Expand neighbours
			{
				v_i = (*v).neighbors[i];													
				if (!(*v_i).closed)															//If v_i is not closed yet
				{
					if(!(*v_i).open || (*v).cost + VertexDistance(v, v_i) < (*v_i).cost)	//Complicated --> look up in documentation
					{
						(*v_i).parent = v;													//Set v as a parent of v_i
						(*v_i).cost = (*v).cost + VertexDistance(v, v_i);					//Path cost so far
						(*v_i).heruistic = VertexDistance(v_i, &(G.vertices[1]));						//Heuristic cost to target
						(*v_i).priority = (*v_i).cost + (*v_i).heruistic;					//Priority
						q.push(v_i);														//Push v_i into the q
						(*v_i).open = true;
					}
				}
			}
		}
		else {																				//Polyhon p has been intersected
			
			(*u).RemoveNeighbor(v);															//v is no longer neighbor of u
			(*v).RemoveNeighbor(u);

			(*v).parent = NULL;																//Remove parent of v
			(*v).cost = DBL_MAX;
			(*v).priority = DBL_MAX;
			FindParent(v, qptr);															//Look up the function
			if (!polygons[p].closed)														//if polygon p has not been closed
			{
				ConnectObstacle(&(polygons[p]), &G, qptr, &polygons);						//Look up the function
				polygons[p].closed = true;													//Close the intersected polygon
			}
		}
		(*v).open = false;
	}
	return Path;
}