# Minimal Construct Algorithm
This repository contains **C++** implementation of the Minimal Construct Algorithm as proposed by Missura, Lee and Bennewitz[1]. The paper can be found under /src/**source.pdf**. In short, it is used to find the shortest path from point A to point B in a plane covered by polygons.

All that is needed is to include the MCA.h header file and correctly use the FindDirectionOfTravel function, which returns the shortest path as a vector of Points.

The simplest implementation is contained in the testingMCA.cpp file. Sample polygon map is loaded from the /polygons/**TestPolygons.txt**, which follows a simple structure which goes like this:

MAIN<br />
x1<br />
y1<br />
x2<br />
y2<br />
...<br />
HOLE<br />
x1<br />
y1<br />
...<br />
HOLE<br />
x1<br />
y1<br />
...<br />

**MAIN** are polygons which encircle the start and end points. The final path has to be cointained within them, it makes sense to have just one such polygon, but there can be more. These polygons have to be written down in a clockwise manner.

**HOLE** are polygons which act as obstacles and cannot be crossed, such polygons have to be written down in a counter-clockwise manner.

Even if such a file is not used to load the polygons, the clockwise / counter-clockwise conditions still holds.

When the python script /polygons/**display_map.py** is used, it draws a visualization which looks like this:

![text](https://github.com/MachTe/Minimal_Construct_Algorithm/blob/main/TestMapEmpty.PNG?raw=true)

# Compilation
In order to execute **testingMCA.cpp** normaly, it has to be compiled and run from the same folder where **testingMCA.cpp** is situated. Start point S and target point T are chosen withing the file itself.

Once the program has finished, the shortest path can be found appended at the end of the **TestPolygons.txt** file. If you run the python script on it, it is going to visualize the path:

![text](https://github.com/MachTe/Minimal_Construct_Algorithm/blob/main/TestMapPtah.PNG?raw=true)


# Bibliography
[1] Missura, Marcell & Lee, Daniel & Bennewitz, Maren. (2018). Minimal Construct: Efficient Shortest Path Finding for Mobile Robots in Polygonal Maps. 7918-7923. 10.1109/IROS.2018.8594124. 
