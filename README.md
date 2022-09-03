# Minimal Construct Algorithm
This repository contains C++ implementation of the Minimal Construct Algorithm as proposed by Missura, Lee and Bennewitz[1]. The paper can be found under /src/source.pdf. In short, it is used to find the shortest path from point A to point B in a plane covered by polygons.

All that is needed is to include the MCA.h header file and correctly use the FindDirectionOfTravel function, which returns the shortest path as a vector of Points.

The simplest implementation is contained in the testingMCA.cpp file. Sample polygon map is loaded from the /polygons/TestPolygons.txt, which follows a simple structure which goes like this:

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

# Bibliography
[1] Missura, Marcell & Lee, Daniel & Bennewitz, Maren. (2018). Minimal Construct: Efficient Shortest Path Finding for Mobile Robots in Polygonal Maps. 7918-7923. 10.1109/IROS.2018.8594124. 
