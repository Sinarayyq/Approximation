#pragma once
#ifndef ALPHA_SHAPE_POLYGONS
#define ALPHA_SHAPE_POLYGONS

// CGAL headers
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Alpha_shape_2.h>
#include <CGAL/point_generators_2.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/algorithm.h>

// Qt headers
#include <QtGui>
#include <QString>
#include <QActionGroup>
#include <QFileDialog>
#include <QInputDialog>
#include <QLineF>
#include <QRectF>
#include <QApplication> 
#include <QGraphicsScene>
#include <QGraphicsView> 

// GraphicsView items and event filters (input classes)
#include <CGAL/Qt/AlphaShapeGraphicsItem.h>
#include <CGAL/Qt/GraphicsViewPolylineInput.h>
#include <CGAL/Qt/GraphicsViewNavigation.h>

// for viewportsBbox
#include <CGAL/Qt/utility.h>

// the two base classes
#include "ui_Alpha_shapes_2.h"
#include <CGAL/Qt/DemosMainWindow.h>


#include <iostream>
#include <fstream>
#include <vector>
#include <list>
#include <algorithm>
#include <boost/tuple/tuple.hpp>
#include "border_definition.h"
#include "read_parameters.h"

////////////////////////////////////////////////////////////////////////////////////////////////////
// There is no typedef.h currently in our project, therefore the definitions are placed in header files at the moment. (Apr. 11th)
typedef CGAL::Exact_predicates_inexact_constructions_kernel K;

//a template parameter for Cartesian kernels
typedef K::FT FT;

typedef K::Point_2  Point;
typedef K::Point_2  Point_2;
typedef K::Iso_rectangle_2 Iso_rectangle_2;
typedef K::Segment_2  Segment;
typedef CGAL::Alpha_shape_vertex_base_2<K> Vb;
typedef CGAL::Alpha_shape_face_base_2<K>  Fb;
typedef CGAL::Triangulation_data_structure_2<Vb, Fb> Tds;
typedef CGAL::Delaunay_triangulation_2<K, Tds> Triangulation_2;
typedef CGAL::Alpha_shape_2<Triangulation_2>  Alpha_shape_2;
typedef Alpha_shape_2::Alpha_iterator Alpha_iterator;

//A bidirectional and non-mutable iterator that allow to traverse the edges 
//which belongs to the £\-shape for the current £\.
typedef Alpha_shape_2::Alpha_shape_edges_iterator Alpha_shape_edges_iterator;
typedef Alpha_shape_2::Alpha_shape_vertices_iterator Alpha_shape_vertices_iterator;




typedef CGAL::Polygon_2<K> Polygon_2;
typedef Polygon_2::Vertex_iterator VertexIterator;
typedef Polygon_2::Edge_const_iterator EdgeIterator;
typedef std::list<Polygon_2> Polygon_list;

//class declaration and definition


//member function declaration
double getAlphaShape(std::string path);

void displayAlphaShape(double optimal_alpha, std::string path);


#endif