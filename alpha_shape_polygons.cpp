#include "alpha_shape_polygons.h"
#include "main_window.h"
#include <CGAL/Qt/resources.h>

struct SPoint
{
	bool multiple;
	float x;
	float y;
};

struct SEdge
{
	int serial_number;
	bool found_marker;
	SPoint start_point;
	SPoint end_point;
	int polygon_number;
};


template <class OutputIterator>
void alpha_edges(const Alpha_shape_2& A, OutputIterator out)
{
	for (Alpha_shape_edges_iterator it = A.alpha_shape_edges_begin(); it != A.alpha_shape_edges_end(); ++it)
	{
		*out++ = A.segment(*it);

	}
}


template <class OutputIterator>
bool file_input(OutputIterator out, std::string path)
{
	std::ifstream is(path, std::ios::in);
	if (is.fail())
	{
		std::cerr << "unable to open file for input" << std::endl;
		return false;
	}
	int n;
	is >> n;
	//std::cout << "Reading " << n << " points from file" << std::endl;
	CGAL::cpp11::copy_n(std::istream_iterator<Point>(is), n, out);
	return true;
}
// this is a new branch.

void outputCloudOnTXT_PtNumber(Polygon_2 polygon)
{
	std::ofstream outdata;
	std::string name_file = "C:\\Surface_approximation\\temp\\Polygon_list.txt";

	outdata.open(name_file, std::ios::out);

	outdata << polygon << std::endl;
	outdata << std::endl << std::endl;
}

void outputSerialNumber(std::vector<int> output_serial_number)
{
	std::ofstream outdata;
	std::string name_file = "C:\\Surface_approximation\\temp\\output_serial_number.xls";

	outdata.open(name_file, std::ios::out);
	outdata.clear();

	for (int i = 0; i < output_serial_number.size(); i++)
	{
		outdata << output_serial_number[i] << std::endl;
	}

}

void outputEdgeOnTXT(std::vector<Alpha_shape_2::Point> alpha_shape_edges)
{
	std::ofstream outdata;
	std::string name_file = "C:\\Surface_approximation\\temp\\alpha_shape_edges.txt";

	outdata.open(name_file, std::ios::out);
	outdata.clear();

	for (int i = 0; i < alpha_shape_edges.size() - 2; i = i + 2)
	{
		outdata << alpha_shape_edges[i] << "   " << alpha_shape_edges[i + 1] << std::endl;
	}
	outdata << alpha_shape_edges[alpha_shape_edges.size() - 2] << "   " << alpha_shape_edges[alpha_shape_edges.size() - 1];
}

void outputEdgeList(std::vector<SEdge> edge_list)
{
	std::ofstream outdata;
	std::string name_file = "C:\\Surface_approximation\\temp\\alpha_shape_edge.xls";

	outdata.open(name_file, std::ios::out);
	outdata.clear();
	outdata << "serial_number" << "\t" << "found_marker" << "\t" << "multiple" << "\t" << "start_point.x" << "\t" << "start_point.y" << "\t" << "end_point.x" << "\t" << "end_point.y" << "\t" << std::endl;
	for (int i = 0; i < edge_list.size(); i++)
	{
		outdata << edge_list[i].serial_number << "\t" << edge_list[i].found_marker << "\t" << edge_list[i].start_point.multiple << "\t" <<
			edge_list[i].start_point.x << "\t" << edge_list[i].start_point.y << "\t" << edge_list[i].end_point.x << "\t" << edge_list[i].end_point.y << std::endl;
	}

}

void outputPolylineEdgeList(std::vector<SEdge> edge_list)
{
	std::ofstream outdata;
	std::string name_file = "C:\\Surface_approximation\\temp\\PolylineEdgeList.xls";

	outdata.open(name_file, std::ios::out);
	outdata.clear();
	outdata << "serial_number" << "\t" << "found_marker" << "\t" << "multiple" << "\t" << "start_point.x" << "\t" << "start_point.y" << "\t" << "end_point.x" << "\t" << "end_point.y" << "\t" << "polygon_number" << "\t" << std::endl;
	for (int i = 0; i < edge_list.size(); i++)
	{
		outdata << edge_list[i].serial_number << "\t" << edge_list[i].found_marker << "\t"
			<< edge_list[i].start_point.multiple << "\t" << edge_list[i].start_point.x << "\t" << edge_list[i].start_point.y << "\t"
			<< edge_list[i].end_point.x << "\t" << edge_list[i].end_point.y << "\t" << edge_list[i].polygon_number << std::endl;
	}

}

void visualizePolygonList(Polygon_list polygon_list)
{
	int argc = 1;
	char **argv;
	argv = (char **)malloc(sizeof(char**));
	*argv = "C:\\Surface_approximation\\build\\Debug\\Surface_approximation.exe";
	QApplication app(argc, argv);
	/*QPen marker_red(Qt::red, 2, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
	QPen marker_green(Qt::green, 3, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
	QPen marker_blue(Qt::blue, 0, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);*/

	QGraphicsScene scene;
	//scene.setSceneRect(0, 0, 2, 2);
	Polygon_2 polygon;
	float start_point_x, start_point_y, end_point_x, end_point_y;
	int polygon_list_size = polygon_list.size();
	int j;
	for (int i = 0; i < polygon_list_size; i++)
	{
		polygon = polygon_list.back();
		polygon_list.pop_back();
		for (j = 0; j < polygon.size() - 1; j++)
		{
			start_point_x = polygon[j].x();
			start_point_y = polygon[j].y();
			end_point_x = polygon[j + 1].x();
			end_point_y = polygon[j + 1].y();
			scene.addLine(QLineF(start_point_x, start_point_y, end_point_x, end_point_y));
		}
		/*if (polygon.size() > 1)
		{
		start_point_x = polygon[j].x();
		start_point_y = polygon[j].y();
		end_point_x = polygon[0].x();
		end_point_y = polygon[0].y();
		scene.addLine(QLineF(start_point_x, start_point_y, end_point_x, end_point_y));
		}*/
	}


	QGraphicsView* view = new QGraphicsView(&scene);
	CGAL::Qt::GraphicsViewNavigation navigation;
	view->installEventFilter(&navigation);
	view->viewport()->installEventFilter(&navigation);
	view->setRenderHint(QPainter::Antialiasing);

	view->show();
	app.exec();
}

int visualizePolygonList(Polygon_list *polygon_list, Polygon_2 polygon_add)
{
	Polygon_list polygon_list_copy = *polygon_list;
	int argc = 1;
	char **argv;
	argv = (char **)malloc(sizeof(char**));
	*argv = "C:\\Surface_approximation\\build\\Debug\\Surface_approximation.exe";
	QApplication app(argc, argv);
	QPen marker_red(Qt::red, 2, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
	QPen marker_green(Qt::green, 3, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
	QPen marker_blue(Qt::blue, 10, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
	QPen marker_yellow(Qt::yellow, 10, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);

	QGraphicsScene scene;
	//scene.setSceneRect(0, 0, 2, 2);
	Polygon_2 polygon;
	float start_point_x, start_point_y, end_point_x, end_point_y;
	int polygon_list_size = polygon_list_copy.size();
	int j;
	for (int i = 0; i < polygon_list_size; i++)
	{
		polygon = polygon_list_copy.back();
		polygon_list_copy.pop_back();
		for (j = 0; j < polygon.size() - 1; j++)
		{
			start_point_x = polygon[j].x();
			start_point_y = polygon[j].y();
			end_point_x = polygon[j + 1].x();
			end_point_y = polygon[j + 1].y();
			scene.addLine(QLineF(start_point_x, start_point_y, end_point_x, end_point_y), marker_green);
		}
		/*if (polygon.size() > 1)
		{
		start_point_x = polygon[j].x();
		start_point_y = polygon[j].y();
		end_point_x = polygon[0].x();
		end_point_y = polygon[0].y();
		scene.addLine(QLineF(start_point_x, start_point_y, end_point_x, end_point_y));
		}*/
	}

	for (int i = 0; i < polygon_add.size() - 1; i++)
	{
		start_point_x = polygon_add[i].x();
		start_point_y = polygon_add[i].y();
		end_point_x = polygon_add[i + 1].x();
		end_point_y = polygon_add[i + 1].y();
		if (i == 0)
		{
			scene.addLine(QLineF(start_point_x, start_point_y, end_point_x, end_point_y), marker_blue);
		}
		else
		{
			if (i == 1)
			{
				scene.addLine(QLineF(start_point_x, start_point_y, end_point_x, end_point_y), marker_yellow);
			}
			else
			{
				scene.addLine(QLineF(start_point_x, start_point_y, end_point_x, end_point_y), marker_red);
			}
		}

	}


	QGraphicsView* view = new QGraphicsView(&scene);
	CGAL::Qt::GraphicsViewNavigation navigation;
	view->installEventFilter(&navigation);
	view->viewport()->installEventFilter(&navigation);
	view->setRenderHint(QPainter::Antialiasing);

	view->show();
	return app.exec();
}


void sortrows(std::vector<std::vector<float>>& edge_points_matrix, int col)
{
	std::sort(edge_points_matrix.begin(), edge_points_matrix.end(),
		[col](const std::vector<float>& lhs, const std::vector<float>& rhs)
	{
		if (lhs[col] == rhs[col])
		{
			return lhs[col + 1]<rhs[col + 1];
		}
		return lhs[col] < rhs[col];
	});
}

std::vector<SEdge> loadEdgeFromTXT(std::string path, int num_edges)
{
	std::vector<std::vector<float>> edge_points_matrix;
	std::ifstream in(path, std::ifstream::in);
	edge_points_matrix.resize(num_edges);
	for (int i = 0; i < num_edges; ++i)
	{
		edge_points_matrix[i].resize(4);
		in >> edge_points_matrix[i][0];
		in >> edge_points_matrix[i][1];
		in >> edge_points_matrix[i][2];
		in >> edge_points_matrix[i][3];
	}
	in.close();
	sortrows(edge_points_matrix, 0);

	std::vector<SEdge> edge_list;
	SEdge edge_temp, edge_temp1;

	edge_temp.serial_number = 0;
	edge_temp.found_marker = false;
	edge_temp.start_point.x = edge_points_matrix[0][0];
	edge_temp.start_point.y = edge_points_matrix[0][1];
	edge_temp.end_point.x = edge_points_matrix[0][2];
	edge_temp.end_point.y = edge_points_matrix[0][3];
	edge_temp.start_point.multiple = false;
	edge_list.push_back(edge_temp);

	for (int i = 1; i < num_edges; i++)
	{
		edge_temp1 = edge_list.back();
		edge_list.pop_back();
		edge_temp.serial_number = i;
		edge_temp.found_marker = false;
		edge_temp.start_point.x = edge_points_matrix[i][0];
		edge_temp.start_point.y = edge_points_matrix[i][1];
		edge_temp.end_point.x = edge_points_matrix[i][2];
		edge_temp.end_point.y = edge_points_matrix[i][3];

		if ((edge_temp.start_point.x == edge_temp1.start_point.x) && (edge_temp.start_point.y == edge_temp1.start_point.y))
		{
			edge_temp1.start_point.multiple = true;
			edge_temp.start_point.multiple = true;

			edge_list.push_back(edge_temp1);
			edge_list.push_back(edge_temp);
		}
		else
		{
			edge_temp.start_point.multiple = false;
			edge_list.push_back(edge_temp1);
			edge_list.push_back(edge_temp);
		}
	}
	//outputEdgeList(edge_list);
	return edge_list;
}

std::vector<SEdge> loadEdgeFromTXT(std::vector<Alpha_shape_2::Point> alpha_shape_edges)
{
	std::vector<std::vector<float>> edge_points_matrix;
	int num_edges = alpha_shape_edges.size() / 2;
	edge_points_matrix.resize(num_edges);
	for (int i = 0; i < num_edges; ++i)
	{
		edge_points_matrix[i].resize(4);
		edge_points_matrix[i][0] = alpha_shape_edges[i * 2].x();
		edge_points_matrix[i][1] = alpha_shape_edges[i * 2].y();
		edge_points_matrix[i][2] = alpha_shape_edges[i * 2 + 1].x();
		edge_points_matrix[i][3] = alpha_shape_edges[i * 2 + 1].y();
	}
	sortrows(edge_points_matrix, 0);

	std::vector<SEdge> edge_list;
	SEdge edge_temp, edge_temp1;

	edge_temp.serial_number = 0;
	edge_temp.found_marker = false;
	edge_temp.start_point.x = edge_points_matrix[0][0];
	edge_temp.start_point.y = edge_points_matrix[0][1];
	edge_temp.end_point.x = edge_points_matrix[0][2];
	edge_temp.end_point.y = edge_points_matrix[0][3];
	edge_temp.start_point.multiple = false;
	edge_list.push_back(edge_temp);

	for (int i = 1; i < num_edges; i++)
	{
		edge_temp1 = edge_list.back();
		edge_list.pop_back();
		edge_temp.serial_number = i;
		edge_temp.found_marker = false;
		edge_temp.start_point.x = edge_points_matrix[i][0];
		edge_temp.start_point.y = edge_points_matrix[i][1];
		edge_temp.end_point.x = edge_points_matrix[i][2];
		edge_temp.end_point.y = edge_points_matrix[i][3];

		if ((edge_temp.start_point.x == edge_temp1.start_point.x) && (edge_temp.start_point.y == edge_temp1.start_point.y))
		{
			edge_temp1.start_point.multiple = true;
			edge_temp.start_point.multiple = true;

			edge_list.push_back(edge_temp1);
			edge_list.push_back(edge_temp);
		}
		else
		{
			edge_temp.start_point.multiple = false;
			edge_list.push_back(edge_temp1);
			edge_list.push_back(edge_temp);
		}
	}
	//outputEdgeList(edge_list);
	return edge_list;
}

SEdge findFirstEdge(std::vector<SEdge> *edge_list)
{
	int i;
	for (i = 0; i < (*edge_list).size(); i++)
	{
		if (((*edge_list)[i].start_point.multiple) && (!(*edge_list)[i].found_marker))
		{
			(*edge_list)[i].found_marker = true;
			return (*edge_list)[i];
		}
	}
	for (i = 0; i < (*edge_list).size(); i++)
	{
		if (!(*edge_list)[i].found_marker)
		{
			(*edge_list)[i].found_marker = true;
			return (*edge_list)[i];
		}
	}

}

bool findNextEdge(std::vector<SEdge> *edge_list, SEdge current_edge, SEdge *next_edge)
{

	bool found = false;
	int low = 0, mid;
	int high = (*edge_list).size() - 1;
	while (low <= high)
	{
		mid = (high + low) / 2;
		if (abs((*edge_list)[mid].start_point.x - current_edge.end_point.x) < 0.0001)
		{
			int mid1 = mid - 5;
			if (mid1 < 0)
			{
				mid1 = 0;
			}
			int mid_range = 10;
			if ((mid + 5) >(*edge_list).size() - 1)
			{
				mid_range = (*edge_list).size() - 1 - mid;
			}
			for (int i = 0; i < mid_range; i++)
			{
				if ((abs((*edge_list)[mid1 + i].start_point.x - current_edge.end_point.x) < 0.0001) && (abs((*edge_list)[mid1 + i].start_point.y - current_edge.end_point.y) < 0.0001))
				{
					if (((*edge_list)[mid1 + i].start_point.multiple) || ((*edge_list)[mid1 + i].found_marker))
					{
						continue;
					}
					else
					{
						found = true;
						(*next_edge) = (*edge_list)[mid1 + i];
						(*edge_list)[mid1 + i].found_marker = true;
						break;
					}

				}
			}
			break;
		}
		else
		{
			if ((*edge_list)[mid].start_point.x <= current_edge.end_point.x)
				low = mid + 1;
			else
				high = mid - 1;
		}


	}

	return found;
}

std::vector<SEdge> buildPolylineEdgeList(Polygon_list polyline_list)
{
	int polyline_list_size = polyline_list.size(), polyline_size;
	std::vector<std::vector<float>> polyline_edge_points_matrix;
	polyline_edge_points_matrix.resize(polyline_list_size);
	Polygon_2 polyline;
	for (int i = 0; i < polyline_list_size; ++i)
	{
		polyline = polyline_list.back();
		polyline_list.pop_back();
		polyline_size = polyline.size();
		polyline_edge_points_matrix[i].resize(5);
		polyline_edge_points_matrix[i][0] = polyline[0].x();
		polyline_edge_points_matrix[i][1] = polyline[0].y();
		polyline_edge_points_matrix[i][2] = polyline[polyline_size - 1].x();
		polyline_edge_points_matrix[i][3] = polyline[polyline_size - 1].y();
		polyline_edge_points_matrix[i][4] = polyline_list_size - i - 1;
	}
	sortrows(polyline_edge_points_matrix, 0);

	std::vector<SEdge> polyline_edge_list;
	SEdge edge_temp, edge_temp1;

	edge_temp.serial_number = 0;
	edge_temp.found_marker = false;
	edge_temp.start_point.x = polyline_edge_points_matrix[0][0];
	edge_temp.start_point.y = polyline_edge_points_matrix[0][1];
	edge_temp.end_point.x = polyline_edge_points_matrix[0][2];
	edge_temp.end_point.y = polyline_edge_points_matrix[0][3];
	edge_temp.polygon_number = polyline_edge_points_matrix[0][4];
	edge_temp.start_point.multiple = false;
	polyline_edge_list.push_back(edge_temp);

	for (int i = 1; i < polyline_list_size; i++)
	{
		edge_temp1 = polyline_edge_list.back();
		polyline_edge_list.pop_back();
		edge_temp.serial_number = i;
		edge_temp.found_marker = false;
		edge_temp.start_point.x = polyline_edge_points_matrix[i][0];
		edge_temp.start_point.y = polyline_edge_points_matrix[i][1];
		edge_temp.end_point.x = polyline_edge_points_matrix[i][2];
		edge_temp.end_point.y = polyline_edge_points_matrix[i][3];
		edge_temp.polygon_number = polyline_edge_points_matrix[i][4];
		if ((edge_temp.start_point.x == edge_temp1.start_point.x) && (edge_temp.start_point.y == edge_temp1.start_point.y))
		{
			edge_temp1.start_point.multiple = true;
			edge_temp.start_point.multiple = true;

			polyline_edge_list.push_back(edge_temp1);
			polyline_edge_list.push_back(edge_temp);
		}
		else
		{
			edge_temp.start_point.multiple = false;
			polyline_edge_list.push_back(edge_temp1);
			polyline_edge_list.push_back(edge_temp);
		}
	}
	outputPolylineEdgeList(polyline_edge_list);
	return polyline_edge_list;
}

bool findNextPolyline(std::vector<SEdge> *polyline_edge_list, SEdge current_edge, SEdge *next_edge)
{
	bool found = false;
	int low = 0, mid;
	int high = (*polyline_edge_list).size() - 1;
	while (low <= high)
	{
		mid = (high + low) / 2;
		if (abs((*polyline_edge_list)[mid].start_point.x - current_edge.end_point.x) < 0.0001)
		{
			if (abs((*polyline_edge_list)[mid].start_point.y - current_edge.end_point.y) < 0.0001)
			{
				if ((*polyline_edge_list)[mid].found_marker)
				{
					if ((*polyline_edge_list)[mid].start_point.x < current_edge.end_point.x)
						low = mid + 1;
					else
						high = mid - 1;
				}
				else
				{
					found = true;
					(*next_edge) = (*polyline_edge_list)[mid];
					(*polyline_edge_list)[mid].found_marker = true;
					return found;
				}
			}
			else
			{
				if (abs((*polyline_edge_list)[mid + 1].start_point.y - current_edge.end_point.y) < 0.0001)
				{
					found = true;
					(*next_edge) = (*polyline_edge_list)[mid + 1];
					(*polyline_edge_list)[mid + 1].found_marker = true;
					return found;
				}
				else
				{
					if (abs((*polyline_edge_list)[mid - 1].start_point.y - current_edge.end_point.y) < 0.0001)
					{
						found = true;
						(*next_edge) = (*polyline_edge_list)[mid - 1];
						(*polyline_edge_list)[mid - 1].found_marker = true;
						return found;
					}
					else
					{
						break;
					}
				}
			}

		}
		else
		{
			if ((*polyline_edge_list)[mid].start_point.x < current_edge.end_point.x)
				low = mid + 1;
			else
				high = mid - 1;
		}
	}
	found = false;
	return found;
}

Polygon_2 updatePolygonList(std::vector<int> number_polygon, Polygon_list polyline_list)
{
	int number_polygon_size = number_polygon.size();
	Polygon_2 polygon;
	for (int i = 0; i < number_polygon_size; i++)
	{
		int j = number_polygon[i];
		Polygon_list polyline_list_copy = polyline_list;
		while (j > 0)
		{
			polyline_list_copy.pop_front();
			j--;
		}
		Polygon_2 polygon_add = polyline_list_copy.front();
		int polygon_add_size = polygon_add.size();
		if (i != (number_polygon_size - 1))
		{
			polygon_add_size--;
		}
		for (int k = 0; k < polygon_add_size; k++)
		{
			polygon.push_back(Point(polygon_add[k].x(), polygon_add[k].y()));
		}

	}
	return polygon;
	//(*polygon_list).push_back(polygon);
	/*Polygon_list new_polygon_list;
	new_polygon_list.push_back(polygon);
	visualizePolygonList(new_polygon_list);*/
}

void assemblePolylineList(Polygon_list *polygon_list, Polygon_list polyline_list)
{
	std::vector<SEdge> polyline_edge_list = buildPolylineEdgeList(polyline_list);
	int size = polyline_edge_list.size();
	float end_x, end_y;
	std::vector<int> number_polygon;
	do
	{
		std::vector<int> number_polygon;
		SEdge first_edge = findFirstEdge(&polyline_edge_list);
		number_polygon.push_back(first_edge.polygon_number);
		size--;
		SEdge current_edge = first_edge, next_edge;
		//output_serial_number.push_back(current_edge.serial_number);
		while (findNextPolyline(&polyline_edge_list, current_edge, &next_edge))
		{
			number_polygon.push_back(next_edge.polygon_number);
			size--;
			if ((next_edge.end_point.x == first_edge.start_point.x) && (next_edge.end_point.y == first_edge.start_point.y))
			{
				break;
			}
			current_edge = next_edge;
			//output_serial_number.push_back(current_edge.serial_number);
		}
		Polygon_2 polygon = updatePolygonList(number_polygon, polyline_list);
		(*polygon_list).push_back(polygon);
		//std::cout << "area:" << polygon.area() << std::endl;
		//visualizePolygonList(polygon_list, polygon);

	} while (size > 0);


}

Polygon_list polygonPartition(std::vector<SEdge> edge_list)
{
	Polygon_list polygon_list, polyline_list, polygon_and_polyline_list;
	//std::vector<Polygon_2> polyline_list;
	int size = edge_list.size();
	float end_x, end_y;
	//std::vector<int> output_serial_number;
	do
	{
		SEdge first_edge = findFirstEdge(&edge_list);
		Polygon_2 polygon;
		polygon.push_back(Point(first_edge.start_point.x, first_edge.start_point.y));
		polygon.push_back(Point(first_edge.end_point.x, first_edge.end_point.y));
		end_x = first_edge.end_point.x;
		end_y = first_edge.end_point.y;
		size--;
		SEdge current_edge = first_edge, next_edge;
		//output_serial_number.push_back(current_edge.serial_number);
		while (findNextEdge(&edge_list, current_edge, &next_edge))
		{
			polygon.push_back(Point(next_edge.end_point.x, next_edge.end_point.y));
			end_x = next_edge.end_point.x;
			end_y = next_edge.end_point.y;
			size--;
			current_edge = next_edge;
			//output_serial_number.push_back(current_edge.serial_number);
		}
		//if (polygon.size() < 3)
		//{
		//	//size++;
		//	continue;
		//}
		//outputCloudOnTXT_PtNumber(polygon);

		//if (polygon.is_simple())
		//{

		if ((abs(first_edge.start_point.x - end_x) < 0.0001) && (abs(first_edge.start_point.y - end_y) < 0.0001))
		{
			polygon_list.push_back(polygon);
			//std::cout << "area:" << polygon.area() << std::endl;
		}
		else
		{
			polyline_list.push_back(polygon);
		}
		polygon_and_polyline_list.push_back(polygon);
		//outputSerialNumber(output_serial_number);
	} while (size > 0);

	/*int polyline_size = polyline_list.size();
	for (int i = 0; i < polyline_size; i++)
	{

	}*/
	//visualizePolygonList(polygon_list);
	//visualizePolygonList(polyline_list);
	//visualizePolygonList(polygon_and_polyline_list);
	if (polyline_list.size() != 0)
	{
		assemblePolylineList(&polygon_list, polyline_list);
	}



	return polygon_list;
}

double computePolygonArea(Polygon_list polygon_list)
{
	int polygon_list_size = polygon_list.size();
	double total_area = 0.0;
	//std::cout << polygon_list_size << std::endl;
	float area_threshold = POISSON_DISK_SAMPLING_RADIUS * POISSON_DISK_SAMPLING_RADIUS * 2;
	for (int i = 0; i < polygon_list_size; i++)
	{
		Polygon_2 current_polygon = polygon_list.back();
		polygon_list.pop_back();
		//std::cout << "area[" << i << "] = " << current_polygon.area() << std::endl;
		if (abs(current_polygon.area()) > area_threshold)
		{
			total_area = total_area + current_polygon.area();
		}
	}

	return total_area;

}






// Reads a list of points and returns a list of segments
// corresponding to the Alpha shape.
double getAlphaShape(std::string path)
{
	std::list<Point> points;
	if (!file_input(std::back_inserter(points), path))
	{

		//return -1;
	}

	Alpha_shape_2 A(points.begin(), points.end(), FT(100000), Alpha_shape_2::REGULARIZED);
	
	displayAlphaShape(1000.0, path);


	A.set_alpha(1000.0);
	std::vector<Alpha_shape_2::Point> alpha_shape_edges;

	for (Alpha_shape_edges_iterator it = A.alpha_shape_edges_begin(); it != A.alpha_shape_edges_end(); ++it)
	{
		alpha_shape_edges.push_back(A.segment(*it).vertex(0));
		alpha_shape_edges.push_back(A.segment(*it).vertex(1));
	}
	//Future modificaiton: output directly from polygon_list
	//outputEdgeOnTXT(alpha_shape_edges);
	//std::vector<SEdge> edge_list = loadEdgeFromTXT("C:\\Alpha_shapes_2\\alpha_shape_edges.txt", alpha_shape_edges.size() / 2);
	std::vector<SEdge> edge_list = loadEdgeFromTXT(alpha_shape_edges);
	outputEdgeList(edge_list);
	Polygon_list polygon_list = polygonPartition(edge_list);
	//visualizePolygonList(polygon_list);
	double total_area = computePolygonArea(polygon_list);
	return total_area;
}

void displayAlphaShape(double optimal_alpha, std::string path)
{
	int argc = 1;
	char **argv;
	argv = (char **)malloc(sizeof(char**));
	*argv = "C:\\Surface_approximation\\build\\Debug\\Surface_approximation.exe";

		QApplication app(argc, argv);

		app.setOrganizationDomain("geometryfactory.com");
		app.setOrganizationName("GeometryFactory");
		app.setApplicationName("Alpha_shape_2 demo");

		// Import resources from libCGAL (Qt5).
		CGAL_QT_INIT_RESOURCES;


		MainWindow mainWindow;
		mainWindow.alpha = optimal_alpha;
		QString qstr = QString::fromStdString(path);
		mainWindow.open(qstr);

		mainWindow.as.set_alpha(optimal_alpha);
		//mainWindow.on_actionRecenter_triggered();
		app.exec();
		mainWindow.show();
		app.exec();
		//app.exit();
		//app.closeAllWindows();
		//optimal_alpha += 50;
}