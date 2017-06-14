#pragma once
#ifndef MAIN_WINDOW_H
#define MAIN_WINDOW_H


#include "alpha_shape_polygons.h"

class QWidget;

class MainWindow : public CGAL::Qt::DemosMainWindow, public Ui::Alpha_shapes_2
{
	Q_OBJECT

private:


	std::vector<Point_2> points;

	QGraphicsScene scene;

	CGAL::Qt::AlphaShapeGraphicsItem<Alpha_shape_2> * agi;
	CGAL::Qt::GraphicsViewPolylineInput<K> * pi;

public:
	Alpha_shape_2 as;

	double alpha;

	QString fileName;

	MainWindow();

	public Q_SLOTS:

	void processInput(CGAL::Object o);

	void alphaChanged(int i);

	void on_actionInsertRandomPoints_triggered();

	void on_actionLoadPoints_triggered();

	void on_actionClear_triggered();

	void on_actionRecenter_triggered();

	void open(std::list<Point> points);

	void open(QString fileName);

Q_SIGNALS:

	void changed();

};

#endif