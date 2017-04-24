#include "main_window.h"


MainWindow::MainWindow() : DemosMainWindow()
{
	setupUi(this);

	//setAttribute(Qt::WA_DeleteOnClose);

	this->graphicsView->setAcceptDrops(false);

	// Add a GraphicItem for the alpha shape
	agi = new CGAL::Qt::AlphaShapeGraphicsItem<Alpha_shape_2>(&as);

	QObject::connect(this, SIGNAL(changed()), agi, SLOT(modelChanged()));

	agi->setVerticesPen(QPen(Qt::red, 3, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));

	agi->setEdgesPen(QPen(Qt::lightGray, 0, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));

	agi->setRegularEdgesPen(QPen(Qt::blue, 0, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));

	agi->setSingularEdgesPen(QPen(Qt::cyan, 0, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));

	agi->setRegularFacesBrush(QBrush(Qt::cyan));

	scene.addItem(agi);


	// Manual handling of actions

	QObject::connect(this->alphaSlider, SIGNAL(valueChanged(int)), this, SLOT(alphaChanged(int)));

	QObject::connect(this->alphaBox, SIGNAL(valueChanged(int)), this, SLOT(alphaChanged(int)));

	QObject::connect(this->alphaSlider, SIGNAL(valueChanged(int)), this->alphaBox, SLOT(setValue(int)));

	QObject::connect(this->alphaBox, SIGNAL(valueChanged(int)), this->alphaSlider, SLOT(setValue(int)));

	QObject::connect(this->actionQuit, SIGNAL(triggered()), this, SLOT(close()));

	pi = new CGAL::Qt::GraphicsViewPolylineInput<K>(this, &scene, 1, false); // inputs a list with one point
	QObject::connect(pi, SIGNAL(generate(CGAL::Object)), this, SLOT(processInput(CGAL::Object)));

	scene.installEventFilter(pi);

	//this->actionShowAlphaShape->setChecked(true);

	//

	// Setup the scene and the view
	//
	scene.setItemIndexMethod(QGraphicsScene::NoIndex);
	//scene.setSceneRect(-500, -1000, 500, 1000);
	this->graphicsView->setScene(&scene);
	this->graphicsView->setMouseTracking(true);

	// Turn the vertical axis upside down
	this->graphicsView->matrix().scale(-1, 1);

	// The navigation adds zooming and translation functionality to the
	// QGraphicsView
	this->addNavigation(this->graphicsView);

	this->setupStatusBar();
	this->setupOptionsMenu();
	this->addAboutDemo(":/cgal/help/about_Alpha_shapes_2.html");
	this->addAboutCGAL();

	this->addRecentFiles(this->menuFile, this->actionQuit);
	connect(this, SIGNAL(openRecentFile(QString)), this, SLOT(open(QString)));
}

void MainWindow::processInput(CGAL::Object o)
{
	std::list<Point_2> input;
	if (CGAL::assign(input, o))
	{
		if (input.size() == 1)
		{
			points.push_back(input.front());
			as.make_alpha_shape(points.begin(), points.end());
			alpha = MainWindow::alpha;
			//std::cerr << endl << "alpha = " << alpha << endl;
			as.set_alpha(alpha);
		}
		Q_EMIT(changed());
	}
}

void MainWindow::alphaChanged(int i)
{
	if (as.number_of_alphas() > 0)
	{
		if (i < 100)
		{
			int n = static_cast<int>((i * as.number_of_alphas()) / 100);
			if (n == 0) n++;
			alpha = as.get_nth_alpha(n);
			std::cerr << endl << "alpha = " << alpha << endl;
			alpha = MainWindow::alpha;
			std::cerr << endl << "alpha = " << alpha << endl;
			as.set_alpha(alpha);

		}
		else
		{
			Alpha_iterator alpha_end_it = as.alpha_end();
			alpha = (*(--alpha_end_it)) + 1;
			alpha = MainWindow::alpha;
			//std::cerr << endl << "alpha = " << alpha << endl;
			as.set_alpha(alpha);
		}
	}
	else
	{
		alpha = 0;
		alpha = MainWindow::alpha;
		//std::cerr << endl << "alpha = " << alpha << endl;
		as.set_alpha(alpha);
	}
	//std::cerr << endl << "alpha = " << alpha << endl;
	Q_EMIT(changed());
}

/*
*  Qt Automatic Connections
*  http://doc.qt.io/qt-5/designer-using-a-ui-file.html#automatic-connections
*
*  setupUi(this) generates connections to the slots named
*  "on_<action_name>_<signal_name>"
*/

void MainWindow::on_actionClear_triggered()
{
	as.clear();
	points.clear();
	Q_EMIT(changed());
}

void MainWindow::on_actionInsertRandomPoints_triggered()
{
	QRectF rect = CGAL::Qt::viewportsBbox(&scene);
	CGAL::Qt::Converter<K> convert;
	Iso_rectangle_2 isor = convert(rect);
	CGAL::Random_points_in_iso_rectangle_2<Point_2> pg((isor.min)(), (isor.max)());
	bool ok = false;

	const int number_of_points = QInputDialog::getInt(this, tr("Number of random points"), tr("Enter number of random points"),
		100, 0, (std::numeric_limits<int>::max)(), 1, &ok);

	if (!ok)
	{
		return;
	}

	// wait cursor
	QApplication::setOverrideCursor(Qt::WaitCursor);

	points.reserve(points.size() + number_of_points);
	for (int i = 0; i < number_of_points; ++i)
	{
		points.push_back(*pg++);
	}
	as.make_alpha_shape(points.begin(), points.end());
	as.set_alpha(alpha);
	// default cursor
	QApplication::restoreOverrideCursor();
	Q_EMIT(changed());
}

void MainWindow::on_actionLoadPoints_triggered()
{
	QString fileName = QFileDialog::getOpenFileName(this, tr("Open Points file"), ".");
	if (!fileName.isEmpty())
	{
		open(fileName);
	}
}

void MainWindow::open(QString fileName)
{
	//std::cerr << "open " << std::endl;
	//std::cerr << qPrintable(fileName) << std::endl;
	// wait cursor
	QApplication::setOverrideCursor(Qt::WaitCursor);
	std::ifstream ifs(qPrintable(fileName));
	int n;
	ifs >> n;

	K::Point_2 p;
	while (ifs >> p)
	{
		points.push_back(p);
	}
	as.make_alpha_shape(points.begin(), points.end());
	as.set_alpha(alpha);

	// default cursor
	QApplication::restoreOverrideCursor();
	this->addToRecentFiles(fileName);
	actionRecenter->trigger();
	Q_EMIT(changed());
}

void MainWindow::on_actionRecenter_triggered()
{
	this->graphicsView->setSceneRect(agi->boundingRect());
	this->graphicsView->fitInView(agi->boundingRect(), Qt::KeepAspectRatio);
}
