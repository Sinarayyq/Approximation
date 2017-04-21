#include "../build/check_patch_validity.h"

//using namespace boost;

//typedef adjacency_list < vecS, vecS, undirectedS, no_property, property < edge_weight_t, double >> Graph;
//typedef boost::graph_traits<Graph>::vertex_descriptor vertex_descriptor;


std::vector<uint32_t, std::allocator<char32_t>> mstPrim (Graph g)
{
	std::vector < graph_traits < Graph >::vertex_descriptor > p(num_vertices(g));
		prim_minimum_spanning_tree(g, &p[0]);
	
		for (std::size_t i = 0; i != p.size(); ++i)
		/*if (p[i] != i)
			std::cerr << "parent[" << i << "] = " << p[i] << std::endl;
		else
			std::cerr << "parent[" << i << "] = no parent" << std::endl;*/

		return(p);
}

void myBFS (Graph mstGraph, int source, std::vector<int>& parents, std::vector<int>& distances)
{
	////Build the MST of the graph
	//std::vector<uint32_t, std::allocator<char32_t>> p = mstPrim (g);
	////Build the graph of the resulting MST
	//const int num_nodes = num_vertices(g);
	//std::cerr << "num_vertices" << num_nodes << std::endl;
	//GraphNoWeight mstGraph(num_nodes);

	//std::vector<E> edgeVec;
	////std::vector<float> weightVec;
	////Eigen::Vector3f a, b;
	////double current_weight;
	//for (std::size_t i = 0; i < p.size(); ++i)
	//{
	//	if (p[i] != i)
	//	{
	//		add_edge(i, p[i], mstGraph);
	//	}
	//}

	

	//typedef boost::graph_traits<Graph>::vertex_descriptor vertex_descriptor;
	//std::vector<int> distances (num_nodes1, INF);
	//std::vector<int> parents  (num_nodes1,-1);
	//distances.reserve(num_nodes1);
	//parents.reserve(num_nodes1);
	std::fill(distances.begin(), distances.end(), INF);
	std::fill(parents.begin(), parents.end(), -1);

	std::list<int> queue;
	int current = source;
	distances[current] = 0;
	parents[current] = current;
	queue.push_back(current);
	std::list<int>::iterator it = queue.begin();
		//std::next(queue.begin(), current);
	typedef boost::graph_traits<Graph>::adjacency_iterator adjacency_iterator;

	while (queue.size()>0)
	{
		current = *it;
		//std::cerr << std::endl;
		//std::cerr << "current index of node:  " << current << std::endl;
		//std::cerr << "number of nodes in the list:  " << queue.size() << std::endl;
		it = queue.erase (it);
		//std::cerr << "number of nodes in the list after erasing:  " << queue.size() << std::endl;

		/*for (boost::tie(vi, vi_end) = adjacent_vertices(current, g); vi != vi_end; ++vi)
		{
			std::cerr << "adjacent node:  " << queue.size() << std::endl;
		}*/
		
		//std::pair<adjacency_iterator, adjacency_iterator> p = adjacent_vertices(current, g);
		adjacency_iterator vi, vi_end;
		boost::tie(vi, vi_end) = adjacent_vertices(current, mstGraph);

		for (boost::tie(vi, vi_end) = adjacent_vertices(current, mstGraph); vi != vi_end; ++vi)
		{
			//std::cerr << "adjacent:  " << *vi << std::endl;
			if (parents[*vi]==-1)
			{
				//std::cerr << "parents[*vi]=  " << parents[*vi] << ". Nodo NUOVO." << std::endl;
				distances[*vi]=distances[current]+1;
				//std::cerr << "distances[*vi]=  " << distances[*vi] << std::endl;
				parents[*vi]=current;
				queue.push_back(*vi);
			}
			/*else
			{
				std::cerr << "parents[*vi]=  " << parents[*vi] << ". Il nodo era gia' stato visitato"
					<< std::endl;
				std::cerr << "distances[*vi]=  " << distances[*vi] << std::endl;
			}*/
		}

		/*std::cerr << "In queue:  ";
		for (std::size_t i = 0; i < queue.size(); ++i)
		{
			std::list<int>::iterator it_list = std::next(queue.begin(), i);
			std::cerr << *it_list << ", ";
		}
		std::cerr << std::endl; */

		if(queue.size()>0)
		{
			it = queue.begin();
			//std::cerr << "iteratore adesso punta a: " << *it << std::endl;
		}
	}
	/*for (std::size_t i = 0; i < parents.size(); ++i)
	{
		std::cerr << "parents vector: " << parents[i] << std::endl;
	}
	for (std::size_t i = 0; i < distances.size(); ++i)
	{
		std::cerr << "distances vector: " << distances[i] << std::endl;
	}*/
}

int furthestNode (int source, Graph mstGraph, std::list<int>& path)
{
	const int num_nodes = num_vertices(mstGraph);
	std::vector<int> distances(num_nodes);
	std::vector<int> parents(num_nodes);
	myBFS(mstGraph, source, parents, distances);

	auto max_value = std::max_element(distances.begin(), distances.end());
	int position = std::distance(distances.begin(),max_value);
	//std::cerr << "max value: " << *max_value << std::endl;
	//std::cerr << "max position: " << position << std::endl;
	
	int i=position;
	path.push_back(i);
	while (parents[i]!=i)
	{
		path.push_back(parents[i]);
		i=parents[i];
	}

	/*std::cerr << "max path: ";
		for (std::size_t i = 0; i < path.size(); ++i)
		{
			std::list<int>::iterator it_list = std::next(path.begin(), i);
			std::cerr << *it_list << ", ";
		}
		std::cerr << std::endl;*/
	return position;
}

void findLongestPathInTree (Graph mstGraph, std::list<int>& longest_path)
{
	const int num_nodes = num_vertices(mstGraph);
	std::list<int> provisional_path;

	int source = 0; //the source is arbitrarily chosen (it can be whatever node)
	int furthest_node = furthestNode (furthestNode (source, mstGraph, provisional_path), mstGraph, longest_path);
}

bool checkMSTToFindStrips (Graph mstGraph)
{
	const int num_nodes = num_vertices(mstGraph);
	
	//Find the longest path in MST
	std::list<int> longest_path;
	findLongestPathInTree(mstGraph, longest_path);
	int max_path_length = longest_path.size();
	std::cerr << "lenght longest path: " << max_path_length << std::endl;

	//If the length of the longest path in MST in more than MST_RATIO*'total number of nodes'
	//the found patch is a strip
	if (max_path_length>MST_RATIO*num_nodes)
	{
		std::cerr << "---->>> IT IS A STRIP" << std::endl;
		return true;
	}
	{
		std::cerr << "---->>> IT IS not A STRIP" << std::endl;
		return false;
	}

/* CODE FOR FINDING NODES WITH DEGREE 1
bool found = false;
GraphNoWeight::vertex_iterator vertexIt, vertexEnd; tie(vertexIt, vertexEnd) = vertices(mstGraph);
vertex_descriptor sourceNode = *vertexIt;
std::cout << "Initialize the source node: index " << *vertexIt << ", degree = "  << degree(*vertexIt, mstGraph) << "\n";
for (; vertexIt != vertexEnd; ++vertexIt) 
{ 
	if (degree(*vertexIt, mstGraph)==1)
	{
		std::cout << "degree for " << *vertexIt << ": "  << degree(*vertexIt, mstGraph) << "\n";
		leafNodeList.push_back(*vertexIt);
	}
	else
	{
		if (!found)
		{
			vertex_descriptor sourceNode = *vertexIt;
			found = true;
			std::cout << "Changed the source node: index " << 
				*vertexIt << ", degree = "  << degree(*vertexIt, mstGraph) << "\n";
		}
	}
}*/
}

std::pair<bool,std::list<std::pair<vertex_descriptor,vertex_descriptor>>> longLinks(Graph mstGraph, 
	std::vector<uint32_t, std::allocator<char32_t>> p)
{
	typedef boost::graph_traits<Graph>::edge_iterator edge_iterator;
	typedef std::pair<edge_iterator, edge_iterator> edge_pair;
	property_map<Graph, edge_weight_t>::type weightmap = get(edge_weight, mstGraph);

	//Compute average lenght of the edges in the MST	
	edge_pair ep;
	double sum=0, average_weight;
	unsigned int num_of_edges_mst = 0;
	for (ep = edges(mstGraph); ep.first != ep.second; ++ep.first)
	{
		sum += weightmap(*ep.first);
		++num_of_edges_mst;
	}
	average_weight=sum/num_of_edges_mst;

	//Long link search:
	double threshold = (1/MST_RATIO2)*average_weight;  //set this threshold proportionally to the lenght average of the edges
	std::list<std::pair<vertex_descriptor,vertex_descriptor>> longLinkList;

	for (ep = edges(mstGraph); ep.first != ep.second; ++ep.first)
	{
		//std::cerr << "Edge:  (" << source(*ep.first,mstGraph) << "," << target(*ep.first,mstGraph) << ")" << std::endl;
		//std::cerr << "weight: " << weightmap(*ep.first) << std::endl;
		if (weightmap(*ep.first)> threshold)
		{
			std::pair<int,int> bb = std::make_pair(source(*ep.first,mstGraph), target(*ep.first,mstGraph));
			longLinkList.push_back(bb);
		}
	}

	if (longLinkList.size()>0)
	{
		std::cerr << "---> FOUND LONG LINKS " << std::endl;
		return std::make_pair(true, longLinkList);
	}
	std::cerr << "---> NO LONG LINKS FOUND " << std::endl;
	return std::make_pair(false, longLinkList);
	/*PREVIOUS VERSION
	property_map<Graph, edge_weight_t>::type weightmap = get(edge_weight, g);
	
	//Compute average lenght of the edges in the MST
		
	std::pair<edge_descriptor, bool> current_edge;
	double sum=0, average_weight;
	unsigned int num_of_edges_mst = 0;
	for (std::size_t i = 0; i != p.size(); ++i)
	{
		if (p[i] != i)
		{
			current_edge  = edge(i, p[i], g);
			sum += weightmap(current_edge.first);
			++num_of_edges_mst;
		}	  
	}
	average_weight=sum/num_of_edges_mst;

	//Long link search:
	double threshold = (1/MST_RATIO2)*average_weight;  //set this threshold proportionally to the lenght average of the edges
	std::list<std::pair<vertex_descriptor,vertex_descriptor>> longLinkList;

	for (std::size_t i = 0; i != p.size(); ++i)
	{
		if (p[i] != i)
		{
			current_edge  = edge(i, p[i], g);
			//std::cerr << " " << current_edge.first << std::endl;
			//std::cerr << " " << weightmap(current_edge.first) << std::endl;
			if (weightmap(current_edge.first)> threshold)
			{
				//longLinkList.push_back(current_edge.first);
				std::pair<int,int> bb = std::make_pair(i, p[i]);
				longLinkList.push_back(bb);
			}
		}
	}


	if (longLinkList.size()>0)
	{
		std::cerr << "---> FOUND LONG LINKS " << std::endl;
		return std::make_pair(true, longLinkList);
	}
	std::cerr << "---> NO LONG LINKS FOUND " << std::endl;
	return std::make_pair(false, longLinkList);*/
}

Graph buildGraphFromCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, const int num_nodes)
{
	//typedef boost::graph_traits<Graph>::edge_iterator edge_iterator;
	//const int num_nodes = cloud->size();
	Graph g(num_nodes);

	//Example uses an array, but we can easily use another container type
	//to hold our edges. 
	//typedef Graph::edge_property_type Weight;
	
	std::vector<E> edgeVec;
	std::vector<float> weightVec;
	Eigen::Vector3f a, b;
	double current_weight;
	for (std::size_t i = 0; i < (num_nodes-1); ++i)
	{
		for (std::size_t j = i+1; j < (num_nodes); ++j)
		{
			if (i!=j)
			{
				a = Eigen::Vector3f(cloud->at(i).getArray3fMap());
				b = Eigen::Vector3f(cloud->at(j).getArray3fMap());
				current_weight = pcl::geometry::distance(a,b);
				add_edge(i, j, Weight(current_weight), g);			
				//std::cerr << "Distance d(" << i << "," << j << ") = " << i << pcl::geometry::distance(a,b) << std::endl;
			}	
		}
	}
	//property_map<Graph, edge_weight_t>::type weightmap = get(edge_weight, g);

	return g;
}

std::pair<Graph, std::vector<uint32_t, std::allocator<char32_t>>> getMSTGraph(
	pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, const int num_nodes)
{
	Graph g = buildGraphFromCloud(cloud, num_nodes);
	std::vector<uint32_t, std::allocator<char32_t>> p = mstPrim (g);

	property_map<Graph, edge_weight_t>::type weightmap = get(edge_weight, g);

	//Build the graph of the MST
	Graph mstGraph(num_nodes);
	//std::vector<E> edgeVec;
	std::pair<edge_descriptor, bool> current_edge;
	for (std::size_t i = 0; i < p.size(); ++i)
	{
		if (p[i] != i)
		{
			current_edge  = edge(i, p[i], g);
			add_edge(i, p[i], weightmap(current_edge.first), mstGraph);
			//std::cerr << "new mst edge (" << i << "," << p[i] << "). WEIGTH= " << weightmap(current_edge.first) <<  std::endl;
		}
	}
	/*int num_of_edges = num_edges(mstGraph);
	std::cerr << "FINE CREAZIONE mstGraph. Numero di edge aggiunti: " << num_of_edges <<  std::endl;*/

	return std::make_pair(mstGraph, p);
}

std::tuple<bool, Graph, std::vector<uint32_t, std::allocator<char32_t>>> isStrip(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
	const int num_nodes = cloud->points.size ();
	Graph mstGraph(num_nodes);
	std::vector<uint32_t, std::allocator<char32_t>> p;

	std::tie(mstGraph, p) = getMSTGraph(cloud, num_nodes);
	
	//check if it is strip
	if (checkMSTToFindStrips(mstGraph))
	{
		return std::make_tuple(true, mstGraph, p);
	}
	return std::make_tuple(false, mstGraph, p);
}

bool isToBeSeparated(std::string surface_type,
	pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, Graph mstGraph, std::vector<uint32_t,std::allocator<char32_t>> p, 
	std::list<pcl::PointCloud<pcl::PointXYZ>::Ptr>& group_clouds)
{
	//check if the patch has to be separated
	bool longLinksExist;
	std::list<std::pair<vertex_descriptor,vertex_descriptor>> listLongLinks;
	std::tie(longLinksExist, listLongLinks) = longLinks(mstGraph, p);
	if (longLinksExist)
	{
		//LONG LING PROCEDURE
		int num_of_edges = num_edges(mstGraph);
		std::cerr << "Number of edges before deleting long edges: " << num_of_edges << std::endl;

		//Delete the long edges
		for (std::size_t i = 0; i != listLongLinks.size(); ++i)
		{
			std::list<std::pair<vertex_descriptor,vertex_descriptor>>::iterator it_list = 
				std::next(listLongLinks.begin(), i);
			remove_edge((*it_list).first, (*it_list).second, mstGraph);
		}

		std::cerr << "Number of edges after deleting long edges: " << num_edges(mstGraph) << std::endl;

		//Separate the resulting graph in connected subgraphs (still (sub) trees)
		int num_vert = num_vertices(mstGraph);
		std::vector<int> component(num_vert);
		int num = connected_components(mstGraph, &component[0]);
    
		std::vector<int>::size_type i;
		/*cout << "Total number of components: " << num << endl;
		for (i = 0; i != component.size(); ++i)
		  cout << "Vertex " << i <<" is in component " << component[i] << endl;
		cout << endl;*/

		//cout << "component.size=" << component.size() << endl;

		//std::list<pcl::PointCloud<pcl::PointXYZ>::Ptr> subgraphs;
		Eigen::Vector3f point;
		for (std::size_t i = 0; i < (num); ++i)
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr subcloud (new pcl::PointCloud<pcl::PointXYZ>);
			for (std::size_t j = 0; j < (component.size()); ++j)
			{
				if (component[j]==i)
				{
					point = Eigen::Vector3f(cloud->at(j).getArray3fMap());
					subcloud->push_back (pcl::PointXYZ (point[0], point[1], point[2])); 
					//subcloud.push_back (pcl::PointXYZ (rand(), rand(), rand())); 
				}
			}

			/*std::cerr << "New subgraph with " << subcloud.width << " points:" << std::endl;
			for (std::size_t j = 0; j < (subcloud.width); ++j)
			{
				std::cerr << "Added: (" << subcloud.at(j).x << "," << subcloud.at(j).y << "," << subcloud.at(j).z << ")" << std::endl;
			}*/
			group_clouds.push_back(subcloud);
		}

		int num_of_subclouds = group_clouds.size();
		std::cerr << std::endl;
		std::cerr << "Num of subgraphs created =  " << num_of_subclouds << std::endl;
		// If the number of subclouds created is >1 it returns true, otherwise false	
		if (num_of_subclouds>1)
		{
			return true;
		}
		return false;

		//SO FAR: ho creato i sotto cloud dopo aver cancellato i long edges
		//adesso c'e' da (mandarli in output con la funzione), applicare di nuovo il fitting con lo stesso tipo di superficie
		//per vedere se si possono catturare altri punti da quelli disponibili.
	}

}

void separateAndOptimizePlanarPatches(std::list<pcl::PointCloud<pcl::PointXYZ>::Ptr> &group_clouds, 
	pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_of_the_other_points)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_to_analyse (new pcl::PointCloud<pcl::PointXYZ>);
	int num_of_subclouds = group_clouds.size();
	for (std::size_t i = 0; i < num_of_subclouds; ++i)
	{
		// --- STEPS:
		//RANSAC with the same surface type
		//enlarge consensus set (see: http://pointclouds.org/documentation/tutorials/model_outlier_removal.php#model-outlier-removal)
		//check if it is strips, or groups to be separated
		//update point cloud of points still available in the process

		//PLANE
		std::vector<int> inliers;

	    // created RandomSampleConsensus object and compute the appropriated model for the i-th subcloud
		std::list<pcl::PointCloud<pcl::PointXYZ>::Ptr>::iterator it = std::next(group_clouds.begin(), i);
		pcl::PointCloud<pcl::PointXYZ>::Ptr starting_cloud = *it;
		std::cerr << "New subcloud to analyse. Num of points = " << starting_cloud->size() << std::endl;
	    pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr 
		  model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (starting_cloud));
		// Create a point cloud to store the inliers
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_inliers (new pcl::PointCloud<pcl::PointXYZ>);
		// Create a point cloud to store the further inliers, when we try to enlarge the consensus set
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_more_inliers (new pcl::PointCloud<pcl::PointXYZ>);
	
		// Compute the RANSAC model 
		pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_p);
		ransac.setDistanceThreshold (PLANE_TOL);
		ransac.computeModel();
		ransac.getInliers(inliers);
	 
		// copies all inliers of the model computed to another PointCloud called cloud_inliers
		// Obs: actually 'starting_cloud' and 'cloud_inliers' should contain almost the same points, 
		// as the starting cloud contains points that have already been fitted previously in the same patch, 
		// so it is a "clean" set of points (I don't expect any outlier)
	    pcl::copyPointCloud<pcl::PointXYZ>(*starting_cloud, inliers,*cloud_inliers);
		std::cerr << "Model recomputed. Now num of inliers = " << cloud_inliers->size() << std::endl;
		
		// Get the parameters of the found model
		Eigen::VectorXf model_coef;
		ransac.getModelCoefficients(model_coef);
		size_t num_parameters = model_coef.size();
		pcl::ModelCoefficients model_coeff_format;
	    model_coeff_format.values.resize (num_parameters);
		std::cerr << std::endl << "---> Refitting for plane. New parameters:" << model_coeff_format.values[i] << std::endl;
		for (std::size_t i = 0; i < num_parameters; ++i)
		{
			model_coeff_format.values[i] = model_coef[i];
			std::cerr << "param plane: " << model_coeff_format.values[i] << std::endl;
		}
		std::cerr << std::endl;

		// Try to enlarge the consensus set
	    pcl::ModelOutlierRemoval<pcl::PointXYZ> plane_filter;
		plane_filter.setModelCoefficients (model_coeff_format);
		plane_filter.setThreshold (PLANE_TOL);
		plane_filter.setModelType (pcl::SACMODEL_PLANE);
		plane_filter.setInputCloud (cloud_of_the_other_points);   //cloud containing the points that are still available, i.e. not fitted yet in any patch
		plane_filter.filter (*cloud_more_inliers);                //cloud containing other found inliers
		std::cerr << "Num of new of inliers = " << cloud_more_inliers->size() << std::endl;

		// Concatenate the main inlier set with the newly found inliers
		*cloud_inliers += *cloud_more_inliers;
		std::cerr << "TOT num of inliers = " << cloud_inliers->size() << std::endl;
	}
}

void separateAndOptimizeCylindricalPatches(std::list<pcl::PointCloud<pcl::PointXYZ>::Ptr> &group_clouds, 
	pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_of_the_other_points)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_to_analyse (new pcl::PointCloud<pcl::PointXYZ>);
	int num_of_subclouds = group_clouds.size();
	for (std::size_t i = 0; i < num_of_subclouds; ++i)
	{
		//CYLINDER
		std::vector<int> inliers;

	    // created RandomSampleConsensus object and compute the appropriated model for the i-th subcloud
		std::list<pcl::PointCloud<pcl::PointXYZ>::Ptr>::iterator it = std::next(group_clouds.begin(), i);
		pcl::PointCloud<pcl::PointXYZ>::Ptr starting_cloud = *it;
		std::cerr << "New subcloud to analyse. Num of points = " << starting_cloud->size() << std::endl;
		pcl::SampleConsensusModelCylinder<pcl::PointXYZ, pcl::Normal>::Ptr 
		  model_c (new pcl::SampleConsensusModelCylinder<pcl::PointXYZ, pcl::Normal> (starting_cloud));
		// Create a point cloud to store the inliers
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_inliers (new pcl::PointCloud<pcl::PointXYZ>);
		// Create a point cloud to store the further inliers, when we try to enlarge the consensus set
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_more_inliers (new pcl::PointCloud<pcl::PointXYZ>);
	
		// Compute the RANSAC model 
		pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_c);
		ransac.setDistanceThreshold (CYL_TOL);
		ransac.computeModel();
		ransac.getInliers(inliers);
	 
		// copies all inliers of the model computed to another PointCloud called cloud_inliers
		// Obs: actually 'starting_cloud' and 'cloud_inliers' should contain almost the same points, 
		// as the starting cloud contains points that have already been fitted previously in the same patch, 
		// so it is a "clean" set of points (I don't expect any outlier)
	    pcl::copyPointCloud<pcl::PointXYZ>(*starting_cloud, inliers,*cloud_inliers);
		std::cerr << "Model recomputed. Now num of inliers = " << cloud_inliers->size() << std::endl;
		
		// Get the parameters of the found model
		Eigen::VectorXf model_coef;
		ransac.getModelCoefficients(model_coef);
		size_t num_parameters = model_coef.size();
		pcl::ModelCoefficients model_coeff_format;
	    model_coeff_format.values.resize (num_parameters);
		std::cerr << std::endl << "---> Refitting for cylinder. New parameters:" << model_coeff_format.values[i] << std::endl;
		for (std::size_t i = 0; i < num_parameters; ++i)
		{
			model_coeff_format.values[i] = model_coef[i];
			std::cerr << "param cylinder: " << model_coeff_format.values[i] << std::endl;
		}
		std::cerr << std::endl;

		// Try to enlarge the consensus set
	    pcl::ModelOutlierRemoval<pcl::PointXYZ> cyl_filter;
		//cyl_filter.setInputNormals
		cyl_filter.setModelCoefficients (model_coeff_format);
		cyl_filter.setThreshold (CYL_TOL);
		cyl_filter.setModelType (pcl::SACMODEL_CYLINDER);
		cyl_filter.setInputCloud (cloud_of_the_other_points);   //cloud containing the points that are still available, i.e. not fitted yet in any patch
		cyl_filter.filter (*cloud_more_inliers);                //cloud containing other found inliers
		std::cerr << "Num of new of inliers = " << cloud_more_inliers->size() << std::endl;

		// Concatenate the main inlier set with the newly found inliers
		*cloud_inliers += *cloud_more_inliers;
		std::cerr << "TOT num of inliers = " << cloud_inliers->size() << std::endl;
	}
}

void separateAndOptimizeConicalPatches(std::list<pcl::PointCloud<pcl::PointXYZ>::Ptr> &group_clouds, 
	pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_of_the_other_points)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_to_analyse (new pcl::PointCloud<pcl::PointXYZ>);
	int num_of_subclouds = group_clouds.size();
	for (std::size_t i = 0; i < num_of_subclouds; ++i)
	{
		//CONE
		std::vector<int> inliers;

	    // created RandomSampleConsensus object and compute the appropriated model for the i-th subcloud
		std::list<pcl::PointCloud<pcl::PointXYZ>::Ptr>::iterator it = std::next(group_clouds.begin(), i);
		pcl::PointCloud<pcl::PointXYZ>::Ptr starting_cloud = *it;
		std::cerr << "New subcloud to analyse. Num of points = " << starting_cloud->size() << std::endl;
		pcl::SampleConsensusModelCone<pcl::PointXYZ, pcl::Normal>::Ptr 
		  model_c (new pcl::SampleConsensusModelCone<pcl::PointXYZ, pcl::Normal> (starting_cloud));
		// Create a point cloud to store the inliers
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_inliers (new pcl::PointCloud<pcl::PointXYZ>);
		// Create a point cloud to store the further inliers, when we try to enlarge the consensus set
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_more_inliers (new pcl::PointCloud<pcl::PointXYZ>);
	
		// Compute the RANSAC model 
		//model_c->setInputNormals(cloud_normals); 
		pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_c);
		ransac.setDistanceThreshold (CONE_TOL);
		ransac.computeModel();
		ransac.getInliers(inliers);
	 
		// copies all inliers of the model computed to another PointCloud called cloud_inliers
		// Obs: actually 'starting_cloud' and 'cloud_inliers' should contain almost the same points, 
		// as the starting cloud contains points that have already been fitted previously in the same patch, 
		// so it is a "clean" set of points (I don't expect any outlier)
	    pcl::copyPointCloud<pcl::PointXYZ>(*starting_cloud, inliers,*cloud_inliers);
		std::cerr << "Model recomputed. Now num of inliers = " << cloud_inliers->size() << std::endl;
		
		// Get the parameters of the found model
		Eigen::VectorXf model_coef;
		ransac.getModelCoefficients(model_coef);
		size_t num_parameters = model_coef.size();
		pcl::ModelCoefficients model_coeff_format;
	    model_coeff_format.values.resize (num_parameters);
		std::cerr << std::endl << "---> Refitting for cone. New parameters:" << model_coeff_format.values[i] << std::endl;
		for (std::size_t i = 0; i < num_parameters; ++i)
		{
			model_coeff_format.values[i] = model_coef[i];
			std::cerr << "param cone: " << model_coeff_format.values[i] << std::endl;
		}
		std::cerr << std::endl;

		// Try to enlarge the consensus set
	    pcl::ModelOutlierRemoval<pcl::PointXYZ> cone_filter;
		cone_filter.setModelCoefficients (model_coeff_format);
		cone_filter.setThreshold (CONE_TOL);
		cone_filter.setModelType (pcl::SACMODEL_CONE);
		cone_filter.setInputCloud (cloud_of_the_other_points);   //cloud containing the points that are still available, i.e. not fitted yet in any patch
		cone_filter.filter (*cloud_more_inliers);                //cloud containing other found inliers
		std::cerr << "Num of new of inliers = " << cloud_more_inliers->size() << std::endl;

		// Concatenate the main inlier set with the newly found inliers
		*cloud_inliers += *cloud_more_inliers;
		std::cerr << "TOT num of inliers = " << cloud_inliers->size() << std::endl;
	}
}

void separateAndOptimizePatches(std::string surface_type, std::list<pcl::PointCloud<pcl::PointXYZ>::Ptr> &group_clouds, 
	pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_of_the_other_points)
{
	if(surface_type=="plane")
	{
		separateAndOptimizePlanarPatches(group_clouds, cloud_of_the_other_points);
	}
	if (surface_type=="cylinder")
	{
		separateAndOptimizePlanarPatches(group_clouds, cloud_of_the_other_points);
	}
	if (surface_type=="cone")
	{
		separateAndOptimizePlanarPatches(group_clouds, cloud_of_the_other_points);
	}
}

bool checkPatchValidity(std::string surface_type, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_on_the_patch, 
	pcl::PointCloud<pcl::Normal>::Ptr &cloud_normals_on_the_patch,
	pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_of_the_other_points, 
	pcl::PointCloud<pcl::Normal>::Ptr &cloud_normals_of_the_other_points)
{
	// Check if it is strip
	bool thisPatchIsStrip;
	Graph mstGraph;
	std::vector<uint32_t, std::allocator<char32_t>> p;
	std::tie(thisPatchIsStrip, mstGraph, p) = isStrip(cloud_on_the_patch);
	////(keep 3 mstGraph and p because they are used by the SeparatedGroup procedure)
	//if (thisPatchIsStrip)
	//{
	//	return false;
	//}
	//else
	//{
		// Check if there are groups to be separated
		std::list<pcl::PointCloud<pcl::PointXYZ>::Ptr> group_clouds;
		if (isToBeSeparated(surface_type, cloud_on_the_patch, mstGraph, p, group_clouds))
		{
			separateAndOptimizePatches(surface_type, group_clouds, cloud_of_the_other_points);
			return false;
		}
	/*}*/
	return true;
}
