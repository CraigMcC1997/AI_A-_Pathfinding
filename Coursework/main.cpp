


//=======================================================================
//	Copyright (c) 2004 Kristopher Beevers
//
//	Distributed under the Boost Software License, Version 1.0. (See
//	accompanying file LICENSE_1_0.txt or copy at
//	http://www.boost.org/LICENSE_1_0.txt)
//=======================================================================



//=======================================================================
//	Code based on and largely inspired by Lab1: astar cities
//	from the Computer Game A.I. Module
//	Year 3, Computer Games Technologies
//=======================================================================



#include <boost/graph/astar_search.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/random.hpp>
#include <boost/random.hpp>
#include <boost/graph/graphviz.hpp>
#ifndef _WIN32
#include <sys/time.h>
#endif
#include <vector>
#include <list>
#include <iostream>
#include <fstream>
#include <math.h>    // for sqrt

using namespace boost;
using namespace std;

struct location {
	float x, y;
};
typedef float cost;

template <class Name, class LocMap>

class node_writer {
private:
	Name name;
	LocMap loc;
	float minx, maxx, miny, maxy;
	unsigned int ptx, pty;

public:
	node_writer(Name n, LocMap l, float _minx, float _maxx, float _miny, float _maxy, unsigned int _ptx, unsigned int _pty)
		: name(n), loc(l), minx(_minx), maxx(_maxx), miny(_miny), maxy(_maxy), ptx(_ptx), pty(_pty) {}

	template <class Vertex>

	void operator()(ostream& out, const Vertex& v) const
	{
		float px = 1 - (loc[v].x - minx) / (maxx - minx);
		float py = (loc[v].y - miny) / (maxy - miny);

		out << "[label=\"" << name[v] << "\", pos=\""
			<< static_cast<unsigned int> (ptx * px) << " , "
			<< static_cast<unsigned int> (pty * py)
			<< "\", fonsize=\"8\"]";
	}
};

template <class WeightMap>

class time_writer
{
private:
	WeightMap wm;
public:
	time_writer(WeightMap w) : wm(w) {}

	template<class Edge>

	void operator()(ostream& out, const Edge &e) const
	{
		out << "[label=\"" << wm[e] << "\", fontsize=\"8\"]";
	}
};

template<class Graph, class CostType, class LocMap>

class distance_heuristic : public astar_heuristic<Graph, CostType>
{
private:
	LocMap m_location;
	Vertex m_goal;

public:
	typedef typename graph_traits<Graph>::vertex_descriptor Vertex;

	distance_heuristic(LocMap l, Vertex goal) : m_location(l), m_goal(goal) {}

	CostType operator()(Vertex u)
	{
		CostType dx = m_location[m_goal].x - m_location[u].x;
		CostType dy = m_location[m_goal].y - m_location[u].y;
		return ::sqrt(dx * dx + dy * dy);
	}
};

struct found_goal {};

template<class Vertex>

class astar_goal_visitor : public boost::default_astar_visitor
{
private:
	Vertex m_goal;
public:
	astar_goal_visitor(Vertex goal) : m_goal(goal) {}

	template <class Graph>
	void examine_vertex(Vertex u, Graph& g) {
		if (u == m_goal)
			throw found_goal();
	}
};


int main(int argc, char **argv)
{
	typedef adjacency_list<listS, vecS, undirectedS, no_property, property<edge_weight_t, cost> > mygraph_t;
	typedef property_map<mygraph_t, edge_weight_t>::type WeightMap;
	typedef mygraph_t::vertex_descriptor vertex;
	typedef mygraph_t::edge_descriptor edge_descriptor;
	typedef std::pair<int, int> edge;

	enum nodes {
		//put nodes here
		Zero, One, Two, Three, Four, Five, Six, Seven, Eight, Nine, Ten,
		Eleven, Twelve, Thirteen, Fourteen, Fifteen, Sixteen, Seventeen,
		Eighteen, Nineteen, Twenty, TwentyOne, TwentyTwo, TwentyThree,
		TwentyFour, TwentyFive, TwentySix, TwentySeven, TwentyEight,
		TwentyNine, Thirty, ThirtyOne, ThirtyTwo, ThirtyThree, ThirtyFour,
		ThirtyFive, ThirtySix, ThirtySeven, ThirtyEight, ThirtyNine, Forty,
		FortyOne, FortyTwo, FortyThree, FortyFour, FortyFive, FortySix,
		FortySeven, FortyEight, FortyNine, Fifty, FiftyOne, FiftyTwo, FiftyThree,
		FiftyFour, FiftyFive, FiftySix, FiftySeven, FiftyEight, FiftyNine,
		Sixty, SixtyOne, SixtyTwo, SixtyThree
	};

	const char *name[] = {
		//names of each node
		"Zero", "One", "Two", "Three", "Four", "Five", "Six", "Seven", "Eight", "Nine", "Ten",
		"Eleven", "Twelve", "Thirteen", "Fourteen", "Fifteen", "Sixteen", "Seventeen",
		"Eighteen", "Nineteen", "Twenty", "TwentyOne", "TwentyTwo", "TwentyThree",
		"TwentyFour", "TwentyFive", "TwentySix", "TwentySeven", "TwentyEight",
		"TwentyNine", "Thirty", "ThirtyOne", "ThirtyTwo", "ThirtyThree", "ThirtyFour",
		"ThirtyFive", "ThirtySix", "ThirtySeven", "ThirtyEight", "ThirtyNine", "Forty",
		"FortyOne", "FortyTwo", "FortyThree", "FortyFour", "FortyFive", "FortySix",
		"FortySeven", "FortyEight", "FortyNine", "Fifty", "FiftyOne", "FiftyTwo", "FiftyThree",
		"FiftyFour", "FiftyFive", "FiftySix", "FiftySeven", "FiftyEight", "FiftyNine",
		"Sixty", "SixtyOne", "SixtyTwo", "SixtyThree"
	};

	location locations[] = {
		//location values
		{165, 0}, {390, 0}, {495, 0}, {270, 15}, {360, 15}, {420, 45}, {300, 60}, {240, 75}, {330, 75}, {345, 75},
		{435, 75}, {525, 75}, {570, 75}, {300, 105}, {555, 105}, {390, 120}, {315, 135}, {60, 150}, {375, 150}, {270, 165},
		{300, 180}, {435, 195}, {570, 195}, {45, 225}, {285, 225}, {585, 225}, {120, 240}, {165, 240}, {45, 270}, {255, 270}, 
		{180, 285}, {195, 315}, {525, 315}, {345, 345}, {270, 360}, {525, 360}, {60, 375}, {165, 375}, {390, 375}, {405, 375},
		{75, 405}, { 435,405 }, { 135,420 }, { 315,420 }, { 480,420 }, { 510,420 }, { 570,420 }, { 360,435 }, { 375,450 },
		{ 435,450 }, { 345,465 }, { 465,480 }, { 285,495 }, { 555,495 }, { 270,525 }, { 150,540 }, { 90,555 }, { 120,555 },
		{ 210,555 }, { 570,555 }, { 270,570 }, { 420,570 }, { 435,570 }, { 495,570 }
	};

	edge edge_array[] = {
		//add edges between adjacent nodes
		edge(Zero,Three), edge(Zero, Seven), edge(Zero, Six), edge(Zero, Thirteen),
		edge(One, Four), edge(One, Five), edge(One, Nine), edge(One, Ten),
		edge(Two, Eleven), edge(Two, Five), edge(Two, Ten),edge(Two, Twelve),
		edge(Three, Six), edge(Three, Seven), edge(Three, Eight),
		edge(Four, Nine), edge(Four, Five), edge(Four, Eight),
		edge(Five, Ten),
		edge(Six, Eight), edge(Six, Thirteen),
		edge(Seven, Thirteen), edge(Three, Eight),
		edge(Nine, Thirteen), edge(Nine, Fifteen),
		edge(Ten, Fifteen),
		edge(Eleven, Fourteen), edge(Eleven, Twelve), edge(Eleven, TwentyTwo),
		edge(Twelve, Fourteen), edge(Twelve, TwentyTwo),
		edge(Fourteen, TwentyTwo), edge(Fourteen, TwentyFive),
		edge(Fifteen, Eighteen), edge(Fifteen, Sixteen),
		edge(Sixteen, Twenty), edge(Sixteen, Nineteen), edge(Sixteen, Eighteen),
		edge(Seventeen, TwentyThree), edge(Seventeen, TwentySix), edge(Seventeen, TwentyEight), edge(Seventeen, TwentySeven),
		edge(Eighteen, TwentyOne), edge(Eighteen, Twenty),
		edge(Nineteen, Twenty), edge(Nineteen, TwentyFour),
		edge(Twenty, TwentyFour),
		edge(TwentyOne, TwentyTwo), edge(TwentyOne, TwentyFour), edge(TwentyOne, TwentyFive),
		edge(TwentyThree, TwentyEight), edge(TwentyThree, TwentySix), edge(TwentyThree, TwentySeven),
		edge(TwentyFour, TwentyNine),
		edge(TwentyFive, ThirtyTwo), edge(TwentyFive, ThirtyFive),
		edge(TwentySix, TwentySeven), edge(TwentySix, Thirty),
		edge(TwentySeven, Thirty),
		edge(TwentyEight, ThirtySix), edge(TwentyEight, Thirty),
		edge(TwentyNine, ThirtyOne), edge(TwentyNine, Thirty),
		edge(ThirtyOne, ThirtySeven), edge(ThirtyOne, ThirtyFour), edge(ThirtyOne, FortyTwo),
		edge(ThirtyTwo, ThirtyFive), edge(ThirtyTwo, FortyFive), edge(ThirtyTwo, FortyFour),
		edge(ThirtyThree, ThirtyEight), edge(ThirtyThree, ThirtyNine), edge(ThirtyThree, ThirtyFour), edge(ThirtyThree, FortyThree),
		edge(ThirtyFour, FortyThree), edge(ThirtyFour, ThirtySeven),
		edge(ThirtyFive, FortyFive), edge(ThirtyFive, FortyFour),
		edge(ThirtySix, Forty), edge(ThirtySix, FortyTwo), edge(ThirtySix, ThirtySeven),
		edge(ThirtySeven, FortyTwo),
		edge(ThirtyEight, ThirtyNine), edge(ThirtyEight, FortyOne), edge(ThirtyEight, FortySeven),
		edge(ThirtyNine, FortyTwo), edge(ThirtyNine, FortySeven),
		edge(Forty, FortyTwo), edge(Forty, FiftySix), edge(Forty, FiftyFive),
		edge(FortyOne, FortyNine), edge(FortyOne, FortyFour),
		edge(FortyThree, FortySeven), edge(FortyThree, Fifty),
		edge(FortyFour, FortyFive),
		edge(FortyFive, FortySix),
		edge(FortySix, FiftyThree), edge(FortySix, FiftyOne), edge(FortySix, FiftyNine),
		edge(FortySeven, FortyEight),
		edge(FortyEight, Fifty), edge(FortyEight, FortyNine), edge(FortyEight, FiftyOne),
		edge(FortyNine, FiftyOne), edge(FortyNine, FortyNine),
		edge(Fifty, FiftyTwo),
		edge(FiftyOne, FiftyThree),
		edge(FiftyTwo, FiftyFour), edge(FiftyTwo, Sixty), edge(FiftyTwo, FiftyEight),
		edge(FiftyThree, FiftyNine), edge(FiftyThree, SixtyThree),
		edge(FiftyFour, Sixty), edge(FiftyFour, FiftyEight), edge(FiftyFour, FiftyFive),
		edge(FiftyFive, FiftySeven), edge(FiftyFive, FiftySix),
		edge(FiftySix, FiftySeven), edge(FiftySix, FiftyEight),
		edge(FiftySeven, FiftyEight), edge(FiftySeven, Sixty),
		edge(FiftyNine, SixtyThree), edge(FiftyNine, SixtyTwo),
		edge(Sixty, SixtyOne),
		edge(SixtyOne, SixtyTwo), edge(SixtyOne, SixtyThree),
		edge(SixtyTwo, SixtyThree)
	};

	unsigned int num_edges = sizeof(edge_array) / sizeof(edge);

	cost weights[] = 
	{
		//weights between nodes, i.e cost of travel
		106, 106, 147, 171, 33, 54, 87, 87, 80, 87, 96, 106, 54, 67, 84, 61, 67, 67, 33, 33, 45, 67, 90, 54, 63, 63, 42, 45, 128, 33, 120, 91, 123, 33, 76, 47, 54, 
		61, 76, 108, 120, 138, 75, 80, 33, 61, 106, 47, 135, 152, 152, 45, 76, 120, 54, 120, 147, 45, 75, 47, 106, 135, 75, 76, 67, 87, 120, 30, 91, 100,
		54, 67, 76, 80, 75, 106, 61, 75, 33, 87, 105, 54, 15, 54, 67, 42, 75, 61, 150, 154, 45, 47, 47, 54, 30, 60, 76, 120, 135, 21, 33, 60, 94, 42, 91, 67, 91,
		33, 76, 96, 61, 96, 45, 67, 120, 33, 61, 30, 120, 90, 150, 76, 135, 150, 15, 75, 60
	};

	mygraph_t g(SixtyThree);
	WeightMap weightmap = get(edge_weight, g);

	for (std::size_t j = 0; j < num_edges; ++j)
	{
		edge_descriptor e; 
		bool inserted;
		
		boost::tie(e, inserted) = add_edge(edge_array[j].first, edge_array[j].second, g);

		weightmap[e] = weights[j];
	}

	//user inputs used for creating path start and end points
	vertex start;	
	vertex goal;

	cout << "Starting Node: ";
	cin >> start;
	cout << "Goal Node: ";
	cin >> goal;

	cout << "Start vertex: " << name[start] << endl;
	cout << "Goal vertex: " << name[goal] << endl;

	ofstream dotfile;
	dotfile.open("random64_4_1485816605.dot");
	
	write_graphviz(dotfile, g, node_writer<const char **, location*>
		(name, locations, 45, 585, 0, 570, 480, 400), time_writer<WeightMap>(weightmap));

	vector<mygraph_t::vertex_descriptor> p(num_vertices(g));
	vector<cost> d(num_vertices(g));
	try {
		// call astar named parameter interface
		astar_search_tree
		(g, start,
			distance_heuristic<mygraph_t, cost, location*>
			(locations, goal),
			predecessor_map(make_iterator_property_map(p.begin(), get(vertex_index, g))).
			distance_map(make_iterator_property_map(d.begin(), get(vertex_index, g))).
			visitor(astar_goal_visitor<vertex>(goal)));


	}
	catch (found_goal fg) { // found a path to the goal
		list<vertex> shortest_path;
		for (vertex v = goal;; v = p[v]) {
			shortest_path.push_front(v);
			if (p[v] == v)
				break;
		}
		cout << "Shortest path from " << name[start] << " to "
			<< name[goal] << ": ";
		list<vertex>::iterator i = shortest_path.begin();
		cout << name[start];
		for (++i; i != shortest_path.end(); ++i)
			cout << " -> " << name[*i];
		cout << endl << "Total travel time: " << d[goal] << endl;
		system("PAUSE");	//used for pausing system when not running on VS17
		return 0;
	}

	//error message, no path possible 
	cout << "Didn't find a path from " << name[start] << "to"
		<< name[goal] << "!" << endl;

	return 0;

}