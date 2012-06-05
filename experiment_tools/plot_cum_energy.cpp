#include <cassert>
#include <cmath>
#include <fstream>
#include <iostream>
#include <vector>
#include "../proto/proto.pb.h"
#include "discpp.h"

using namespace std;

void init_dislin(Dislin& g) {

	g.metafl ("cons");
	g.scrmod ("revers");
	g.disini ();
	g.pagera ();
	g.complx ();
	g.axspos (450, 1800);
	g.axslen (2200, 1200);

	g.name   ("X-axis", "x");
	g.name   ("Y-axis", "y");

	g.labdig (-1, "x");
	g.ticks  (9, "x");
	g.ticks  (10, "y");

	g.titlin ("Energy over generations", 1);
	g.titlin ("Energy", 3);

	int ic = g.intrgb (0.95,0.95,0.95);
	g.axsbgd (ic);

	//g.graf   (0.0, 360.0, 0.0, 90.0, -1.0, 1.0, -1.0, 0.5);
	g.setrgb (0.7, 0.7, 0.7);
	g.grid   (1, 1);

	g.color  ("fore");
	g.height (50);
	g.title  ();
}

void plot_curve(Dislin& g, const vector<double>& x, const vector<double>& y) {
	g.color  ("red");
	g.curve  (&x[0], &y[0], x.size());
	g.disfin ();
}

int main () {
	//ifstream ifs("..\\experiment_data\\8_seconds_flight\\bird_data.pbdata", ios::in | ios::binary);
	ifstream ifs("C:\\Users\\k\\bird_data.pbdata", ios::in | ios::binary);
	assert(ifs.is_open());

	//string string_data;
	//ifs >> string_data;

	proto::BirdOptimizerData data;
	if (!data.ParseFromIstream(&ifs)) {
		cerr << "Cannot parse protobuf!" << endl;
	}

	//cout << data.DebugString();
	assert(data.IsInitialized());

	cout << "Choose from [0-" << data.result_size() << ") results.";

	int result_ii;
	cin >> result_ii;
	const proto::BirdOptimizerResult& result = data.result(result_ii);

	const proto::WingbeatData& wingbeat_data = result.bird().wingbeatdata();

	vector<double> x1;
	vector<double> x2;
	vector<double> t;
	x1.reserve(wingbeat_data.sample_size());
	x2.reserve(wingbeat_data.sample_size());
	t.reserve(wingbeat_data.sample_size());

	for (int ii = 0; ii < wingbeat_data.sample_size(); ++ii) {
		t.push_back(ii);
		x1.push_back(wingbeat_data.sample(ii).feather());
		x2.push_back(wingbeat_data.sample(ii).wing());
	}

	Dislin g;
	init_dislin(g);
	g.graf(0.0, 100.0, 0.0, 100.0, -70.0, 70.0, -70.0, 70.0);

	g.color  ("red");
	g.curve  (&t[0], &x1[0], x1.size());

	g.color  ("blue");
	g.curve  (&t[0], &x2[0], x2.size());
	g.disfin ();
	
	return 0;
}