#include <iostream>
#include <rw/rw.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <rw/pathplanning/PathAnalyzer.hpp>
#include <fstream>

using namespace std;
using namespace rw::common;
using namespace rw::math;
using namespace rw::kinematics;
using namespace rw::loaders;
using namespace rw::models;
using namespace rw::pathplanning;
using namespace rw::proximity;
using namespace rw::trajectory;
using namespace rwlibs::pathplanners;
using namespace rwlibs::proximitystrategies;

#define MAXTIME 120000

void makeLuaFile(QPath &path)
{
	/*
	*  Function used to generate a luascipt for the given path
	*/
	ofstream luafile;
	luafile.open("LuaPath.txt");
	luafile << "wc = rws.getRobWorkStudio():getWorkCell()\n"
						 "state = wc:getDefaultState()\n"
						 "device = wc:findDevice(\"KukaKr16\")\n"
						 "gripper = wc:findFrame(\"Tool\");\n"
						 "bottle = wc:findFrame(\"Bottle\");\n"
						 "table = wc:findFrame(\"Table\");\n"

						 "function setQ(q)\n"
						 "qq = rw.Q(#q,q[1],q[2],q[3],q[4],q[5],q[6])\n"
						 "device:setQ(qq,state)\n"
						 "rws.getRobWorkStudio():setState(state)\n"
						 "rw.sleep(0.1)\n"
						 "end\n"

						 "function attach(obj, tool)\n"
						 "rw.gripFrame(obj, tool, state)\n"
						 "rws.getRobWorkStudio():setState(state)\n"
						 "rw.sleep(0.1)\n"
						 "end\n\n\n"
						 "setQ({-3.142, -0.827, -3.002, -3.143, 0.099, -1.573})\n"
						 "attach(bottle,gripper)\n"	<< endl;
	// For loop that inserts the correct path of configurations needed to reach the goal
	for (QPath::iterator value = path.begin(); value < path.end(); value++) {
			luafile << "setQ({"<< (*value)[0];

			for(int i = 1; i < (*value).size(); i++){
					luafile << "," << (*value)[i];
			}
			luafile << "})" << endl;
	}
	luafile <<	"attach(bottle,table)\n"
							"setQ({1.571, 0.006, 0.03, 0.153, 0.762, 4.49})\n";
	luafile.close();
}

bool checkCollisions(Device::Ptr device, const State &state, const CollisionDetector &detector, const Q &q) {
	State testState;
	CollisionDetector::QueryResult data;
	bool colFrom;

	testState = state;
	device->setQ(q,testState);
	colFrom = detector.inCollision(testState,&data);
	if (colFrom) {
		cerr << "Configuration in collision: " << q << endl;
		cerr << "Colliding frames: " << endl;
		FramePairSet fps = data.collidingFrames;
		for (FramePairSet::iterator it = fps.begin(); it != fps.end(); it++) {
			cerr << (*it).first->getName() << " " << (*it).second->getName() << endl;
		}
		return false;
	}
	return true;
}

int main(int argc, char** argv) {
	// Initialize random seed
	rw::math::Math::seed();

	// Initialize the devices and objects used during the simulation
	const string wcFile = "./../Kr16WallWorkCell/Scene.wc.xml";
	const string deviceName = "KukaKr16";
	const string TCPName = "ToolMount";
	const string objectName = "Bottle";
	WorkCell::Ptr wc = WorkCellLoader::Factory::load(wcFile);
	Device::Ptr device = wc->findDevice(deviceName);
	Frame* object = wc->findFrame(objectName);
	Frame* TCP = wc->findFrame(TCPName);
	if (device == NULL) {
		cerr << "Device: " << deviceName << " not found!" << endl;
		return 0;
	}

	// Get the state
	State state = wc->getDefaultState();

	// Set the start and goal configurations
	Q from(6,-3.142,-0.827,-3.002,-3.143,0.099,-1.573);
	Q to(6,1.571,0.006,0.030,0.153,0.762,4.490);

	// Move Robot to start position
	device->setQ(from, state);

	// Grip the object (bottle)
	Kinematics::gripFrame(object, TCP, state);

	// Initialize collision detector
	CollisionDetector detector(wc, ProximityStrategyFactory::makeDefaultCollisionStrategy());
	PlannerConstraint constraint = PlannerConstraint::make(&detector,device,state);

	// More complex way: allows more detailed definition of parameters and methods
	QSampler::Ptr sampler = QSampler::makeConstrained(QSampler::makeUniform(device),constraint.getQConstraintPtr());
	QMetric::Ptr metric = MetricFactory::makeEuclidean<Q>();

	// Initialze the analysers
	PathAnalyzer analysis(device, state);
	PathAnalyzer::CartesianAnalysis result_cartesian;
	PathAnalyzer::JointSpaceAnalysis result_config;
	QMetric::Ptr metric_config = MetricFactory::makeManhattan<Q>();

	// Create paths to hold the generated paths
	QPath path, shortest_path;
	float shortest_length = 999;
	// Initialize the timer
	Timer t;
	// Initialize the file to hold the test data
	ofstream test_file;

	// Start the planner test with different epsilons.
	cout << "Planning from " << from << " to " << to << endl;
	for (float epsi = 0.05; epsi <= 0.06; epsi+=0.05) {
		// Make new planner with the new epsilon
		QToQPlanner::Ptr planner = RRTPlanner::makeQToQPlanner(constraint, sampler, metric, epsi, RRTPlanner::RRTConnect);
		cout << "\nTest epsilon: " << epsi << endl;
		test_file.open("./../data/"+to_string(epsi)+"_eps.txt");
		test_file << "length\tconfig\ttime\n";

		// Run the test 100 times for the given epsilon (num of samples)
		for (int j = 1; j <= 3; j++)
		{
			cout << j << endl;

			// Time the planenr
			t.resetAndResume();
			planner->query(from,to,path,MAXTIME);
			t.pause();

			// Extract the results for the path lengths
			result_cartesian = analysis.analyzeCartesian(path, TCP);
			result_config = analysis.analyzeJointSpace(path, metric_config);

			// If the time is over the allowed time, then write NaN in the test file.
			// If the path is in collision, then write NaN in the test file.
			// Else write the actual results to the test file.
			if (t.getTimeMs() >= MAXTIME) {
				test_file << "NaN" << "\t" << "NaN" << "\t" << t.getTimeMs() << "\n";
			}
			else
			{
				test_file << result_cartesian.length << "\t" << result_config.length << "\t" << t.getTimeMs() << "\n";
				if (result_cartesian.length > 0 && result_cartesian.length < shortest_length) {
					shortest_length = result_cartesian.length;
					shortest_path = path;
				}
			}
			// Clear the path for the next test.
			path.clear();
		}
		// Close the test file and end the tests
		test_file.close();
	}

	// Tell the user about the shortest path
	cout << "\n\nShortest path was of node-length " << shortest_path.size() << endl;
	result_cartesian = analysis.analyzeCartesian(shortest_path, TCP);
	cout << "The TCP length was " << result_cartesian.length << endl;
	cout << "This path is written to the luafile!" << endl;

	// Make lua file from the shortest path so that it can be visualized
	makeLuaFile(shortest_path);
	cout << "Program done." << endl;

	return 0;
}
