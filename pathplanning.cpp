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

#define MAXTIME 120.

void makeLuaFile(QPath &path)
{
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
	rw::math::Math::seed();
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
	State state = wc->getDefaultState();

	// Grip the object (bottle)
	Kinematics::gripFrame(object, TCP, state);

	CollisionDetector detector(wc, ProximityStrategyFactory::makeDefaultCollisionStrategy());
	PlannerConstraint constraint = PlannerConstraint::make(&detector,device,state);

	/** More complex way: allows more detailed definition of parameters and methods */
	QSampler::Ptr sampler = QSampler::makeConstrained(QSampler::makeUniform(device),constraint.getQConstraintPtr());
	QMetric::Ptr metric = MetricFactory::makeEuclidean<Q>();

	// Initialze the analyser
	PathAnalyzer analysis(device, state);
	PathAnalyzer::CartesianAnalysis result_cartesian;
	PathAnalyzer::JointSpaceAnalysis result_config;
	QMetric::Ptr metric_config = MetricFactory::makeManhattan<Q>();

	Q from(6,-3.142,-0.827,-3.002,-3.143,0.099,-1.573);
	Q to(6,1.571,0.006,0.030,0.153,0.762,4.490);

	if (!checkCollisions(device, state, detector, from))
		return 0;
	if (!checkCollisions(device, state, detector, to))
		return 0;

	cout << "Planning from " << from << " to " << to << endl;
	QPath path, shortest_path;
	Timer t;
	float shortest_length = 999;
	ofstream test_file;

	for (float i = 0.05; i <= 1; i+=0.05) {
		QToQPlanner::Ptr planner = RRTPlanner::makeQToQPlanner(constraint, sampler, metric, i, RRTPlanner::RRTConnect);
		cout << "\nTest epsilon: " << i << endl;
		test_file.open("./../data/"+to_string(i)+"_eps.txt");
		test_file << "length\tconfig\ttime\n";

		for (int j = 1; j <= 100; j++)
		{
			cout << j << endl;
			t.resetAndResume();
			planner->query(from,to,path,MAXTIME);
			t.pause();
			result_cartesian = analysis.analyzeCartesian(path, TCP);
			result_config = analysis.analyzeJointSpace(path, metric_config);
			if (t.getTime() >= MAXTIME) {
				test_file << "NaN" << "\t" << "NaN" << "\t" << t.getTime() << "\n";
			}
			else
			{
				test_file << result_cartesian.length << "\t" << result_config.length << "\t" << t.getTime() << "\n";
				if (result_cartesian.length > 0 && result_cartesian.length < shortest_length) {
					shortest_length = result_cartesian.length;
					shortest_path = path;
				}
			}
			path.clear();
		}
		test_file.close();
	}

	cout << "\n\nShortest path was of node-length " << shortest_path.size() << endl;
	result_cartesian = analysis.analyzeCartesian(shortest_path, TCP);
	cout << "The TCP length was " << result_cartesian.length << endl;
	cout << "This path is written to the luafile!" << endl;
	makeLuaFile(shortest_path);
	cout << "Program done." << endl;

	return 0;
}
