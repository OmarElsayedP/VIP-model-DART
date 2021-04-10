#include <dart/dart.hpp>
#include <dart/gui/osg/osg.hpp>
#include <dart/utils/utils.hpp>
#include <dart/utils/urdf/urdf.hpp>
#include <dart/gui/gui.hpp>
#include "HRP4WorldNode.hpp"
#include "HRP4EventHandler.hpp"

int main(int argc, char* argv[])
{
  // Create a world
  dart::simulation::WorldPtr world(new dart::simulation::World);

  // Load ground and HRP4 robot and add them to the world
  dart::utils::DartLoader urdfLoader;
  auto ground = urdfLoader.parseSkeleton(realpath("../ground.urdf", NULL));
  auto hrp4 = urdfLoader.parseSkeleton(realpath("../urdf/hrp4.urdf", NULL));
  world->addSkeleton(ground);
  world->addSkeleton(hrp4);
  
  // set joint actuator type and force limits

  for (int i = 0; i < hrp4->getNumJoints(); i++) {
	  size_t  dim   = hrp4->getJoint(i)->getNumDofs();
	  if(dim==6) {
		  hrp4->getJoint(i)->setActuatorType(dart::dynamics::Joint::PASSIVE);
      // std::cout << "hrp4->getBodyNode(i)->getName"<< hrp4->getBodyNode(i)->getName() << '\n';
	  }
	  if(dim==1) {
		  hrp4->getJoint(i)->setActuatorType(dart::dynamics::Joint::SERVO);
                  //hrp4->getJoint(i)->setActuatorType(dart::dynamics::Joint::ACCELERATION);
		  hrp4->getJoint(i)->setForceUpperLimit(0,  forceLimit);
		  hrp4->getJoint(i)->setForceLowerLimit(0, -forceLimit);
		  hrp4->getJoint(i)->setPositionLimitEnforced(true);
      //Getting inertia
	  }
  }

  // Set gravity of the world
  world->setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  world->setTimeStep(1.0/100.0);

  // std::cout << "main before new world" << '\n';
  // Wrap a WorldNode around it
  //THis is the step where we go into the update() function in Controller
  osg::ref_ptr<HRP4WorldNode> node = new HRP4WorldNode(world, hrp4);
  // std::cout << "main after new world BUT BEFORE numperstepscycle" << '\n';
  node->setNumStepsPerCycle(1);
  // Create a Viewer and set it up with the WorldNode
  dart::gui::osg::ImGuiViewer viewer;
  viewer.addWorldNode(node);

  // Set recording
  // viewer.record("../data","frame");

  // Pass in the custom event handler
  viewer.addEventHandler(new HRP4EventHandler(node));

  // Set the dimensions for the window
  viewer.setUpViewInWindow(0, 0, 1280, 960);

  // Set the window name
  viewer.realize();
  osgViewer::Viewer::Windows windows;
  viewer.getWindows(windows);
  windows.front()->setWindowName("HRP4 MPC");

  // Adjust the viewpoint of the Viewer
int sim = 2;

if( sim <= 1) {
viewer.getCameraManipulator()->setHomePosition(
        ::osg::Vec3d( 0.8,  -8.2*3.28*0.2, 3.3*0.155)*1.0,
        ::osg::Vec3d( -0.10,  2.5, 0.35),
        ::osg::Vec3d( 0.00,  0.2, 2.1));

  viewer.getCameraManipulator()->setHomePosition(
        ::osg::Vec3d( 5.14,  3.28, 6.28)*0.7,
        ::osg::Vec3d( 0.50,  -1.00, 0.00),
        ::osg::Vec3d( 0.00,  0.00, 0.1));
}
if (sim == 2){
  viewer.getCameraManipulator()->setHomePosition(
        ::osg::Vec3d( 5.14,  3.28, 6.28)*0.7,
        ::osg::Vec3d( 0.50,  -1.00, 0.00),
        ::osg::Vec3d( 0.00,  0.00, 0.1));
}

  // We need to re-dirty the CameraManipulator by passing it into the viewer
  // again, so that the viewer knows to update its HomePosition setting
  viewer.setCameraManipulator(viewer.getCameraManipulator());

  // Begin running the application loop
  viewer.run();
}
