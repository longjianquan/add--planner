#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/SimpleSetup.h>

#include <ompl/config.h>
#include <iostream>
#include <fstream>
#include <ostream>
#include "boost/bind.hpp"

using namespace std;

namespace ob = ompl::base;
namespace og = ompl::geometric;

bool isStateValid(const ob::State *state)
{
  // cast the abstract state type to the type we expect
  const ob::SE3StateSpace::StateType *se3state = state->as<ob::SE3StateSpace::StateType>();

  // extract the first component of the state and cast it to what we expect
  const ob::RealVectorStateSpace::StateType *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

  // extract the second component of the state and cast it to what we expect
  const ob::SO3StateSpace::StateType *rot = se3state->as<ob::SO3StateSpace::StateType>(1);

  // check validity of state defined by pos & rot

  // return a value that is always true but uses the two variables we define, so we avoid compiler
  // warnings
  return (const void *)rot != (const void *)pos;
}

void planWithSimpleSetup(void)
{
  // construct the state space we are planning in
  ob::StateSpacePtr space(new ob::SE3StateSpace());

  // set the bounds for the R^3 part of SE(3)
  ob::RealVectorBounds bounds(3);
  bounds.setLow(-1);
  bounds.setHigh(1);

  space->as<ob::SE3StateSpace>()->setBounds(bounds);

  // define a simple setup class
  og::SimpleSetup ss(space);

  // set state validity checking for this space
  ss.setStateValidityChecker(boost::bind(&isStateValid, _1));

  // create a random start state
  ob::ScopedState<> start(space);
  start.random();

  // create a random goal state
  ob::ScopedState<> goal(space);
  goal.random();

  // set the start and goal states
  ss.setStartAndGoalStates(start, goal);

  ob::PlannerPtr planner(new og::RRT(ss.getSpaceInformation()));
  ss.setPlanner(planner);

  // attempt to solve the problem within one second of planning time
  ob::PlannerStatus solved = ss.solve(1.0);

  if (solved) {
    std::cout << "Found solution:" << std::endl;
    // print the path to screen

    std::ofstream ofs0("../plot/path0.dat");
    ss.getSolutionPath().printAsMatrix(ofs0);

    ss.simplifySolution();
    ss.getSolutionPath().print(std::cout);

    std::ofstream ofs("../plot/path.dat");
    ss.getSolutionPath().printAsMatrix(ofs);
  } else
    std::cout << "No solution found" << std::endl;
}

int main(int, char **)
{
  std::cout << "OMPL version: " << OMPL_VERSION << std::endl;
  planWithSimpleSetup();

  return 0;
}
