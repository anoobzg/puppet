#ifndef STATE_DECLARE
#define STATE_DECLARE
#include <osg/State>

#define state_on (osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE | osg::StateAttribute::PROTECTED)
#define state_off (osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE | osg::StateAttribute::PROTECTED)

#endif // STATE_DECLARE