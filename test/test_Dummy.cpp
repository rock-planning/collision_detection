#include <boost/test/unit_test.hpp>
#include <collision_detection/Dummy.hpp>

using namespace collision_detection;

BOOST_AUTO_TEST_CASE(it_should_not_crash_when_welcome_is_called)
{
    collision_detection::DummyClass dummy;
    dummy.welcome();
}
