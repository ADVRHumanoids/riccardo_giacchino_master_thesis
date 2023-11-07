#include <cartesianimpedancecontroller.h>

int main (int argc, char **argv)
{

    ros::init(argc, argv, "cart_imp_controller_node");
    ros::NodeHandle nh ("xbotcore");

    const double dt = 0.01; // sampling time

    CartesianImpedanceController controller_front_left_leg(nh, dt);

    while (true) {

        controller_front_left_leg.update_inertia();
    }

    return 0;
}
