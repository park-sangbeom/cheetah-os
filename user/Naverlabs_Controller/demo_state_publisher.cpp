/*!
 * @file main.cpp
 *
 * The main function parses command line arguments and starts the appropriate
 * driver.
 */

#include <main_helper.h>
#include "Lowlevel_Controller.hpp"

int main(int argc, char** argv) {
    (void)argc;
    (void)argv;
    Lowlevel_Controller lowlevel_controller; 
    lowlevel_controller.pub_sub_lcm();
    return 0;
}