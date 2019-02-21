#ifndef THREADS_H_
#define THREADS_H_

#include "../structs.h"
#include "../globals.h"
#include "../HelperFunctions/visualization_functions.h"

// Thread for publishing moving objects (high frequency update)
void VisualizationThread(const double &rate, const std::string &ns);


#endif  // THREADS_H_