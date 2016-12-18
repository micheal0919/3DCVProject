#ifndef __RECONSTRUCTION_IO_H__
#define __RECONSTRUCTION_IO_H__

#include <string>

#include "Reconstruction.h"

bool WriteReconstruction(const CReconstruction& reconstruction,
	const std::string& output_file);

#endif // __RECONSTRUCTION_IO_H__
