//********************************
// MarkerTracker
// (c) 2010 DFKI / AV
//********************************
#pragma once

#if defined(WIN32)
#	if defined(pulsar_marker_EXPORTS)  && defined(_WINDLL)
#		define MARKERTRACKER_EXPORTED __declspec(dllexport)
#	else
#		define MARKERTRACKER_EXPORTED __declspec(dllimport)
#	endif
#else
#	define MARKERTRACKER_EXPORTED
#endif