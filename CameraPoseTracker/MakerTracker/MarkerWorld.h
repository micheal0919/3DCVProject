#pragma once
#include "MarkerRigidBody.h"
#include <vector>
#include <fstream>


class MarkerWorld :
    public std::vector<MarkerRigidBody>
{
public:
    MarkerWorld(void);
    ~MarkerWorld(void);

    MarkerWorld(const MarkerWorld& source);
    MarkerWorld& operator =(const MarkerWorld&source);

    bool read(std::ifstream& world_stream );

    void updateMarkerPointerList();

    typedef std::vector<MarkerModel*>::iterator  MarkerPtrIterator;
    typedef std::vector<MarkerModel*>::const_iterator const_MarkerPtrIterator;
    MarkerPtrIterator markerBegin(){return _markerPointerList.begin();}
    MarkerPtrIterator markerEnd(){return _markerPointerList.end();}
    const_MarkerPtrIterator markerBegin() const{return _markerPointerList.begin();}
    const_MarkerPtrIterator markerEnd() const{return _markerPointerList.end();}

    
private:
    std::string               _name;
    std::vector<MarkerModel*> _markerPointerList;
     
};
