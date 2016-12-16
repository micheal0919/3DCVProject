#include "opencv\cxcore.h"

#include "MarkerWorld.h"
#include <fstream>
#include <iostream>

using namespace std;

MarkerWorld::MarkerWorld(void)
{
}

MarkerWorld::~MarkerWorld(void)
{
}

void MarkerWorld::updateMarkerPointerList()
{
    _markerPointerList.clear();
    for(iterator itRigidBody = begin();
        itRigidBody !=end();
        itRigidBody++)
    {
        for(MarkerRigidBody::iterator itMarker=itRigidBody->begin();
            itMarker!=itRigidBody->end();
            itMarker++)
        {
            _markerPointerList.push_back(&(*itMarker));
        }
    }
}

MarkerWorld& MarkerWorld::operator =(const MarkerWorld&source)
{
    std::vector<MarkerRigidBody>::operator=(source);
    _name = source._name;

    updateMarkerPointerList();

    return *this;

}

MarkerWorld::MarkerWorld(const MarkerWorld&source)
	: std::vector<MarkerRigidBody>()
{
    *this = source;

}

bool MarkerWorld::read( std::ifstream& world_stream )
{
    
    
    std::string line;
    size_t pos;
    size_t pos2;

    while((pos = line.find("<MarkerWorld"))==-1 && !world_stream.eof())
    {
        getline(world_stream, line);
    }
    if(world_stream.eof())
    {
        return false;
    }
    while((pos = line.find("id=\"name\">"))==-1 && !world_stream.eof())
    {
        getline(world_stream, line);
    }
    if(world_stream.eof())
    {
        return false;
    }
    pos+=10; //10 is the size of id=\"name\">
    size_t pos_end = line.find("<",pos);
    std::string name = line.substr(pos, pos_end-pos);
    _name = name;

    while((pos = line.find("id=\"RigidBodyList\">"))==-1  && !world_stream.eof())
    {
        getline(world_stream, line);
    }
    if(world_stream.eof())
    {
        return false;
    }
    

    //look for all MarkerRigidBodies
    while(!world_stream.eof())
    {
        getline(world_stream, line);
        pos2=-1;
        while((pos = line.find("<MarkerRigidBody>"))==-1 && (pos2=line.find("</MarkerWorld>"))==-1 && !world_stream.eof())
        {
            getline(world_stream, line);
        }
        if(world_stream.eof())
        {
            return false;
        }
        if(pos2!=-1) break;
        
        MarkerRigidBody mrb;
        mrb.read(world_stream);
        push_back(mrb);

    }
    
    
    

    updateMarkerPointerList();

    return true;

}
