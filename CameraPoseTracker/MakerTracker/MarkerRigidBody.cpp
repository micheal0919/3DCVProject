#include "MarkerRigidBody.h"
#include <fstream>
#include <iostream>



MarkerRigidBody::MarkerRigidBody(void):
_name()
{
}

MarkerRigidBody::~MarkerRigidBody(void)
{
}


void MarkerRigidBody::updateMarkerPlaneList()
{
    _markerPlaneList.clear();

    for(iterator itMarker=begin();
        itMarker!=end();
        itMarker++)
    {
        //compute the plane to which the marker belongs
        bool foundPlanarGroup = false;
        for(vector<MarkerPlane>::iterator itPlane = _markerPlaneList.begin();
            itPlane != _markerPlaneList.end() && foundPlanarGroup!=true;
            itPlane++)
        {

            if(itPlane->contains(&(*itMarker)))
            {
                itPlane->push_back(&(*itMarker));
                foundPlanarGroup = true;
            }
        }
        if(!foundPlanarGroup)
        {
            MarkerPlane novelPlane;
            novelPlane.addAsFirstOccurence(&(*itMarker));
            _markerPlaneList.push_back(novelPlane);
        }
    }
}

bool MarkerPlane::contains( MarkerModel * markerPtr ) const
{
    VectorCv3 mn = markerPtr->getNormal();
    //normalize
    double mnx = cvmGet(mn.cvMatPtr(), 0,0);
    double mny = cvmGet(mn.cvMatPtr(), 1,0);
    double mnz = cvmGet(mn.cvMatPtr(), 2,0);
    double norm = sqrt(mnx*mnx+mny*mny+mnz*mnz);
    if(norm>0 && norm!=1.0)
    {
        mnx/=norm;
        mny/=norm;
        mnz/=norm;
    }


    //up vector has to be the same
    if(mnx!=cvmGet(_normal.cvMatPtr(), 0,0) || mny!=cvmGet(_normal.cvMatPtr(), 1,0) || mnz!=cvmGet(_normal.cvMatPtr(), 2,0))
    {
        return false;
    }

    //same up vector. check distance
    double dist = mnx*cvmGet(markerPtr->getCenter().cvMatPtr(),0,0) + mny*cvmGet(markerPtr->getCenter().cvMatPtr(),1,0) + mnz*cvmGet(markerPtr->getCenter().cvMatPtr(),1,0);

    //distance has to be the same
    if(dist!=_dist)
    {
        return false;
    }

    return true;

}

void MarkerPlane::addAsFirstOccurence( MarkerModel * markerPtr )
{
    clear();

    VectorCv3 mn = markerPtr->getNormal();
    VectorCv3 mu = markerPtr->getOrigin();

    
    double mnx = cvmGet(mn.cvMatPtr(), 0,0);
    double mny = cvmGet(mn.cvMatPtr(), 1,0);
    double mnz = cvmGet(mn.cvMatPtr(), 2,0);
    double mux = cvmGet(mu.cvMatPtr(), 0,0);
    double muy = cvmGet(mu.cvMatPtr(), 1,0);
    double muz = cvmGet(mu.cvMatPtr(), 2,0);

    //normalize n
    double normN=sqrt(mnx*mnx+mny*mny+mnz*mnz);
    cvmSet(_normal.cvMatPtr(), 0, 0, mnx/normN);
    cvmSet(_normal.cvMatPtr(), 1, 0, mny/normN);
    cvmSet(_normal.cvMatPtr(), 2, 0, mnz/normN);

    //normalize u
    double normU = sqrt(mux*mux+muy*muy+muz*muz);
    
    cvmSet(_u.cvMatPtr(), 0, 0, mux/normU);
    cvmSet(_u.cvMatPtr(), 1, 0, muy/normU);
    cvmSet(_u.cvMatPtr(), 2, 0, muz/normU);

    //compute normalized v vector (v = -u *^* n)
    cvmSet(_v.cvMatPtr(), 0,0, cvmGet(_normal.cvMatPtr(), 1,0)*cvmGet(_u.cvMatPtr(), 2,0)-cvmGet(_normal.cvMatPtr(), 2,0)*cvmGet(_u.cvMatPtr(), 1,0));
    cvmSet(_v.cvMatPtr(), 1,0, cvmGet(_normal.cvMatPtr(), 2,0)*cvmGet(_u.cvMatPtr(), 0,0)-cvmGet(_normal.cvMatPtr(), 0,0)*cvmGet(_u.cvMatPtr(), 2,0));
    cvmSet(_v.cvMatPtr(), 2,0, cvmGet(_normal.cvMatPtr(), 0,0)*cvmGet(_u.cvMatPtr(), 1,0)-cvmGet(_normal.cvMatPtr(), 1,0)*cvmGet(_u.cvMatPtr(), 0,0));
    
    //get distance
    _dist = cvmGet(_normal.cvMatPtr(), 0,0)*cvmGet(markerPtr->getCenter().cvMatPtr(),0,0) + cvmGet(_normal.cvMatPtr(), 1,0)*cvmGet(markerPtr->getCenter().cvMatPtr(),1,0) + cvmGet(_normal.cvMatPtr(), 2,0)*cvmGet(markerPtr->getCenter().cvMatPtr(),2,0);

    push_back(markerPtr);

    


}

MarkerRigidBody& MarkerRigidBody::operator =(const MarkerRigidBody&source)
{
    std::vector<MarkerModel>::operator=(source);
    _name = source._name;

    updateMarkerPlaneList();

    return *this;

}

MarkerRigidBody::MarkerRigidBody(const MarkerRigidBody&source):
	std::vector<MarkerModel>()
{
    *this = source;

}

bool MarkerRigidBody::read( std::ifstream& world_stream )
{


    std::string line;
    size_t pos;
    size_t pos2;
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
    while((pos = line.find("id=\"MarkerList\">"))==-1 && !world_stream.eof())
    {
        getline(world_stream, line);
    }
    if(world_stream.eof())
    {
        return false;
    }

    //look for all MarkerModels
    while(!world_stream.eof())
    {
        getline(world_stream, line);
        pos2=-1;
        while((pos = line.find("<MarkerModel>"))==-1 && (pos2=line.find("</MarkerRigidBody>"))==-1 && !world_stream.eof())
        {
            getline(world_stream, line);
        }
        if(world_stream.eof())
        {
            return false;
        }
        if(pos2!=-1) break;

        MarkerModel mm;
        mm.read(world_stream);
        push_back(mm);

    }

    updateMarkerPlaneList();



    return true;
   
}
