#pragma once
#include "MarkerModel.h"
#include <vector>
#include "opencv\cxcore.h"
#include "MatrixCv.h"

class MarkerPlane :
    public std::vector<MarkerModel*>
{
public:
    bool contains(MarkerModel * markerPtr)const; //< tells if this plane contains that marker
    void addAsFirstOccurence(MarkerModel * markerPtr);
    VectorCv3 getNormal() const {return _normal;}
    VectorCv3 getU() const {return _u;}
    VectorCv3 getV() const {return _v;}
    void setNormal(VectorCv3 normal){_normal = normal;}
    void setU(VectorCv3 u){_u = u;}
    void setV(VectorCv3 v){_v = v;}
    double getDist() const {return _dist;}
private:
    VectorCv3 _normal; //< unit normal vector
    VectorCv3 _u; //< unit U vector
    VectorCv3 _v; //< unit V vector
    
    double _dist; //< signed distance to world center

};

class MarkerRigidBody :
    public std::vector<MarkerModel>
{
public:
    MarkerRigidBody(void);
    MarkerRigidBody(const MarkerRigidBody& source);
    ~MarkerRigidBody(void);

    MarkerRigidBody& operator =(const MarkerRigidBody&source);

    void updateMarkerPlaneList();

    typedef std::vector<MarkerPlane>::iterator  MarkerPlaneIterator;
    typedef std::vector<MarkerPlane>::const_iterator const_MarkerPlaneIterator;
    MarkerPlaneIterator planeBegin(){return _markerPlaneList.begin();}
    MarkerPlaneIterator planeEnd(){return _markerPlaneList.end();}
    const_MarkerPlaneIterator planeBegin() const{return _markerPlaneList.begin();}
    const_MarkerPlaneIterator planeEnd() const{return _markerPlaneList.end();}

    const std::string& getName() const {return _name;}
    bool read(std::ifstream& world_stream );

private:
    std::string                 _name;
    std::vector<MarkerPlane>    _markerPlaneList;
};
