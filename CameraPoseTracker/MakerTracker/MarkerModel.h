#pragma once
#include "opencv\cxcore.h"
#include "MatrixCv.h"


/* Represents analog and digital markers. They are uniquely
represented by _model or _ID. These values are semantically 
the same, yet in a different format - _model holds the binary 
digits of the id for quick access.
The representation in _model is used in case of analog-, the 
representation in _ID in case of digital markers. */

class MarkerModel 
{
public:
    MarkerModel(void);
    ~MarkerModel(void);

    MarkerModel(const MarkerModel& source);
    MarkerModel& operator =(const MarkerModel&source);

    VectorCv3 getCenter() const {return _center;}
    VectorCv3 getNormal() const {return _normal;}
    VectorCv3 getOrigin() const {return _origin;}
    double getRadius() const {return _radius;}
    void setCenter(const VectorCv3& center){_center = center;}
    void setNormal(const VectorCv3& normal){_normal = normal;}
    void setOrigin(const VectorCv3& origin){_origin = origin;}
    void setRadius(double radius) {_radius = radius;}

    const int* getModel() const {return _model;}
	unsigned int getID() const { return _ID;}
    std::string getName() const {return _name;}
	
    bool read(std::ifstream& world_stream );

private:
	int _model[16];
	unsigned int _ID;
    std::string _name;
    double _radius;
    VectorCv3 _center;
    VectorCv3 _normal;
    VectorCv3 _origin;

	void initID();

};
