#include "MarkerModel.h"
#include <cstring>
#include <fstream>
#include <iostream>

using namespace std;

MarkerModel::MarkerModel(void)
{
    memset(_model, 0, 16*sizeof(int));
}

MarkerModel::MarkerModel( const MarkerModel& source )
{
    *this = source;
}


MarkerModel& MarkerModel::operator=( const MarkerModel&source )
{
    memcpy(_model, source._model, 16*sizeof(int));
    setCenter(source.getCenter());
    setNormal(source.getNormal());
    setOrigin(source.getOrigin());
    _name = source._name;
    _radius = source._radius;
    _ID = source._ID;

    return (*this);

}

MarkerModel::~MarkerModel(void)
{
}


bool MarkerModel::read( std::ifstream& world_stream )
{


    std::string line;
    size_t pos;
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

    

    size_t pos_r, pos_cx, pos_cy, pos_cz, pos_nx, pos_ny, pos_nz, pos_ux, pos_uy, pos_uz, pos_f, pos_ir, pos_or;
    pos_end = line.find("</MarkerModel>");
    pos_r = line.find("\"Radius\">");
    pos_cx = line.find("\"cx\">");
    pos_cy = line.find("\"cy\">");
    pos_cz = line.find("\"cz\">");
    pos_nx = line.find("\"nx\">");
    pos_ny = line.find("\"ny\">");
    pos_nz = line.find("\"nz\">");
    pos_ux = line.find("\"ux\">");
    pos_uy = line.find("\"uy\">");
    pos_uz = line.find("\"uz\">");
    pos_ir = line.find("\"innerRing\">");
    pos_or = line.find("\"outerRing\">");
    string innerCode, outerCode;
    double curval;
    while(pos_end==-1 && pos_r==-1 && pos_cx==-1 && pos_cy==-1 && pos_cz==-1 && pos_nx==-1 && pos_ny==-1 && pos_nz==-1 && pos_ux==-1 && pos_uy==-1 && pos_uz==-1 && pos_ir==-1 && pos_or==-1 && !world_stream.eof())
    {
        getline(world_stream, line);
        pos_end = line.find("</MarkerModel>");
        pos_r = line.find("\"Radius\">");
        pos_cx = line.find("\"cx\">");
        pos_cy = line.find("\"cy\">");
        pos_cz = line.find("\"cz\">");
        pos_nx = line.find("\"nx\">");
        pos_ny = line.find("\"ny\">");
        pos_nz = line.find("\"nz\">");
        pos_ux = line.find("\"ux\">");
        pos_uy = line.find("\"uy\">");
        pos_uz = line.find("\"uz\">");
        pos_ir = line.find("\"innerRing\">");
        pos_or = line.find("\"outerRing\">");


        if(world_stream.eof())
        {
            return false;
        }
        if(pos_end!=-1) break;

        if(pos_r!=-1)
        {
            pos=pos_r+9;
            pos_f = line.find("<",pos);
            std::string val_str = line.substr(pos, pos_f-pos);
            stringstream ss(val_str);
            ss >> _radius;
            pos_r=-1;
        }
        if(pos_cx!=-1)
        {
            pos=pos_cx+5;
            pos_f = line.find("<",pos);
            std::string val_str = line.substr(pos, pos_f-pos);
            stringstream ss(val_str);
            ss >> curval;
            cvmSet(_center.cvMatPtr(),0,0,curval);
            pos_cx=-1;
        }
        if(pos_cy!=-1)
        {
            pos=pos_cy+5;
            pos_f = line.find("<",pos);
            std::string val_str = line.substr(pos, pos_f-pos);
            stringstream ss(val_str);
            ss >> curval;
            cvmSet(_center.cvMatPtr(),1,0,curval);
            pos_cy=-1;
        }
        if(pos_cz!=-1)
        {
            pos=pos_cz+5;
            pos_f = line.find("<",pos);
            std::string val_str = line.substr(pos, pos_f-pos);
            stringstream ss(val_str);
            ss >> curval;
            cvmSet(_center.cvMatPtr(),2,0,curval);
            pos_cz=-1;
        }
        if(pos_nx!=-1)
        {
            pos=pos_nx+5;
            pos_f = line.find("<",pos);
            std::string val_str = line.substr(pos, pos_f-pos);
            stringstream ss(val_str);
            ss >> curval;
            cvmSet(_normal.cvMatPtr(),0,0,curval);
            pos_nx=-1;
        }
        if(pos_ny!=-1)
        {
            pos=pos_ny+5;
            pos_f = line.find("<",pos);
            std::string val_str = line.substr(pos, pos_f-pos);
            stringstream ss(val_str);
            ss >> curval;
            cvmSet(_normal.cvMatPtr(),1,0,curval);
            pos_ny=-1;
        }
        if(pos_nz!=-1)
        {
            pos=pos_nz+5;
            pos_f = line.find("<",pos);
            std::string val_str = line.substr(pos, pos_f-pos);
            stringstream ss(val_str);
            ss >> curval;
            cvmSet(_normal.cvMatPtr(),2,0,curval);
            pos_nz=-1;
        }
        if(pos_ux!=-1)
        {
            pos=pos_ux+5;
            pos_f = line.find("<",pos);
            std::string val_str = line.substr(pos, pos_f-pos);
            stringstream ss(val_str);
            ss >> curval;
            cvmSet(_origin.cvMatPtr(),0,0,curval);
            pos_ux=-1;
        }
        if(pos_uy!=-1)
        {
            pos=pos_uy+5;
            pos_f = line.find("<",pos);
            std::string val_str = line.substr(pos, pos_f-pos);
            stringstream ss(val_str);
            ss >> curval;
            cvmSet(_origin.cvMatPtr(),1,0,curval);
            pos_uy=-1;
        }
        if(pos_uz!=-1)
        {
            pos=pos_uz+5;
            pos_f = line.find("<",pos);
            std::string val_str = line.substr(pos, pos_f-pos);
            stringstream ss(val_str);
            ss >> curval;
            cvmSet(_origin.cvMatPtr(),2,0,curval);
            pos_uz=-1;
        }
        if(pos_ir!=-1)
        {
            pos=pos_ir+12;
            pos_f = line.find("<",pos);
            innerCode = line.substr(pos, pos_f-pos);
            pos_ir=-1;
        }
        if(pos_or!=-1)
        {
            pos=pos_or+12;
            pos_f = line.find("<",pos);
            outerCode = line.substr(pos, pos_f-pos);
            pos_or=-1;
        }
    }

    string fullcode = innerCode+outerCode;
    string::iterator it = fullcode.begin();
    for (int i=0; i<16; i++)
    {
        _model[i] = (*it++=='1'?1:0);
    }

    initID();
    

}

void MarkerModel::initID()
{
	for(unsigned int i = 0; i < 16; i++)
	{
		_ID <<= 1;
		if(_model[i] > 0)
			_ID++;
	}
}
