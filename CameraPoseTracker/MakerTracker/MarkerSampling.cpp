#include "MarkerSampling.h"
#include <cstring>

MarkerSampling::MarkerSampling(void):
_codeNumSamples(0),
_codeBufferRing1(0),
_codeBufferRing2(0)
{
}

MarkerSampling::~MarkerSampling(void)
{
    if(_codeBufferRing1) delete[] _codeBufferRing1;
    if(_codeBufferRing2) delete[] _codeBufferRing2;

}

MarkerSampling::MarkerSampling( const MarkerSampling& source )
	
{
    _codeNumSamples = source._codeNumSamples;
    _codeBufferRing1 = 0;
    _codeBufferRing2 = 0;

    if(_codeNumSamples!=0)
    {
        _codeBufferRing1 = new double[_codeNumSamples];
        _codeBufferRing2 = new double[_codeNumSamples];
        memcpy(_codeBufferRing1, source._codeBufferRing1, _codeNumSamples*sizeof(double));
        memcpy(_codeBufferRing2, source._codeBufferRing2, _codeNumSamples*sizeof(double));
    }

}

MarkerSampling& MarkerSampling::operator=( const MarkerSampling& source )
{
    if (this != &source)
    {
        _codeNumSamples = source._codeNumSamples;
        if(_codeBufferRing1) delete[] _codeBufferRing1;
        if(_codeBufferRing2) delete[] _codeBufferRing2;

        _codeBufferRing1 = 0;
        _codeBufferRing2 = 0;
        if(_codeNumSamples!=0)
        {
            _codeBufferRing1 = new double[_codeNumSamples];
            _codeBufferRing2 = new double[_codeNumSamples];
            memcpy(_codeBufferRing1, source._codeBufferRing1, _codeNumSamples*sizeof(double));
            memcpy(_codeBufferRing2, source._codeBufferRing2, _codeNumSamples*sizeof(double));
        }
    }
    return *this;
}

void MarkerSampling::resizeCodeBuffer( unsigned int size )
{
    if(_codeBufferRing1) delete[] _codeBufferRing1;
    if(_codeBufferRing2) delete[] _codeBufferRing2;

    _codeBufferRing1 = 0;
    _codeBufferRing2 = 0;
    _codeNumSamples = size;
    if(_codeNumSamples!=0)
    {
        _codeBufferRing1 = new double[_codeNumSamples];
        _codeBufferRing2 = new double[_codeNumSamples];
        memset(_codeBufferRing1, 0, _codeNumSamples*sizeof(double));
        memset(_codeBufferRing2, 0, _codeNumSamples*sizeof(double));
    }
}

double * MarkerSampling::getCodeBufferRing1() const
{
    return _codeBufferRing1;
}

double * MarkerSampling::getCodeBufferRing2() const
{
    return _codeBufferRing2;
}

int MarkerSampling::getNumSamples() const
{
    return _codeNumSamples;
}
