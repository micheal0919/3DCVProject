#pragma once

class MarkerSampling
{
public:
    MarkerSampling(void);
    ~MarkerSampling(void);

    MarkerSampling( const MarkerSampling& source );
    MarkerSampling& operator=( const MarkerSampling& source );

    void resizeCodeBuffer( unsigned int size );
    double * getCodeBufferRing1() const;
    double * getCodeBufferRing2() const;
    int getNumSamples() const;

   
private:
    unsigned int _codeNumSamples;  //< number of samples read in a ring
    double *     _codeBufferRing1; //< code as read in the first (internal) code ring
    double *     _codeBufferRing2; //< code as read in the second (external) code ring
};
