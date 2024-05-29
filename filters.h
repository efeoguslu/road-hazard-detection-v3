#ifndef FILTERS_H
#define FILTERS_H

#include <cstdint>
#include <vector>
#include <deque>
#include <iostream>
#include <fstream>

typedef struct{
    double alpha;
    double out;
} FirstOrderIIR;

typedef struct {
    FirstOrderIIR x;
    FirstOrderIIR y;
    FirstOrderIIR z;
} ThreeAxisIIR;

void FirstOrderIIR_Init(FirstOrderIIR *filt, double alpha);
double FirstOrderIIR_Update(FirstOrderIIR *filt, double in);

void ThreeAxisIIR_Init(ThreeAxisIIR *filt, double alpha);
void ThreeAxisIIR_Update(ThreeAxisIIR *filt, double in_x, double in_y, double in_z, double *out_x, double *out_y, double *out_z);

// -------------------------------------------------------------------------------------------------------------------------

class ActiveFilter
{    
private:
    /* data */
    int m_windowSize;
    int m_overlapSize;
    int m_diffSize;

    // https://stackoverflow.com/questions/5563952/why-there-is-no-pop-front-method-in-c-stdvector
    // http://adrinael.net/containerchoice 

    double m_posCoef     = 1.6; // was 1.4 before, but 1.2 was written in Python analysis
    double m_negCoef     = 0.1;
    double m_threshold   = 0.2; // was 0.15 before, 0.5 is more suitable after analysis
    double m_offset      = 1.0;      //signal offset
    bool  m_dataInit    = false;
    std::deque<double> m_data;  //size: windowSize
    std::deque<double> m_newData; // size: windowSize-overlapSize
    std::deque<double> m_completedData; // completed data , no more operations will be done on there

    // |_______||__overlap___||__newdata__|
    // |---------window-------|
    //          |---------new-window------|
    
public:
    ActiveFilter(/* args */);
    ~ActiveFilter();
    void                setWindowParameters(int windowSize, int overlapSize);
    void                setThreshold(double threshold);
    void                setOffset(double offset);
    void                feedData(double data);
    long unsigned int   getCompletedDataSize();
    std::deque<double>  getCompletedData();     // maybe return can be changed as vector / queue 
    int                getDiffSize();

private:
    void                initMembers();
    void                doCalculation();
    double              getMaxValue();
    double              getMinValue();
};

#endif // FILTERS_H