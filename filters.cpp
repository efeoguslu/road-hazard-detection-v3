#include "filters.h"

void FirstOrderIIR_Init(FirstOrderIIR *filt, double alpha){

    if(alpha < 0.0f){
        filt->alpha = 0.0f;
    }
    else if(alpha > 1.0f){
        filt->alpha = 1.0f;
    }
    else{
        filt->alpha = alpha;
    }

    filt->out = 0.0f;
}

double FirstOrderIIR_Update(FirstOrderIIR *filt, double in){

    filt->out = (1.0f - filt->alpha) * in + filt->alpha * filt->out;
    return filt->out;
}

void ThreeAxisIIR_Init(ThreeAxisIIR *filt, double alpha){
    FirstOrderIIR_Init(&filt->x, alpha);
    FirstOrderIIR_Init(&filt->y, alpha);
    FirstOrderIIR_Init(&filt->z, alpha);
}

void ThreeAxisIIR_Update(ThreeAxisIIR *filt, double in_x, double in_y, double in_z, double *out_x, double *out_y, double *out_z){
    *out_x = FirstOrderIIR_Update(&filt->x, in_x);
    *out_y = FirstOrderIIR_Update(&filt->y, in_y);
    *out_z = FirstOrderIIR_Update(&filt->z, in_z);
}

// ------------------------------------------------------------------------------------------------------------------------------------------------

ActiveFilter::ActiveFilter(/* args */)
{
    this->initMembers();
}

ActiveFilter::~ActiveFilter()
{

}

void ActiveFilter::setWindowParameters(int windowSize, int overlapSize)
{
    this->m_windowSize  = windowSize;
    this->m_overlapSize = overlapSize;
    this->m_diffSize    = this->m_windowSize - this->m_overlapSize;
}

void ActiveFilter::setThreshold(double threshold)
{
    this->m_threshold = threshold;
}

void ActiveFilter::setOffset(double offset = 1.0)
{
    this->m_offset = offset;
}

void ActiveFilter::feedData(double data)
{
    this->m_newData.push_back(data-this->m_offset); //reduce offset value in order to keep signal around 0
    if (this->m_newData.size() >= this->m_diffSize)
    {
        if(this->m_data.size() >= this->m_diffSize)
        {
            //move completed data
            for(int i=0; i< this->m_diffSize; i++)
            {            
                this->m_completedData.push_back(this->m_data.at(0));  
                this->m_data.pop_front();
            }
        }

        //add new data
        for(int i=0; i< this->m_diffSize; i++)
        {
            this->m_data.push_back(this->m_newData.at(0));
            this->m_newData.pop_front();
        }

        this->doCalculation();
    }
}

long unsigned int ActiveFilter::getCompletedDataSize()
{
    return this->m_completedData.size();
}

std::deque<double> ActiveFilter::getCompletedData()
{    
    // temporary copy for return
    std::deque<double> retData(this->m_completedData);
    //clear completed data
    this->m_completedData.clear();
    return retData;
}


void ActiveFilter::initMembers()
{
    this->m_data.clear();
    this->m_completedData.clear();
    this->m_newData.clear();

    this->m_data.resize(0);
    this->m_completedData.resize(0);
    this->m_newData.resize(0);
}

void ActiveFilter::doCalculation()
{
    double minVal = this->getMinValue();
    double maxVal = this->getMaxValue();
    double operationCoef = 1.0;
    
    if( (maxVal-minVal) > this->m_threshold)
    {
        operationCoef = this->m_posCoef;
    }else{
        operationCoef = this->m_negCoef;
    }

    for (int i=0; i<this->m_data.size(); i++)
    {
        this->m_data.at(i) = this->m_data.at(i) * operationCoef;
    }
}

double ActiveFilter::getMaxValue()
{
    double value=this->m_data.at(0);
    for(int i=0; i< this->m_data.size() ;i++ )
    {
        if(value < this->m_data.at(i))
        {
            value = this->m_data.at(i);
        }
    }
    return value;
}   

double ActiveFilter::getMinValue()
{
    double value=this->m_data.at(0);
    for(int i=0; i< this->m_data.size() ;i++ )
    {
        if(value > this->m_data.at(i))
        {
            value = this->m_data.at(i);
        }
    }
    return value;
}
