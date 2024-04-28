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

// ----------------------------------------------------------------------------------------------

/*
void IFX_EMA_Init(IFX_EMA *filt, double alpha){

    IFX_EMA_SetAlpha(filt, alpha);

    filt->out = 0.0f;
}

void IFX_EMA_SetAlpha(IFX_EMA *filt, double alpha){

    if(alpha < 0.0f){
        filt->alpha = 0.0f;
    }
    else if(alpha > 1.0f){
        filt->alpha = 1.0f;
    }
    
    filt->alpha = alpha;   
}

double IFX_EMA_Update(IFX_EMA *filt, double in){
    filt->out = filt->alpha * in + (1.0f - filt->alpha)*filt->out;
    return filt->out;
}
*/



//-----------------------------------------------------------------------------------------------

// cutoff freq: 12
// transition bandwidth: 6
// window type: hamming

/*
static double FIR_IMPULSE_RESPONSE[FIR_FILTER_LENGTH] = {
0.000333853857448447f,
-0.00104609626230262f,
-0.00193297660067591f,
-0.00099355127692384f,
0.002211318039028508f,
0.005192399426243866f,
0.003407568696318785f,
-0.004546772395009404f,
-0.012354273975931732f,
-0.009407970992883001f,
0.007537578639226952f,
0.025560691018175354f,
0.022598172731741546f,
-0.010460981569823067f,
-0.051642602293181204f,
-0.055458993359458156f,
0.012581588402430482f,
0.140648759020065295f,
0.267509851850688374f,
0.320524874089642586f,
0.267509851850688374f,
0.140648759020065295f,
0.012581588402430486f,
-0.055458993359458177f,
-0.051642602293181204f,
-0.010460981569823067f,
0.022598172731741542f,
0.025560691018175357f,
0.007537578639226955f,
-0.009407970992883001f,
-0.012354273975931739f,
-0.004546772395009408f,
0.003407568696318787f,
0.005192399426243865f,
0.002211318039028506f,
-0.000993551276923849f,
-0.001932976600675912f,
-0.001046096262302623f,
0.000333853857448447f,
}; 

void FIRFilter_Init(FIRFilter *fir){

    // Clear input buffer
    for(uint8_t n = 0; n < FIR_FILTER_LENGTH; ++n){
        fir->buf[n] = 0.0f;
    }

    // Reset buffer index
    fir->bufIndex = 0;

    // Clear filter output
    fir->out = 0.0f;
}


double FIRFilter_Update(FIRFilter *fir, double inp){

    // Store latest sample in buffer

    fir->buf[fir->bufIndex] = inp;

    // Increment buffer index and wrap around if necessary

    fir->bufIndex++;

    if(fir->bufIndex == FIR_FILTER_LENGTH){
        fir->bufIndex = 0;
    }

    // Compute new output sample (via Convolution)

    fir->out = 0.0f;

    uint8_t sumIndex = fir->bufIndex;

    for(uint8_t n = 0; n < FIR_FILTER_LENGTH; ++n){

        // Decrement index and wrap if necessary 

        if(sumIndex>0){
            sumIndex--;
        }
        else{
            sumIndex = FIR_FILTER_LENGTH - 1;
        }

        // Multiply impulse response with shifted input sample and add to output

        fir->out += FIR_IMPULSE_RESPONSE[n] * fir->buf[sumIndex];

    }

    // Return filtered output

    return fir->out;
}
*/




// 

double movingAverage(const std::vector<double>& data, int windowSize){
    double sum = 0.0;
    for(int i = 0; i < windowSize; ++i){
        sum += data[i];
    }
    return sum/windowSize;
}




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

int ActiveFilter::getCompletedDataSize()
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
