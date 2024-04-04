#ifndef FILTERS_H
#define FILTERS_H

#include <cstdint>
#include <vector>

#define FIR_FILTER_LENGTH (39)

typedef struct{
    float alpha;
    float out;
} FirstOrderIIR;

void FirstOrderIIR_Init(FirstOrderIIR *filt, float alpha);
float FirstOrderIIR_Update(FirstOrderIIR *filt, float in);


// -------------------------------------------------------------------------------------------------------------------------

typedef struct{
    float alpha;
    float out;
} IFX_EMA;


void IFX_EMA_Init(IFX_EMA *filt, float alpha);
void IFX_EMA_SetAlpha(IFX_EMA *filt, float alpha);
float IFX_EMA_Update(IFX_EMA *filt, float in);

// -------------------------------------------------------------------------------------------------------------------------

typedef struct{
    float buf[FIR_FILTER_LENGTH]; // circular buffer
    uint8_t bufIndex;
    float out;
}FIRFilter;

void FIRFilter_Init(FIRFilter *fir);
float FIRFilter_Update(FIRFilter *fir, float inp);

double movingAverage(const std::vector<double>& data, int windowSize);



#endif // FILTERS_H