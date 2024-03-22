#ifndef FILTERS_H
#define FILTERS_H

#include <cstdint>

#define FIR_FILTER_LENGTH (16)

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





#endif // FILTERS_H