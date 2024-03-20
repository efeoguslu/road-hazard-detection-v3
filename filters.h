#ifndef FILTERS_H
#define FILTERS_H

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



#endif // FILTERS_H