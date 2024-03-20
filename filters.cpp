#include "filters.h"

void FirstOrderIIR_Init(FirstOrderIIR *filt, float alpha){

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


float FirstOrderIIR_Update(FirstOrderIIR *filt, float in){

    filt->out = (1.0f - filt->alpha) * in + filt->alpha * filt->out;
    return filt->out;
}

// ----------------------------------------------------------------------------------------------

void IFX_EMA_Init(IFX_EMA *filt, float alpha){

    IFX_EMA_SetAlpha(filt, alpha);

    filt->out = 0.0f;
}

void IFX_EMA_SetAlpha(IFX_EMA *filt, float alpha){

    if(alpha < 0.0f){
        filt->alpha = 0.0f;
    }
    else if(alpha > 1.0f){
        filt->alpha = 1.0f;
    }
    
    filt->alpha = alpha;   
}

float IFX_EMA_Update(IFX_EMA *filt, float in){
    filt->out = filt->alpha * in + (1.0f - filt->alpha)*filt->out;
    return filt->out;
}
