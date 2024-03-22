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


//-----------------------------------------------------------------------------------------------

static float FIR_IMPULSE_RESPONSE[FIR_FILTER_LENGTH] = {-0.0032906f, -0.0052635f, }; // TBA

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


float FIRFilter_Update(FIRFilter *fir, float inp){

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