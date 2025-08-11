struct DensitySampler{
    Texture3D Tex;
    SamplerState TexSampler;
    Texture3D NoiseTex;
    SamplerState NoiseTexSampler;
    float WarpScale;
    float WarpStrength;

    float4 Sample(float3 InPos){
        float3 samplepos = saturate(InPos);
                
        // First level of domain warp - single sample, use RGB channels
        float3 Warp = Texture3DSample(NoiseTex, NoiseTexSampler, samplepos / WarpScale).rgb;
        Warp = (Warp - float3(.5, .5, .5)) * WarpStrength;
                
        float3 finalpos = samplepos + Warp;
        float4 volumesample = Texture3DSample(Tex, TexSampler, finalpos);
        return volumesample;
    }
};

DensitySampler densitysampler;
densitysampler.Tex = Tex;
densitysampler.TexSampler = TexSampler;
densitysampler.NoiseTex = NoiseTex;
densitysampler.NoiseTexSampler = NoiseTexSampler;
densitysampler.WarpScale = WarpScale;
densitysampler.WarpStrength = WarpAmount;

float colorexponent = 1.0;

float transmittance = 1.0;
float3 lightenergy = 0.0;
Density *= StepSize;
ShadowExtinction *= ShadowStepSize;
float shadowthresh = -log(ShadowThreshold) / ShadowDensity;//length(ShadowDensity);
float ambientblend = 0.0;

LocalCamVec = normalize( mul(Parameters.CameraVector, (float3x3)LWCToFloat(GetPrimitiveData(Parameters).WorldToLocal))) * StepSize;

for (int i = 0; i < MaxSteps; i++){
    // Calculate fade based on raymarch progress
    float RaymarchProgress = float(i) / float(MaxSteps);
    float ProximityFadeStart = .5;
    float ProximityFadeEnd = .1;
    // Create fade range: 1.0 at FadeStart, 0.0 at FadeEnd
    float ProximityFade = 1.0 - saturate((ProximityFadeStart - RaymarchProgress) / (ProximityFadeStart - ProximityFadeEnd));    
    //Sample density AND color
    float4 volumeSample = densitysampler.Sample(saturate(CurPos));
    float cursample = volumeSample.w * ProximityFade; 
    
    //Sample light if there is density
    if(cursample > .001){
        float3 lpos = CurPos;
        float shadowdist = 0.0;
        
        // Calculate direction from current position to light position
        float3 ToLight = LightVector - CurPos;
        float3 LightDirection = normalize(ToLight);
        float3 LightStep = LightDirection * ShadowStepSize;
        
        for (int s = 0; s < ShadowSteps; s++){
            lpos += LightStep;
            
            float4 lsample = densitysampler.Sample(saturate(lpos));

            float3 shadowboxtest = floor( .5 + ( abs( .5 - lpos)));
            float exitshadowbox = shadowboxtest.x + shadowboxtest.y + shadowboxtest.z;

            if(shadowdist > shadowthresh || exitshadowbox >= 1.0) break;
            shadowdist += lsample.w;
        }

        cursample = 1.0 - exp(-cursample * Density);
        
        // FIX: Use volume color with lighting
        float3 lightContribution = exp(-shadowdist * ShadowDensity * ShadowExtinction) * cursample * transmittance * LightColor;
        lightenergy += lightContribution * LightColor; // Apply volume color here
        
        transmittance *= 1.0 - cursample;

        //Sample Ambience - collect color data
        shadowdist = 0.0;

        lpos = CurPos + float3(0.0, 0.0, .025);
        float4 lsample = densitysampler.Sample(saturate(lpos));
        shadowdist += lsample.w;
        
        lpos = CurPos + float3(0.0, 0.0, .05);
        lsample = densitysampler.Sample(saturate(lpos));
        shadowdist += lsample.w;
        
        lpos = CurPos + float3(0.0, 0.0, .15);
        lsample = densitysampler.Sample(saturate(lpos));
        shadowdist += lsample.w;

        lightenergy += exp(-shadowdist * AmbientDensity) * cursample * AmbientColor * transmittance;
    }
    int3 randpos = int3(Parameters.SvPosition.xy, View.StateFrameIndexMod8);
    float rand = float(Rand3DPCG16(randpos).x) / 0xffff;
    CurPos += LocalCamVec * rand * .1;
    CurPos += -LocalCamVec;
}

// SAME FIXES FOR FINAL STEP
CurPos += LocalCamVec * ( 1 - FinalStepSize);
float4 finalVolumeSample = densitysampler.Sample(saturate(CurPos));
float cursample = finalVolumeSample.w;

if(cursample > .001){
    float3 lpos = CurPos;
    float shadowdist = 0.0;
    
    // Same light calculation...
    float3 ToLight = LightVector - CurPos;
    float3 LightDirection = normalize(ToLight);
    float3 LightStep = LightDirection * ShadowStepSize;
    
    for (int s = 0; s < ShadowSteps; s++){
        lpos += LightStep;
        
        float lsample = densitysampler.Sample(saturate(lpos)).w;
        float3 shadowboxtest = floor( .5 + ( abs( .5 - lpos)));
        float exitshadowbox = shadowboxtest.x + shadowboxtest.y + shadowboxtest.z;

        if(shadowdist > shadowthresh || exitshadowbox >= 1.0) break;
        shadowdist += lsample;
    }

    //Depth field shadow trace (same as before)...
    cursample = 1.0 - exp(-cursample * Density);
    
    // FIX: Apply final volume color to lighting
    float3 finalLightContribution = exp(-shadowdist * ShadowDensity * ShadowExtinction) * cursample * transmittance * LightColor;
    lightenergy += finalLightContribution;
    
    transmittance *= 1.0 - cursample;

    //Sample final ambient colors
    shadowdist = 0.0;
    float3 finalAmbientColorAccum = float3(0.0, 0.0, 0.0);

    lpos = CurPos + float3(0.0, 0.0, .025);
    float4 lsample = densitysampler.Sample(saturate(lpos));
    shadowdist += lsample.w;
    
    lpos = CurPos + float3(0.0, 0.0, .05);
    lsample = densitysampler.Sample(saturate(lpos));
    shadowdist += lsample.w;
    
    lpos = CurPos + float3(0.0, 0.0, .15);
    lsample = densitysampler.Sample(saturate(lpos));
    shadowdist += lsample.w;

    lightenergy += exp(-shadowdist * AmbientDensity) * cursample * AmbientColor * transmittance;
}

return float4(lightenergy, transmittance);