float accumdens = 0;
float transmittance = 1;
float3 lightenergy = 0;
Density *= StepSize;
ShadowDensity *= ShadowStepSize;
float shadowthresh = -log(ShadowThreshold) / ShadowDensity;

LocalCamVec = normalize( mul(Parameters.CameraVector, (float3x3)LWCToFloat(GetPrimitiveData(Parameters).WorldToLocal))) * StepSize;

for (int i = 0; i < MaxSteps; i++){
    //Sample density
    float cursample = Texture3DSample(Tex, TexSampler, saturate(CurPos)).w;
    
    //Sample light if there is density
    if(cursample > .001){
        float3 lpos = CurPos;
        float shadowdist = 0;
        
        // MODIFIED: Calculate direction from current position to light position
        float3 ToLight = LightVector - CurPos; // LightVector is now a position
        float DistanceToLight = length(ToLight);
        float3 LightDirection = normalize(ToLight);
        float3 LightStep = LightDirection * ShadowStepSize;
        
        for (int s = 0; s < ShadowSteps; s++){
            lpos += LightStep;
            
            // MODIFIED: Stop shadow march if we've reached or passed the light
            float DistanceTraveled = length(lpos - CurPos);
            if(DistanceTraveled >= DistanceToLight) break;
            
            float lsample = Texture3DSample(Tex, TexSampler, saturate(lpos)).w;

            float3 shadowboxtest = floor( .5 + ( abs( .5 - lpos)));
            float exitshadowbox = shadowboxtest.x + shadowboxtest.y + shadowboxtest.z;

            if(shadowdist > shadowthresh || exitshadowbox >= 1) break;
            shadowdist += lsample;
        }

        //Depth field shadow trace
        float3 dfpos = 2 * (CurPos - .5) * GetPrimitiveData(Parameters).LocalObjectBoundsMax;
        dfpos = LWCToFloat(TransformLocalPositionToWorld(Parameters, dfpos)) - CameraPosWS;
        float dftracedist = 1;
        float dfshadow = 1;
        float curdist = 0;
        float DistanceAlongTrace = 0;

        for (int d = 0; d < DFSSteps; d++){
            DistanceAlongTrace += curdist;
            curdist = GetDistanceToNearestSurfaceGlobal(dfpos);
            float SphereSize = DistanceAlongTrace * LightTangent;
            dfshadow = min( saturate(curdist/SphereSize), dfshadow);
            dfpos.xyz += LightVectorWS * dftracedist * curdist;
            dftracedist *= 1.0001;
        }

        cursample = 1 - exp(-cursample * Density);
        lightenergy += exp(-shadowdist * ShadowDensity) * cursample * transmittance * LightColor * dfshadow;
        transmittance *= 1 - cursample;

        //Sample Ambience
        shadowdist = 0;

        lpos = CurPos + float3(0, 0, .025);
        float4 lsample = Texture3DSample(Tex, TexSampler, saturate(lpos));
        shadowdist += lsample.w;
        lpos = CurPos + float3(0, 0, .05);
        lsample = Texture3DSample(Tex, TexSampler, saturate(lpos));
        shadowdist += lsample.w;
        lpos = CurPos + float3(0, 0, .15);
        lsample = Texture3DSample(Tex, TexSampler, saturate(lpos));
        shadowdist += lsample.w;

        lightenergy += exp(-shadowdist * AmbientDensity) * cursample * AmbientColor * transmittance;
    }
    CurPos += -LocalCamVec;
}

CurPos += LocalCamVec * ( 1 - FinalStepSize);
float cursample = Texture3DSample(Tex, TexSampler, saturate(CurPos)).w;

if(cursample > .001){
    float3 lpos = CurPos;
    float shadowdist = 0;
    
    // MODIFIED: Same calculation for final step
    float3 ToLight = LightVector - CurPos;
    float DistanceToLight = length(ToLight);
    float3 LightDirection = normalize(ToLight);
    float3 LightStep = LightDirection * ShadowStepSize;
    
    for (int s = 0; s < ShadowSteps; s++){
        lpos += LightStep;
        
        // MODIFIED: Stop shadow march if we've reached or passed the light
        float DistanceTraveled = length(lpos - CurPos);
        if(DistanceTraveled >= DistanceToLight) break;
        
        float lsample = Texture3DSample(Tex, TexSampler, saturate(lpos)).w;

        float3 shadowboxtest = floor( .5 + ( abs( .5 - lpos)));
        float exitshadowbox = shadowboxtest.x + shadowboxtest.y + shadowboxtest.z;

        if(shadowdist > shadowthresh || exitshadowbox >= 1) break;
        shadowdist += lsample;
    }

    //Depth field shadow trace
    float3 dfpos = 2 * (CurPos - .5) * GetPrimitiveData(Parameters).LocalObjectBoundsMax;
    dfpos = LWCToFloat(TransformLocalPositionToWorld(Parameters, dfpos)) - CameraPosWS;
    float dftracedist = 1;
    float dfshadow = 1;
    float curdist = 0;
    float DistanceAlongTrace = 0;

    for (int d = 0; d < DFSSteps; d++){
        DistanceAlongTrace += curdist;
        curdist = GetDistanceToNearestSurfaceGlobal(dfpos);
        float SphereSize = DistanceAlongTrace * LightTangent;
        dfshadow = min( saturate(curdist/SphereSize), dfshadow);
        dfpos.xyz += LightVectorWS * dftracedist * curdist;
        dftracedist *= 1.0001;
    }

    cursample = 1 - exp(-cursample * Density);
    lightenergy += exp(-shadowdist * ShadowDensity) * cursample * transmittance * LightColor * dfshadow;
    transmittance *= 1 - cursample;

    //Sample Ambience
    shadowdist = 0;

    lpos = CurPos + float3(0, 0, .025);
    float4 lsample = Texture3DSample(Tex, TexSampler, saturate(lpos));
    shadowdist += lsample.w;
    lpos = CurPos + float3(0, 0, .05);
    lsample = Texture3DSample(Tex, TexSampler, saturate(lpos));
    shadowdist += lsample.w;
    lpos = CurPos + float3(0, 0, .15);
    lsample = Texture3DSample(Tex, TexSampler, saturate(lpos));
    shadowdist += lsample.w;

    lightenergy += exp(-shadowdist * AmbientDensity) * cursample * AmbientColor * transmittance;
}

return float4(lightenergy, transmittance);