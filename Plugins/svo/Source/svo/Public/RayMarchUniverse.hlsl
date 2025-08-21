// --- DENSITY SAMPLER STRUCT ENCAPSULATES SAMPLING LOGIC AND HELPER FUNCTIONS ---
struct DensitySampler{
    Texture3D Tex;
    SamplerState TexSampler;
    Texture3D NoiseTex;
    SamplerState NoiseTexSampler;
    float WarpScale;
    float WarpStrength;
    float TemperatureInfluence;
    float TemperatureScale;
    float3 CoolShift;
    float3 HotShift;
    float HueVariance;
    float HueVarianceScale;
    // --- SAMPLE DENSITY AND COMPOSITION AT POSITION ---
    float4 Sample(float3 InPos){
        // --- SMOOTH SAMPLE WITH DOMAIN WARP ---
        return SampleSmooth(InPos + (Texture3DSample(NoiseTex, NoiseTexSampler, InPos / WarpScale).rgb - float3(.5, .5, .5)) * WarpStrength);
    }
    // --- SAMPLE OF THE VOLUME TEXTURE WITH NEIGBOR BASED TRILINEAR SMOOTHING ---
    float4 SampleSmooth(float3 InPos){
        float3 voxelSpace = InPos * 64.0;
        float3 voxelIndex = floor(voxelSpace);
        float3 t = voxelSpace - voxelIndex;
        // --- SMOOTH STEP ---
        t = t * t * (3.0 - 2.0 * t);
        int sampleoffset = 1;
        // --- SAMPLE 8 ADACENT CORNERS ---
        float4 c000 = SampleSingle((voxelIndex + float3(0,0,0)) / 64.0);
        float4 c100 = SampleSingle((voxelIndex + float3(sampleoffset,0,0)) / 64.0);
        float4 c010 = SampleSingle((voxelIndex + float3(0,sampleoffset,0)) / 64.0);
        float4 c110 = SampleSingle((voxelIndex + float3(sampleoffset,sampleoffset,0)) / 64.0);
        float4 c001 = SampleSingle((voxelIndex + float3(0,0,sampleoffset)) / 64.0);
        float4 c101 = SampleSingle((voxelIndex + float3(sampleoffset,0,sampleoffset)) / 64.0);
        float4 c011 = SampleSingle((voxelIndex + float3(0,sampleoffset,sampleoffset)) / 64.0);
        float4 c111 = SampleSingle((voxelIndex + float3(sampleoffset,sampleoffset,sampleoffset)) / 64.0);
        // --- TRILINEAR INTERPOLATION ---
        float4 c00 = lerp(c000, c100, t.x);
        float4 c10 = lerp(c010, c110, t.x);
        float4 c01 = lerp(c001, c101, t.x);
        float4 c11 = lerp(c011, c111, t.x);
        float4 c0 = lerp(c00, c10, t.y);
        float4 c1 = lerp(c01, c11, t.y);
        return lerp(c0, c1, t.z);
    }
    // --- A SINGLE POINT SAMPLE OF THE VOLUME TEXTURE ---
    float4 SampleSingle(float3 InPos){
        float3 samplepos = saturate(InPos);
        float3 cellpos = samplepos * 64;
        float3 nearestCenter = round(cellpos);
        float3 dist = cellpos - nearestCenter;
        float r2 = dot(dist, dist);
        float4 vsample = Texture3DSample(Tex, TexSampler, samplepos);
        return float4(vsample.rgb, vsample.w);
    }
    // TEMPERATURE/DENSITY BASED SHIFTING OF COLOR ---
    float3 GetTemperatureColor(float InDensity, float3 InBaseColor){
        float3 coldcolor = InBaseColor * CoolShift;
        float3 warmcolor = InBaseColor;
        float3 hotcolor = InBaseColor * HotShift;
        float t = saturate(InDensity * TemperatureScale);
        float3 tempcolor;
        if(t < 0.5)
        {
            tempcolor = lerp(coldcolor, warmcolor, t * 2.0);
        }
        else
        {
            tempcolor = lerp(warmcolor, hotcolor, (t - 0.5) * 2.0);
        }
        return lerp(InBaseColor, tempcolor, saturate(InDensity * TemperatureInfluence));
    }
    // --- LOCATIONALLY BASED SHIFTING OF COLOR ---
    float3 GetCompositionColor(float3 InPos, float3 InBaseColor){
        // --- CONVERT TO HSV ---
        float3 hsv = RGBToHSV(InBaseColor);
        // --- HUE SHIFT BASED ON VOLUME NOISE SAMPLE ---
        hsv.x = frac(hsv.x + (Texture3DSample(NoiseTex, NoiseTexSampler, InPos * HueVarianceScale).r - 0.5) * HueVariance);
        // --- CONVERT BACK TO RBG AND RETURN ---
        return HSVToRGB(hsv);
    }
    // --- HELPER METHODS FOR SWICHING BEWEEN COLOR MODELS - RGB TO HSV ---
    float3 RGBToHSV(float3 InRGB){
        float4 K = float4(0.0, -1.0 / 3.0, 2.0 / 3.0, -1.0);
        float4 p = InRGB.g < InRGB.b ? float4(InRGB.bg, K.wz) : float4(InRGB.gb, K.xy);
        float4 q = InRGB.r < p.x ? float4(p.xyw, InRGB.r) : float4(InRGB.r, p.yzx);
        float d = q.x - min(q.w, q.y);
        float e = 1.0e-10;
        return float3(abs(q.z + (q.w - q.y) / (6.0 * d + e)), d / (q.x + e), q.x);
    }
    // --- HELPER METHODS FOR SWICHING BEWEEN COLOR MODELS - HSV TO RBG ---
    float3 HSVToRGB(float3 InHSV){
        float4 K = float4(1.0, 2.0 / 3.0, 1.0 / 3.0, 3.0);
        float3 p = abs(frac(InHSV.xxx + K.xyz) * 6.0 - K.www);
        return InHSV.z * lerp(K.xxx, saturate(p - K.xxx), InHSV.y);
    }
};
// --- SETUP DENSITY SAMPLER ---
DensitySampler densitysampler;
densitysampler.Tex = Tex;
densitysampler.TexSampler = TexSampler;
densitysampler.NoiseTex = NoiseTex;
densitysampler.NoiseTexSampler = NoiseTexSampler;
densitysampler.WarpScale = WarpScale;
densitysampler.WarpStrength = WarpAmount;
densitysampler.TemperatureInfluence = TemperatureInfluence;
densitysampler.TemperatureScale = TemperatureScale;
densitysampler.CoolShift = CoolShift;
densitysampler.HotShift = HotShift;
densitysampler.HueVariance = HueVariance;
densitysampler.HueVarianceScale = HueVarianceScale;
// --- DITHER/TEMPORAL NOISE - APPLIES RANDOM OFFSET TO THE STEP SIZE FOR THE RAY - ACTS AS A BLUR WHEN USING TEMPORAL AA ---
int3 randpos = int3(Parameters.SvPosition.xy, View.StateFrameIndexMod8);
float rand = float(Rand3DPCG16(randpos).x) / 0xffff;
StepSize *= 1.0 + rand * DitherFactor;
// --- PRECALCULATE STEP VECTOR ---
float3 stepvector = normalize(mul(Parameters.CameraVector, (float3x3)LWCToFloat(GetPrimitiveData(Parameters).WorldToLocal))) * StepSize;
// --- PROXIMITY FADE START END RANGES AS % OF RAYMARCH PROGRESS ---
float ProximityFadeStart = 0.01;
float ProximityFadeEnd   = 0.00;
// --- RAYMARCH ---
float transmittance = 1.0;
float3 volumeColor = 0.0;
for (int i = 0; i < MaxSteps; i++)
{
    // --- SAMPLE ---
    float4 s = densitysampler.Sample(saturate(CurPos));
    // --- PROXIMITY FADE ---
    float ProximityFade = 1.0 - saturate((ProximityFadeStart - float(i) / float(MaxSteps)) / (ProximityFadeStart - ProximityFadeEnd));
    float density = s.w * ProximityFade; 
    // --- APPLY COMPOSITION AND TEMPERATURE COLORING TO SAMPLE ---
    float3 temperatureColor = densitysampler.GetTemperatureColor(density, densitysampler.GetCompositionColor(CurPos, AmbientColor));
    // --- OPACITY ---
    float sigma = density * Density;
    float alpha = 1.0 - exp(-sigma * StepSize);
    // --- ACCUMULATE COLOR AND TRANSMITTANCE ---
    volumeColor += temperatureColor * alpha * transmittance;
    transmittance *= 1.0 - alpha;
    // --- EARLY TERMINATION CONDITION ---
    if (transmittance < 0.001) break;
    // --- STEP ---
    CurPos += -stepvector;
}
// ---- ONE MORE ITERATION TO MARCH THE LEFTOVER PARTIAL STEP ----
float leftover = saturate(FinalStepSize);
if (leftover > 0.00001 && transmittance > 0.001)
{
    // --- STEP LEFTOVER DISTANCE ---
    CurPos += stepvector * (1.0 - leftover);
    // --- SAMPLE ---
    float4 s = densitysampler.Sample(saturate(CurPos));
    // --- PROXIMITY FADE ---
    float ProximityFade = 1.0 - saturate((ProximityFadeStart - 1) / (ProximityFadeStart - ProximityFadeEnd));
    float density = s.w * ProximityFade;
    // --- APPLY COMPOSITION AND TEMPERATURE COLORING TO SAMPLE ---
    float3 temperatureColor = densitysampler.GetTemperatureColor(density, densitysampler.GetCompositionColor(CurPos, AmbientColor));
    // --- OPACITY ---
    float sigma = density * Density;
    float alpha = 1.0 - exp(-sigma * StepSize * leftover);
    // --- ACCUMULATE COLOR AND TRANSMITTANCE ---
    volumeColor += temperatureColor * alpha * transmittance;
    transmittance *= 1.0 - alpha;
}
// --- RETURN - EMISSIVE COLOR IN RGB - TRANSMITTANCE IN A ---
return float4(volumeColor, transmittance);