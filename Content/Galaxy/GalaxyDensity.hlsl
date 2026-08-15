// GalaxyDensity.ush
// HLSL mirror of GalaxyDataGenerator::SampleDensity.
//
// PARITY: C++ is the authority. Particle placement rejection-samples
// SampleDensity, so divergence here = star sprites outside the rendered arms.
// Rules for edits (both sides, together):
//   - No transcendental where an algebraic form exists (integer powers -> multiplies)
//   - Guard every pow() base away from zero: HLSL does pow(x,y) = exp2(y*log2(x)),
//     and log2(0) = -INF, which some compilers turn into NaN
//   - Bound anything that can overflow float32 (exp overflows ~88 on GPU vs ~709
//     for double on CPU) -- see GALAXY_MAX_TWIST_ANGLE
//   - No branch that exists on only one side
//
// SPACE: Sample() takes normalized galaxy space [-1,1], where 1.0 = Params.Extent.
// From a unit-box proxy whose CurPos runs [0,1]:  NormPos = 2.0 * (CurPos - 0.5);
//
// NoisePower is NOT in this function. The old bake applied pow(d, NoisePower) in
// SampleNoiseVolume while the particle path used raw density -- it is a render-side
// shaping term with no CPU counterpart. Apply it AFTER Sample(), never inside.

#define GALAXY_PI 3.14159265358979323846
#define GALAXY_POW_EPSILON 1e-6

// exp(80) is finite in float32; exp(88) overflows. Stops an INF entering the
// arithmetic before the angle clamp below.
#define GALAXY_MAX_TWIST_EXP_ARG 80.0

// Upper bound on |twistAngle| in radians. The core twist term is
// -ArmCoreTwistStrength * exp(ArmCoreTwistRadius / r), which diverges as r -> 0:
//   1. Overflow -- float32 goes INF ~8x further out in radius than double does
//   2. Precision -- fmod(twistAngle, armSpacing) is meaningless once ULP(twistAngle)
//      approaches armSpacing. At 512 the float32 ULP is 6.1e-5 rad, negligible vs PI.
// With shipping defaults (Strength 8, Radius 0.2) this engages below
// normalizedR ~= 0.048, at or inside ArmStartRadius (0.05), where arms have already
// faded into the bulge. Arms stop winding further in rather than winding infinitely.
#define GALAXY_MAX_TWIST_ANGLE 512.0

struct GalaxyDensitySampler
{
    float BoundsFadeStart;

    float BulgeScaleRadius;
    float BulgeCutoffRadius;
    float BulgePeakDensity;
    float BulgeVerticalSquash;

    float DiscRadius;
    float DiscHeightRatio;
    float DiscBaseDensity;
    float DiscRadialScaleLength;
    float DiscVerticalFalloff;

    float ArmCount;
    float ArmTwistStrength;
    float ArmCoreTwistStrength;
    float ArmCoreTwistRadius;
    float ArmStartRadius;
    float ArmStartBlendWidth;
    float ArmVerticalSquash;
    float ArmVerticalSquashOuter;
    float ArmRadialGrowth;
    float ArmDensityFalloffExponent;
    float ArmCoreThickness;
    float ArmEnvelopeThickness;
    float ArmPeakDensity;

    float BackgroundDensity;
    float BackgroundVerticalSquash;
    float BackgroundCutoffRadius;
    float BackgroundFadeStart;

    // --- DEBUG: PER-LAYER SCALES ---
    float LayerScaleBulge;
    float LayerScaleDisc;
    float LayerScaleArm;
    float LayerScaleBackground;

    // --- DEBUG: BLEND MODE ---
    // 0 = log-sum-exp smooth max. Matches C++. Carries a log(3)/K = 0.183 floor
    //     in empty space, so zeroing layers does NOT isolate the remainder.
    // 1 = hard max. Exact, no floor, visible creases where layers meet.
    //     Use this to see true structure and to make isolation meaningful.
    // 2 = p-norm soft max, (A^p+B^p+C^p)^(1/p). Smooth like LSE but exact at
    //     zero and exact when one layer dominates. No floor.
    float BlendMode;
    float BlendPower;   // p for mode 2; 4.0 is a good default

    // --- POW GUARDED AGAINST A ZERO BASE, WITH NO RESIDUE ---
    // max(x, EPSILON) inside pow() still leaves EPSILON^p behind, which is what
    // let the p-norm keep a floor. Layers are always non-negative, so branch.
    float PowSafe(float x, float p)
    {
        return (x > GALAXY_POW_EPSILON) ? pow(x, p) : 0.0;
    }

    // --- SMOOTH MAX OF THREE VALUES, STABILIZED LOG-SUM-EXP ---
    // Subtracting the running max keeps every exp() argument <= 0, so this
    // cannot overflow for any K.
    float SmoothMax3(float A, float B, float C, float K)
    {
        float M = max(A, max(B, C));
        float ExpA = exp(K * (A - M));
        float ExpB = exp(K * (B - M));
        float ExpC = exp(K * (C - M));
        return M + log(ExpA + ExpB + ExpC) / K;
    }

    // --- UNSIGNED DISTANCE FROM NEAREST SPIRAL ARM CENTERLINE ---
    // Finds the closest arm point AT THE SAME RADIUS as the query point, which is
    // what preserves spiral structure. Vertical distance is squashed so the arm
    // cross-section reads round edge-on.
    float SampleArmSDF(float3 InNormPos, float rXY)
    {
        float discR = DiscRadius;
        float armStart = ArmStartRadius * discR;
        float N = max(ArmCount, 1.0);

        if (rXY < 1e-6)  { return 10.0; }
        if (rXY > discR) { return rXY - discR + 1.0; }

        // --- UN-TWIST TO FIND ANGULAR OFFSET FROM NEAREST ARM ---
        float normalizedR = rXY / discR;
        float baseTwist = ArmTwistStrength * normalizedR;

        float expArg = min(ArmCoreTwistRadius / max(normalizedR, 1e-4),
                           GALAXY_MAX_TWIST_EXP_ARG);
        float coreBoost = ArmCoreTwistStrength * -exp(expArg);

        float twistAngle = clamp(baseTwist + coreBoost,
                                 -GALAXY_MAX_TWIST_ANGLE, GALAXY_MAX_TWIST_ANGLE);

        float theta = atan2(InNormPos.y, InNormPos.x);
        float untwistedTheta = theta - twistAngle;

        // --- ANGULAR DISTANCE TO NEAREST ARM ---
        float armSpacing = 2.0 * GALAXY_PI / N;
        float angDist = fmod(untwistedTheta, armSpacing);
        if (angDist < 0.0)                { angDist += armSpacing; }
        if (angDist > armSpacing * 0.5)   { angDist -= armSpacing; }

        // --- CLOSEST POINT ON THE ARM AT THIS RADIUS ---
        float armTheta = theta - angDist;
        float armX = rXY * cos(armTheta);
        float armY = rXY * sin(armTheta);

        float dx = InNormPos.x - armX;
        float dy = InNormPos.y - armY;
        float xyDist = sqrt(dx * dx + dy * dy);

        // --- RADIAL PROGRESS: 0 AT INNER EDGE, 1 AT DISC RIM ---
        float tRadial = saturate((rXY - armStart) / max(discR - armStart, 1e-6));

        float squash = lerp(ArmVerticalSquash, ArmVerticalSquashOuter, tRadial);
        float scaledZ = InNormPos.z * squash;

        float dist = sqrt(xyDist * xyDist + scaledZ * scaledZ);

        // --- FADE IN FROM ARM START RADIUS ---
        float blendWidth = max(ArmStartBlendWidth, 1e-6);
        if (rXY < armStart)
        {
            dist += (armStart - rXY);
        }
        else if (rXY < armStart + blendWidth)
        {
            float blend = (rXY - armStart) / blendWidth;
            float smoothB = blend * blend * (3.0 - 2.0 * blend);
            dist = lerp(dist + blendWidth, dist, smoothB);
        }

        return dist;
    }

    // --- HERNQUIST BULGE IN OBLATE COORDINATES ---
    // Normalised so density(r = BulgeScaleRadius) == BulgePeakDensity:
    // hernquist(a) = 1/8, hence the *8. Hard cutoff keeps the 1/r^4 tail out of
    // the disc/arm region; smoothstep avoids a cliff at the cutoff.
    float SampleBulgeDensity(float3 InNormPos)
    {
        if (BulgePeakDensity <= 0.0) { return 0.0; }

        float a = max(BulgeScaleRadius, 1e-6);
        float cutoff = max(BulgeCutoffRadius, a);

        float squashedZ = InNormPos.z / max(BulgeVerticalSquash, 1e-4);
        float r = sqrt(InNormPos.x * InNormPos.x
                     + InNormPos.y * InNormPos.y
                     + squashedZ * squashedZ);

        if (r >= cutoff) { return 0.0; }

        // Clamp rather than deal with the singularity at r = 0.
        float rClamped = max(r, a * 0.01);
        float rOverA = rClamped / a;

        // pow(1 + rOverA, 3.0) as multiplies: exact on both sides.
        float onePlus = 1.0 + rOverA;
        float hernquist = 1.0 / (rOverA * onePlus * onePlus * onePlus);
        float normalised = hernquist * 8.0;

        float fadeStart = cutoff * 0.75;
        float fade = 1.0;
        if (r > fadeStart)
        {
            float t = (r - fadeStart) / (cutoff - fadeStart);
            fade = 1.0 - t * t * (3.0 - 2.0 * t);
        }

        return saturate(BulgePeakDensity * normalised * fade);
    }

    // --- SEPARABLE EXPONENTIAL DISC PROFILE ---
    // Radial exp(-r/scaleLength) x vertical exp(-(|z|/h)^falloff).
    // Hard cylinder boundary prevents leakage above/below into the arm region.
    float SampleDiscDensity(float rXY, float absZ)
    {
        if (DiscBaseDensity <= 0.0) { return 0.0; }

        float discR = DiscRadius;
        float h = discR * max(DiscHeightRatio, 1e-6);
        float scaleL = discR * max(DiscRadialScaleLength, 1e-6);

        if (rXY >= discR || absZ >= h) { return 0.0; }

        float radialProfile = exp(-rXY / scaleL);

        // absZ == 0 is the galactic plane -- the most-sampled case in the field,
        // so this pow() base guard is load-bearing, not defensive.
        float zNorm = min(absZ / h, 1.0);
        float vExp = max(DiscVerticalFalloff, 0.1);
        float verticalProfile = exp(-pow(max(zNorm, GALAXY_POW_EPSILON), vExp));

        return DiscBaseDensity * radialProfile * verticalProfile;
    }

    // --- EVALUATE ALL FOUR LAYERS INDEPENDENTLY ---
    // Returns (bulge, disc, arm, background), each pre-multiplied by its debug
    // scale. No composition, no bounds fade.
    float4 SampleLayers(float3 InNormPos)
    {
        float px = InNormPos.x;
        float py = InNormPos.y;
        float pz = InNormPos.z;

        float rXY = sqrt(px * px + py * py);
        float absZ = abs(pz);

        // --- ARMS: SDF DISTANCE -> CORE/ENVELOPE REMAP ---
        float ArmDist = SampleArmSDF(InNormPos, rXY);

        float discR = DiscRadius;
        float armStart = ArmStartRadius * discR;
        float tRadial = saturate((rXY - armStart) / max(discR - armStart, 1e-6));

        float growthFactor = lerp(1.0, ArmRadialGrowth, tRadial);
        float core = max(ArmCoreThickness * growthFactor, 0.0);
        float envelope = max(ArmEnvelopeThickness * growthFactor, core + 1e-6);

        float densityScale = pow(max(growthFactor, GALAXY_POW_EPSILON),
                                 ArmDensityFalloffExponent);
        float peakDensity = ArmPeakDensity / max(densityScale, 1e-6);

        float ArmDensity = 0.0;
        if (ArmDist <= core)
        {
            ArmDensity = peakDensity;
        }
        else if (ArmDist < envelope)
        {
            float t = (ArmDist - core) / (envelope - core);
            float smoothT = t * t * (3.0 - 2.0 * t);
            ArmDensity = peakDensity * (1.0 - smoothT);
        }

        float DiscDensity  = SampleDiscDensity(rXY, absZ);
        float BulgeDensity = SampleBulgeDensity(InNormPos);

        // --- BACKGROUND HALO ---
        float BgDensity = 0.0;
        if (BackgroundDensity > 0.0)
        {
            float squashedZ = pz / max(BackgroundVerticalSquash, 0.01);
            float rBg = sqrt(px * px + py * py + squashedZ * squashedZ);
            float cutoff = BackgroundCutoffRadius;

            if (rBg < cutoff)
            {
                float bgFadeStart = BackgroundFadeStart * cutoff;
                float Fade = 1.0;
                if (rBg > bgFadeStart)
                {
                    float t2 = (rBg - bgFadeStart) / (cutoff - bgFadeStart);
                    Fade = 1.0 - t2 * t2 * (3.0 - 2.0 * t2);
                }
                BgDensity = BackgroundDensity * Fade;
            }
        }

        return float4(BulgeDensity * LayerScaleBulge,
                      DiscDensity  * LayerScaleDisc,
                      ArmDensity   * LayerScaleArm,
                      BgDensity    * LayerScaleBackground);
    }

    // --- SPHERICAL BOUNDS FADE ---
    float BoundsFade(float rBounds)
    {
        if (rBounds <= BoundsFadeStart) { return 1.0; }
        float t = (rBounds - BoundsFadeStart) / (1.0 - BoundsFadeStart);
        return 1.0 - t * t * (3.0 - 2.0 * t);
    }

    // --- COMPOSE LAYERS: UNION OF BULGE/DISC/ARMS, PLUS ADDITIVE BACKGROUND ---
    float Compose(float4 L, float rBounds)
    {
        float Density;

        if (BlendMode < 0.5)
        {
            // K = 6 here, NOT the SmoothMax default of 8. Matches the C++ call site.
            Density = SmoothMax3(L.x, L.y, L.z, 6.0);
        }
        else if (BlendMode < 1.5)
        {
            Density = max(L.x, max(L.y, L.z));
        }
        else
        {
            float p = max(BlendPower, 1.0);
            float s = PowSafe(L.x, p) + PowSafe(L.y, p) + PowSafe(L.z, p);
            Density = (s > 0.0) ? pow(s, 1.0 / p) : 0.0;
        }

        Density += L.w;
        Density *= BoundsFade(rBounds);

        return saturate(Density);
    }

    float Sample(float3 InNormPos)
    {
        float rBounds = length(InNormPos);
        if (rBounds >= 1.0) { return 0.0; }

        return Compose(SampleLayers(InNormPos), rBounds);
    }

    // --- ISOLATION: ONE LAYER, BOUNDS FADE ONLY, NO BLEND ---
    // 0 = bulge, 1 = disc, 2 = arms, 3 = background.
    float SampleLayer(float3 InNormPos, int InLayer)
    {
        float rBounds = length(InNormPos);
        if (rBounds >= 1.0) { return 0.0; }

        float4 L = SampleLayers(InNormPos);
        float d = (InLayer == 0) ? L.x
                : (InLayer == 1) ? L.y
                : (InLayer == 2) ? L.z
                                 : L.w;

        return saturate(d * BoundsFade(rBounds));
    }

    // --- DENSITY FETCH, ROUTED BY DEBUG MODE ---
    float FetchDensity(float3 InNormPos, int InDebugMode, float InNoisePower)
    {
        float d = (InDebugMode >= 3)
            ? SampleLayer(InNormPos, InDebugMode - 3)
            : Sample(InNormPos);

        return pow(max(d, GALAXY_POW_EPSILON), InNoisePower);
    }

    // --- DEBUG RAYMARCH: ANALYTIC SPHERE-BOUNDED DENSITY ACCUMULATION ---
    // Marches only the chord where the field can be nonzero. There is NO box
    // bounds test: CurPos starts exactly on the proxy surface, so a component is
    // exactly 0.0 or 1.0 before any FP error, and a box test breaks on iteration
    // zero wherever rounding pushes it outside -- which is view-direction
    // dependent, hence angle-dependent dropout at silhouettes and grazing angles.
    // The proxy mesh decides which pixels run; the unit sphere decides where to
    // march. Step length adapts to chord length, so quality no longer varies
    // with view angle and no trailing partial step is needed.
    //
    // InStartPos     - shaded point in unit-box local space, [0,1]
    // InViewVec      - LOCAL camera vector (surface -> camera; need not be normalized)
    // InMaxSteps     - samples across the chord
    // InDensityScale - global sigma multiplier (the "Density" material param)
    // InJitter       - [0,1] entry-point dither, resolves through temporal AA
    // InNoisePower   - render-side shaping
    // InDebugMode    - 0 composite, 1 max density, 2 optical depth,
    //                  3 bulge, 4 disc, 5 arms, 6 background
    //
    // Returns emissive in RGB, transmittance in A.
    float4 RayMarch(float3 InStartPos, float3 InViewVec, int InMaxSteps,
                    float InDensityScale, float InJitter, float InNoisePower,
                    int InDebugMode)
    {
        // Unit box [0,1] -> normalized galaxy space [-1,1]. Uniform scale, so a
        // direction is identical in both spaces.
        float3 o = 2.0 * (InStartPos - 0.5);
        float3 dir = normalize(-InViewVec);

        // --- RAY / UNIT SPHERE ---
        float b = dot(o, dir);
        float c = dot(o, o) - 1.0;
        float disc = b * b - c;
        if (disc <= 0.0) { return float4(0.0, 0.0, 0.0, 1.0); }

        float sq = sqrt(disc);
        float t0 = -b - sq;
        float t1 = -b + sq;

        // Inside the sphere (c < 0): camera is within the field, clamp the near
        // end. Outside: take the whole chord WITHOUT clamping to zero -- with an
        // inverted-normal proxy the shaded point is the far face, so the chord
        // lies at negative t from o.
        float tEnter = (c < 0.0) ? 0.0 : t0;
        float tExit  = t1;
        if (tExit <= tEnter) { return float4(0.0, 0.0, 0.0, 1.0); }

        int steps = max(InMaxSteps, 1);
        float normStep = (tExit - tEnter) / float(steps);

        // Optical path is measured in LOCAL units; normalized space is 2x local,
        // so this keeps the existing Density calibration on the same scale.
        float localStep = normStep * 0.5;

        // Dither the entry point, not the step length: every ray keeps an
        // identical total path, so density does not shimmer with the noise.
        float t = tEnter + InJitter * normStep;

        float transmittance = 1.0;
        float3 volumeColor = 0.0;
        float maxDensity = 0.0;
        float opticalDepth = 0.0;

        for (int i = 0; i < steps; i++)
        {
            float3 normPos = o + dir * t;

            float density = FetchDensity(normPos, InDebugMode, InNoisePower);

            maxDensity = max(maxDensity, density);
            opticalDepth += density * InDensityScale * localStep;

            // --- OPACITY ---
            float sigma = density * InDensityScale;
            float alpha = 1.0 - exp(-sigma * localStep);

            // --- ACCUMULATE: FLAT WHITE EMISSION, STRUCTURE ONLY ---
            volumeColor += alpha * transmittance;
            transmittance *= 1.0 - alpha;

            if (transmittance < 0.001) { break; }

            t += normStep;
        }

        if (InDebugMode == 1) { return float4(maxDensity.xxx, 1.0 - saturate(maxDensity)); }
        if (InDebugMode == 2) { return float4(opticalDepth.xxx, exp(-opticalDepth)); }

        return float4(volumeColor, transmittance);
    }
};

// --- SETUP ---
GalaxyDensitySampler gd;
gd.BoundsFadeStart           = BoundsFadeStart;

gd.BulgeScaleRadius          = BulgeScaleRadius;
gd.BulgeCutoffRadius         = BulgeCutoffRadius;
gd.BulgePeakDensity          = BulgePeakDensity;
gd.BulgeVerticalSquash       = BulgeVerticalSquash;
gd.LayerScaleBulge           = LayerScaleBulge;


gd.DiscRadius                = DiscRadius;
gd.DiscHeightRatio           = DiscHeightRatio;
gd.DiscBaseDensity           = DiscBaseDensity;
gd.DiscRadialScaleLength     = DiscRadialScaleLength;
gd.DiscVerticalFalloff       = DiscVerticalFalloff;
gd.LayerScaleDisc            = LayerScaleDisc;

gd.ArmCount                  = ArmCount;
gd.ArmTwistStrength          = ArmTwistStrength;
gd.ArmCoreTwistStrength      = ArmCoreTwistStrength;
gd.ArmCoreTwistRadius        = ArmCoreTwistRadius;
gd.ArmStartRadius            = ArmStartRadius;
gd.ArmStartBlendWidth        = ArmStartBlendWidth;
gd.ArmVerticalSquash         = ArmVerticalSquash;
gd.ArmVerticalSquashOuter    = ArmVerticalSquashOuter;
gd.ArmRadialGrowth           = ArmRadialGrowth;
gd.ArmDensityFalloffExponent = ArmDensityFalloffExponent;
gd.ArmCoreThickness          = ArmCoreThickness;
gd.ArmEnvelopeThickness      = ArmEnvelopeThickness;
gd.ArmPeakDensity            = ArmPeakDensity;
gd.LayerScaleArm             = LayerScaleArm;

gd.BackgroundDensity         = BackgroundDensity;
gd.BackgroundVerticalSquash  = BackgroundVerticalSquash;
gd.BackgroundCutoffRadius    = BackgroundCutoffRadius;
gd.BackgroundFadeStart       = BackgroundFadeStart;
gd.LayerScaleBackground      = LayerScaleBackground;

gd.BlendMode                 = BlendMode;   // 0 LSE (parity), 1 hard max, 2 p-norm
gd.BlendPower                = BlendPower;

// --- DITHER: ENTRY-POINT OFFSET, RESOLVES THROUGH TEMPORAL AA ---
int3 randpos = int3(Parameters.SvPosition.xy, View.StateFrameIndexMod8);
float rand = float(Rand3DPCG16(randpos).x) / 0xffff;

return gd.RayMarch(
    CurPos,
    LocalCamVec,      // raw, not normalized -- the method handles it
    (int)MaxSteps,
    .0015,            // InDensityScale -- WILL need recalibrating, see below
    rand,             // InJitter; pass 0.0 to disable dither
    2,                // InNoisePower
    0                 // InDebugMode
);