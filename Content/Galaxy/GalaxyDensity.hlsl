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
//     for double on CPU)
//   - No branch that exists on only one side
//
// SPACE: Sample() takes normalized galaxy space [-1,1], where 1.0 = Params.Extent.
// From a unit-box proxy whose CurPos runs [0,1]:  NormPos = 2.0 * (CurPos - 0.5);
//
// NoisePower is NOT in this function. The old bake applied pow(d, NoisePower) in
// SampleNoiseVolume while the particle path used raw density -- it is a render-side
// shaping term with no CPU counterpart. Apply it AFTER Sample(), never inside.
//
// AHEAD OF C++, to port together:
//   - logarithmic spiral by pitch angle, replacing ArmTwistStrength /
//     ArmCoreTwistStrength / ArmCoreTwistRadius
//   - BlendMode 2 (p-norm) vs the C++ log-sum-exp
//   - per-arm asymmetry (ArmAsym*) and the nearest-arm search that requires
//   - disc warp / flare / lopsidedness (DiscWarp*, DiscFlare, DiscLopsided*)

#define GALAXY_PI 3.14159265358979323846
#define GALAXY_POW_EPSILON 1e-6
#define GALAXY_MAX_ARMS 16

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

    // --- DISC ASYMMETRY ---
    float DiscFlare;            // 0 = constant scale height; >0 thickens toward the rim
    float DiscWarpAmplitude;    // integral-sign vertical warp, in normalized z units
    float DiscWarpPhase;        // radians; azimuth of the warp's rising node
    float DiscLopsidedAmount;   // m=1 in-plane density mode; 0.2-0.4 is typical
    float DiscLopsidedPhase;    // radians; azimuth of the dense side

    float ArmCount;
    float ArmPitchAngle;        // degrees at the disc rim; sign sets chirality
    float ArmPitchTightening;   // 0 = constant pitch (true log spiral); >0 tightens inward
    float ArmPhaseOffset;       // radians; rotates the whole arm set
    float ArmWidthPerp;         // 0 = same-radius width (legacy), 1 = true perpendicular
    float ArmStartRadius;
    float ArmStartBlendWidth;
    float ArmVerticalSquash;
    float ArmVerticalSquashOuter;
    float ArmRadialGrowth;
    float ArmDensityFalloffExponent;
    float ArmCoreThickness;
    float ArmEnvelopeThickness;
    float ArmPeakDensity;

    // --- PER-ARM ASYMMETRY ---
    float ArmAsymSeed;          // integer; changes which arm gets what
    float ArmAsymPitch;         // fractional pitch spread between arms
    float ArmAsymPhase;         // phase jitter, as a fraction of arm spacing
    float ArmAsymDensity;       // strength spread between arms
    float ArmAsymLength;        // fraction of disc radius an arm may end short by

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

    // --- INTEGER HASH, ONE float4 PER ARM INDEX ---
    // uint arithmetic wraps identically in C++ and HLSL, so this is bit-exact
    // across both -- no tolerance discussion needed when the port happens. This
    // is the same primitive the universe layer will need for its noise field.
    float4 ArmHash(int InIndex, int InSeed)
    {
        uint n = (uint)(InIndex * 73856093) ^ (uint)(InSeed * 19349663);
        uint4 v;
        v.x = n   * 1664525u + 1013904223u;
        v.y = v.x * 1664525u + 1013904223u;
        v.z = v.y * 1664525u + 1013904223u;
        v.w = v.z * 1664525u + 1013904223u;
        v ^= v >> 16u;
        v *= 1664525u;
        v ^= v >> 16u;
        return float4(v) * (1.0 / 4294967296.0);
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

    // --- WRAP A SIGNED ANGLE INTO [-PI, PI] ---
    float WrapPi(float InAngle)
    {
        float a = fmod(InAngle, 2.0 * GALAXY_PI);
        if (a >  GALAXY_PI) { a -= 2.0 * GALAXY_PI; }
        if (a < -GALAXY_PI) { a += 2.0 * GALAXY_PI; }
        return a;
    }

    // --- DISTANCE TO NEAREST ARM CENTERLINE, PLUS THAT ARM'S DENSITY MULTIPLIER ---
    // Returns (unsigned distance, per-arm density multiplier).
    //
    // Each arm is searched explicitly rather than found by fmod. The fmod shortcut
    // relies on arms being evenly spaced, which stops being true the moment
    // ArmAsymPhase or ArmAsymPitch is nonzero -- it would then select the wrong arm
    // and produce a discontinuity wherever the true nearest changes. The loop is
    // cheap because u and uTerm are shared across arms.
    float2 SampleArmSDF(float3 InNormPos, float rXY)
    {
        float discR = DiscRadius;
        float armStart = ArmStartRadius * discR;
        int N = (int)clamp(ArmCount, 1.0, (float)GALAXY_MAX_ARMS);

        if (rXY < 1e-6)  { return float2(10.0, 1.0); }
        if (rXY > discR) { return float2(rXY - discR + 1.0, 1.0); }

        // --- LOGARITHMIC SPIRAL ---
        // twistAngle(r) IS the arm's angular position; its derivative is the winding
        // rate. Constant pitch angle p means tan(p) = dr/(r dTheta), which integrates
        // to Theta = ln(r/R)/tan(p) -- a log spiral, which is what real galaxies
        // approximately are. Winding rate dTheta/dr = 1/(r tan p) grows as 1/r toward
        // the center, so the spiral tightens inward BY CONSTRUCTION. No separate
        // core-twist term, no clamp: ln() is self-bounding where exp(C/r) was not.
        //
        // ArmPitchTightening T lets the winding rate grow linearly in u on top of that:
        //   k(u) = k0 (1 + T u)   ->   Theta(u) = k0 (u + T u^2 / 2)
        // Monotonic for T >= 0. At r = 1e-5 (u = 11.5) Theta is ~43 rad at T=0 or
        // ~325 rad at T=1 -- both comfortably inside float32.
        float u = log(discR / max(rXY, 1e-5));      // 0 at the rim, grows inward
        float T = max(ArmPitchTightening, 0.0);
        float uTerm = u + 0.5 * T * u * u;
        float uRate = 1.0 + T * u;

        float pitchDeg = clamp(ArmPitchAngle, -89.0, 89.0);
        float tanP = tan(radians(max(abs(pitchDeg), 1.0)));
        float k0 = ((pitchDeg < 0.0) ? -1.0 : 1.0) / tanP;

        float theta = atan2(InNormPos.y, InNormPos.x);
        float armSpacing = 2.0 * GALAXY_PI / float(N);

        int seed = (int)ArmAsymSeed;

        float bestAbs  = 1e9;
        float bestAng  = 0.0;
        float bestK    = k0 * uRate;
        float bestMult = 1.0;

        for (int i = 0; i < GALAXY_MAX_ARMS; i++)
        {
            if (i >= N) { break; }

            float4 h = ArmHash(i, seed);

            // --- THIS ARM'S OWN PITCH AND PHASE ---
            float ki    = k0 * (1.0 + ArmAsymPitch * (2.0 * h.x - 1.0));
            float phase = ArmPhaseOffset + float(i) * armSpacing
                        + ArmAsymPhase * armSpacing * (2.0 * h.y - 1.0);

            float twistI = ki * uTerm + phase;

            // Signed angular offset from this arm, wrapped to [-PI, PI]. Wrapping to
            // +/-PI rather than +/-armSpacing/2 is what makes uneven spacing safe.
            float d = WrapPi(theta - twistI);
            float ad = abs(d);

            if (ad < bestAbs)
            {
                bestAbs = ad;
                bestAng = d;
                bestK   = ki * uRate;

                // --- PER-ARM STRENGTH, AND ARMS THAT PETER OUT EARLY ---
                float mult = 1.0 + ArmAsymDensity * (2.0 * h.z - 1.0);

                float rEnd  = discR * (1.0 - ArmAsymLength * h.w);
                float fadeW = max(0.2 * discR, 1e-6);
                if (rXY > rEnd - fadeW)
                {
                    float tf = saturate((rXY - (rEnd - fadeW)) / fadeW);
                    mult *= 1.0 - tf * tf * (3.0 - 2.0 * tf);
                }

                bestMult = max(mult, 0.0);
            }
        }

        // --- CLOSEST POINT ON THE WINNING ARM AT THIS RADIUS ---
        float armTheta = theta - bestAng;
        float armX = rXY * cos(armTheta);
        float armY = rXY * sin(armTheta);

        float dx = InNormPos.x - armX;
        float dy = InNormPos.y - armY;
        float xyDist = sqrt(dx * dx + dy * dy);

        // --- SAME-RADIUS DISTANCE -> TRUE PERPENDICULAR DISTANCE ---
        // xyDist measures along the circle of constant radius, which is purely
        // tangential. The spiral's tangent makes angle p with that direction, so the
        // true perpendicular distance is xyDist * sin(p) = xyDist / sqrt(1 + k^2).
        // Without this, arm width is inflated by 1/sin(p) -- and since p varies with
        // radius whenever T > 0, the arms read inconsistently thick along their length.
        xyDist *= lerp(1.0, rsqrt(1.0 + bestK * bestK), saturate(ArmWidthPerp));

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

        return float2(dist, bestMult);
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
        float rn = saturate(rXY / max(discR, 1e-6));

        // --- FLARE ---
        // Real discs thicken outward; a constant scale height is one of the strongest
        // "machined" tells in an edge-on view.
        float h = discR * max(DiscHeightRatio, 1e-6) * (1.0 + max(DiscFlare, 0.0) * rn);
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
        float discR = DiscRadius;
        float rn = rXY / max(discR, 1e-6);
        float theta = atan2(py, px);

        // --- DISC WARP: THE INTEGRAL-SIGN m=1 VERTICAL MODE ---
        // Grows as r^2 so the inner disc and bulge stay put. Applied to arms and
        // disc only -- the bulge and halo are pressure-supported and do not warp.
        float warpZ = DiscWarpAmplitude * rn * rn * sin(theta - DiscWarpPhase);
        float3 discPos = float3(px, py, pz - warpZ);
        float absZ = abs(discPos.z);

        // --- LOPSIDEDNESS: THE m=1 IN-PLANE MODE ---
        // Very common in real galaxies; one side of the disc is simply denser.
        float lopsided = max(1.0 + DiscLopsidedAmount * cos(theta - DiscLopsidedPhase), 0.0);

        // --- ARMS: SDF DISTANCE -> CORE/ENVELOPE REMAP ---
        float2 armResult = SampleArmSDF(discPos, rXY);
        float ArmDist = armResult.x;
        float ArmMult = armResult.y;

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
        ArmDensity *= ArmMult * lopsided;

        float DiscDensity  = SampleDiscDensity(rXY, absZ) * lopsided;
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
gd.DiscFlare                 = 0;
gd.DiscWarpAmplitude         = 0;
gd.DiscWarpPhase             = 0;
gd.DiscLopsidedAmount        = 0;
gd.DiscLopsidedPhase         = 0;
gd.LayerScaleDisc            = LayerScaleDisc;

gd.ArmCount                  = ArmCount;
gd.ArmPitchAngle             = ArmPitchAngle;
gd.ArmPitchTightening        = ArmPitchTightening;
gd.ArmPhaseOffset            = ArmPhaseOffset;
gd.ArmWidthPerp              = ArmWidthPerp;
gd.ArmStartRadius            = ArmStartRadius;
gd.ArmStartBlendWidth        = 0;
gd.ArmVerticalSquash         = ArmVerticalSquash;
gd.ArmVerticalSquashOuter    = ArmVerticalSquashOuter;
gd.ArmRadialGrowth           = ArmRadialGrowth;
gd.ArmDensityFalloffExponent = ArmDensityFalloffExponent;
gd.ArmCoreThickness          = ArmCoreThickness;
gd.ArmEnvelopeThickness      = ArmEnvelopeThickness;
gd.ArmPeakDensity            = ArmPeakDensity;
gd.ArmAsymSeed               = ArmAsymSeed;
gd.ArmAsymPitch              = ArmAsymPitch;
gd.ArmAsymPhase              = ArmAsymPhase;
gd.ArmAsymDensity            = ArmAsymDensity;
gd.ArmAsymLength             = ArmAsymLength;
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
    1,            // InDensityScale
    rand,             // InJitter; pass 0.0 to disable dither
    1,                // InNoisePower
    1                 // InDebugMode
);