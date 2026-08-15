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
#define GALAXY_MAX_OCTAVES 5

struct GalaxyDensitySampler
{
    // --- NOISE VOLUME ---
    // An AUTHORED tiling asset, not a generated one. That distinction is what keeps
    // CPU parity possible: fixed data can be exported once and trilinear-sampled in
    // C++, where a per-galaxy generated texture could not be.
    Texture3D    NoiseTex;
    SamplerState NoiseTexSampler;

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
    float ArmHeightRatio;       // vertical half-thickness at centre, fraction of DiscRadius
    float ArmHeightOuter;       // multiplier on that at the rim; >1 thickens outward
    float ArmVerticalFalloff;   // 2 = rounded/Gaussian, 1 = peaky exponential, 4+ = boxy
    float ArmProfileExponent;   // horizontal shape; >1 tightens the core, lengthens the tail
    float ArmMergeSmooth;       // 0 = hard max between arms; ~0.1 blends crossings

    // --- NOISE / DISTORTION ---
    float3 NoiseOffset;         // per-galaxy variation; a texture has no seed
    float NoiseAmount;          // multiplicative depth; >1 breaks arms into knots
    float NoiseScale;           // base frequency in normalized units
    float NoiseVerticalScale;   // extra z frequency; >1 flattens features into the plane
    float NoiseOctaves;
    float NoiseLacunarity;      // frequency step per octave; 2.0 is standard
    float NoiseGain;            // amplitude step per octave; 0.5 is standard
    float NoiseRidged;          // 0 = fbm (clouds), 1 = ridged (filaments and lanes)
    float NoiseArmMask;         // how strongly arms are modulated
    float NoiseDiscMask;        // how strongly the disc is modulated
    float WarpAmount;           // positional warp, normalized units; small values only
    float WarpScale;            // warp frequency
    float4 NoiseChannelWeights; // channel mix across the asset's frequencies (RGBA)
    float ArmVerticalCutoff;    // skip the arm loop below this vertical profile value
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

    // --- PER-ARM RECORDS, FILLED ONCE BY PrepareArms() ---
    // x = pitch factor (ki = k0 * x), y = phase, z = density multiplier,
    // w = end radius. These are invariant along a ray, so hashing them per march
    // step burned N hashes x MaxSteps per pixel for values that never changed.
    int    ArmN;
    float4 ArmData[GALAXY_MAX_ARMS];

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
    // Branch-free: round-to-nearest beats fmod plus two compares, and this runs
    // ArmN times per march step so it is squarely on the hot path.
    float WrapPi(float InAngle)
    {
        const float TwoPi = 2.0 * GALAXY_PI;
        return InAngle - TwoPi * round(InAngle / TwoPi);
    }

    // --- HOIST PER-ARM CONSTANTS OUT OF THE MARCH ---
    // Call once per pixel, before RayMarch. Every arm is filled unconditionally
    // (not just the first ArmN) so no slot is ever read uninitialised; the cost
    // is a handful of hashes once, against N x MaxSteps previously.
    void PrepareArms()
    {
        ArmN = (int)clamp(ArmCount, 1.0, (float)GALAXY_MAX_ARMS);

        float armSpacing = 2.0 * GALAXY_PI / float(ArmN);
        float discR = DiscRadius;
        int seed = (int)ArmAsymSeed;

        for (int i = 0; i < GALAXY_MAX_ARMS; i++)
        {
            float4 h = ArmHash(i, seed);

            ArmData[i] = float4(
                1.0 + ArmAsymPitch * (2.0 * h.x - 1.0),
                ArmPhaseOffset + float(i) * armSpacing
                    + ArmAsymPhase * armSpacing * (2.0 * h.y - 1.0),
                max(1.0 + ArmAsymDensity * (2.0 * h.z - 1.0), 0.0),
                discR * (1.0 - ArmAsymLength * h.w));
        }
    }

    // --- VOLUME TEXTURE LOOKUP, [-1,1] ---
    // SampleLevel, not Sample: inside a dynamic march loop the derivatives that drive
    // mip selection are undefined. Channel weights let a Perlin-Worley style asset
    // supply several frequencies from ONE fetch, which is how the texture path claws
    // back the octaves that baking into a single channel would lose.
    float SampleNoiseTex(float3 P)
    {
        float4 t = Texture3DSampleLevel(NoiseTex, NoiseTexSampler, P, 0);
        float wsum = dot(NoiseChannelWeights, float4(1.0, 1.0, 1.0, 1.0));
        float v = dot(NoiseChannelWeights, t) / max(abs(wsum), 1e-6);
        return v * 2.0 - 1.0;
    }

    // --- FRACTAL SUM, [-1,1] ---
    // NoiseRidged blends toward 1-2|n|, which turns smooth blobs into filaments --
    // that is what produces dust lanes and feathering rather than lumps.
    float FBm(float3 P, int InOctaves)
    {
        float lac = max(NoiseLacunarity, 1.0);
        float gain = saturate(NoiseGain);
        float ridge = saturate(NoiseRidged);

        float sum = 0.0;
        float amp = 1.0;
        float norm = 0.0;

        for (int o = 0; o < GALAXY_MAX_OCTAVES; o++)
        {
            if (o >= InOctaves) { break; }

            // A texture has no seed, so octaves are decorrelated by offset.
            float n = SampleNoiseTex(P + float3(17.3, 11.7, 23.1) * float(o));
            n = lerp(n, 1.0 - 2.0 * abs(n), ridge);

            sum += n * amp;
            norm += amp;
            P *= lac;
            amp *= gain;
        }

        return sum / max(norm, 1e-6);
    }

    // --- ARM-FRAME REFERENCE TWIST ---
    // The base spiral angle, without any per-arm phase. Rotating a sample position by
    // -twist before evaluating noise makes noise features wind WITH the spiral and
    // shear inward as the twist grows -- world-space noise instead cuts across arms
    // at arbitrary angles and reads as melted rather than turbulent.
    //
    // u is clamped because twist diverges logarithmically at the centre; past ~6 the
    // sincos below would lose the precision that keeps the frame continuous.
    float ReferenceTwist(float rXY)
    {
        float discR = DiscRadius;
        float u = min(log(discR / max(rXY, 1e-5)), 6.0);
        float T = max(ArmPitchTightening, 0.0);

        float pitchDeg = clamp(ArmPitchAngle, -89.0, 89.0);
        float tanP = tan(radians(max(abs(pitchDeg), 1.0)));
        float k0 = ((pitchDeg < 0.0) ? -1.0 : 1.0) / tanP;

        return k0 * (u + 0.5 * T * u * u);
    }

    // --- POSITION IN THE UN-TWISTED, ANISOTROPICALLY SCALED NOISE FRAME ---
    float3 NoiseFrame(float3 InPos, float rXY, float InScale)
    {
        float tw = ReferenceTwist(rXY);
        float st, ct;
        sincos(tw, st, ct);

        float3 pn = float3( InPos.x * ct + InPos.y * st,
                           -InPos.x * st + InPos.y * ct,
                            InPos.z * max(NoiseVerticalScale, 1e-3));
        return pn * max(InScale, 1e-6);
    }

    // --- POLYNOMIAL SMOOTH MAXIMUM ---
    // No pow/exp, unlike SmoothMax3, because this runs once per arm per march step.
    // k = 0 degenerates to a plain max.
    float SmoothMaxPoly(float A, float B, float K)
    {
        if (K <= 0.0) { return max(A, B); }
        float h = saturate(0.5 + 0.5 * (A - B) / K);
        return lerp(B, A, h) + K * h * (1.0 - h);
    }

    // --- COMBINED HORIZONTAL ARM DENSITY ---
    // Every arm contributes and the contributions are merged. This replaces a
    // nearest-arm search, which was C0-DISCONTINUOUS: the distance it returned was
    // continuous, but the WINNER'S attributes (ArmMult, pitch) flipped instantly
    // along the locus where two arms are equidistant, so any ArmAsymDensity produced
    // a hard density step -- worst exactly where arms cross.
    //
    // Affordable per-arm because the closest point on an arm lies at the SAME RADIUS
    // as the query point, so the in-plane distance is just the chord subtended by the
    // angular offset: 2 r sin(d/2). One sin, no cos/sqrt.
    //
    // Z IS DELIBERATELY NOT FOLDED IN. Folding z into the distance metric before the
    // core/envelope remap makes the vertical profile a scaled copy of the horizontal
    // one, so the gradient in z ends up much steeper -- soft sides, vertical faces.
    // The vertical profile is applied separably in SampleLayers, matching how
    // SampleDiscDensity already works.
    float SampleArmHorizontal(float3 InNormPos, float rXY)
    {
        float discR = DiscRadius;
        float armStart = ArmStartRadius * discR;

        if (rXY < 1e-6 || rXY > discR) { return 0.0; }

        // --- LOGARITHMIC SPIRAL ---
        // twistAngle(r) IS the arm's angular position; its derivative is the winding
        // rate. Constant pitch angle p means tan(p) = dr/(r dTheta), which integrates
        // to Theta = ln(r/R)/tan(p) -- a log spiral, which is what real galaxies
        // approximately are. Winding rate grows as 1/r toward the centre, so the
        // spiral tightens inward BY CONSTRUCTION, and ln() is self-bounding.
        //
        // ArmPitchTightening T lets the winding rate grow linearly in u on top:
        //   k(u) = k0 (1 + T u)   ->   Theta(u) = k0 (u + T u^2 / 2)
        float u = log(discR / max(rXY, 1e-5));      // 0 at the rim, grows inward
        float T = max(ArmPitchTightening, 0.0);
        float uTerm = u + 0.5 * T * u * u;
        float uRate = 1.0 + T * u;

        float pitchDeg = clamp(ArmPitchAngle, -89.0, 89.0);
        float tanP = tan(radians(max(abs(pitchDeg), 1.0)));
        float k0 = ((pitchDeg < 0.0) ? -1.0 : 1.0) / tanP;

        float theta = atan2(InNormPos.y, InNormPos.x);

        // --- RADIAL PROGRESS: 0 AT INNER EDGE, 1 AT DISC RIM ---
        float tRadial = saturate((rXY - armStart) / max(discR - armStart, 1e-6));
        float growthFactor = lerp(1.0, ArmRadialGrowth, tRadial);
        float core = max(ArmCoreThickness * growthFactor, 0.0);
        float envelope = max(ArmEnvelopeThickness * growthFactor, core + 1e-6);

        float blendWidth = max(ArmStartBlendWidth, 1e-6);
        float fadeW = max(0.2 * discR, 1e-6);
        float mergeK = max(ArmMergeSmooth, 0.0);

        float acc = 0.0;

        for (int i = 0; i < GALAXY_MAX_ARMS; i++)
        {
            if (i >= ArmN) { break; }

            float4 a = ArmData[i];
            float ki = k0 * a.x;

            // Signed angular offset from this arm, wrapped to [-PI, PI]. Wrapping to
            // +/-PI rather than +/-armSpacing/2 is what makes uneven spacing safe.
            float d = WrapPi(theta - (ki * uTerm + a.y));

            // --- CHORD AT CONSTANT RADIUS ---
            float xyDist = 2.0 * rXY * abs(sin(0.5 * d));

            // --- SAME-RADIUS DISTANCE -> TRUE PERPENDICULAR DISTANCE ---
            // The chord is purely tangential; the spiral tangent makes angle p with
            // that direction, so true perpendicular distance is xyDist / sqrt(1+k^2).
            // Without this, arm width is inflated by 1/sin(p), and since p varies with
            // radius when T > 0 the arms read inconsistently thick along their length.
            float kLocal = ki * uRate;
            float dist = xyDist * lerp(1.0, rsqrt(1.0 + kLocal * kLocal), saturate(ArmWidthPerp));

            // --- FADE IN FROM ARM START RADIUS ---
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

            // --- CORE / ENVELOPE REMAP ---
            float w = 0.0;
            if (dist <= core)
            {
                w = 1.0;
            }
            else if (dist < envelope)
            {
                float t = (dist - core) / (envelope - core);
                w = 1.0 - t * t * (3.0 - 2.0 * t);
            }

            // --- PER-ARM STRENGTH, AND ARMS THAT PETER OUT EARLY ---
            float mult = a.z;
            if (rXY > a.w - fadeW)
            {
                float tf = saturate((rXY - (a.w - fadeW)) / fadeW);
                mult *= 1.0 - tf * tf * (3.0 - 2.0 * tf);
            }

            acc = SmoothMaxPoly(acc, w * max(mult, 0.0), mergeK);
        }

        // --- HORIZONTAL SHAPE ---
        // ArmProfileExponent > 1 pulls density toward the centreline and lengthens
        // the tail, removing the flat-topped plateau the bare remap produces once
        // ArmRadialGrowth widens the core.
        acc = PowSafe(acc, max(ArmProfileExponent, GALAXY_POW_EPSILON));

        // Peak density drops as the arm widens (mass conservation).
        float densityScale = pow(max(growthFactor, GALAXY_POW_EPSILON),
                                 ArmDensityFalloffExponent);
        return acc * (ArmPeakDensity / max(densityScale, 1e-6));
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

        float discR = DiscRadius;

        // --- POSITIONAL WARP ---
        // Bends the geometry itself. Applied to the gaseous layers only -- the bulge
        // and halo are pressure-supported and smooth, so warping them just makes them
        // wobble. Three offset lookups rather than three FBm calls: warp wants one
        // low frequency, not a spectrum.
        float3 gasPos = InNormPos;
        if (WarpAmount > 0.0)
        {
            float rW = max(sqrt(InNormPos.x * InNormPos.x + InNormPos.y * InNormPos.y), 1e-5);
            float3 wf = NoiseFrame(InNormPos, rW, WarpScale) + NoiseOffset;

            // Three decorrelated channels from ONE fetch -- warp wants a vector, and
            // RGB supplies one directly.
            float3 wv = Texture3DSampleLevel(NoiseTex, NoiseTexSampler, wf, 0).rgb * 2.0 - 1.0;
            gasPos += wv * WarpAmount;
        }

        px = gasPos.x;
        py = gasPos.y;
        pz = gasPos.z;

        float rXY = sqrt(px * px + py * py);
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

        // --- ARMS ---
        // Skipped entirely when the arm layer is off. The arm merge is the most
        // expensive thing in the field, so this makes debug isolation of the other
        // layers cheap rather than merely correct.
        float ArmDensity = 0.0;
        if (LayerScaleArm > 0.0)
        {
            float armStart = ArmStartRadius * discR;
            float tRadialArm = saturate((rXY - armStart) / max(discR - armStart, 1e-6));

            // --- VERTICAL PROFILE, INDEPENDENT OF THE HORIZONTAL ONE ---
            // Same separable form as SampleDiscDensity. ArmVerticalFalloff = 2 gives a
            // Gaussian-like round cross-section; 1 is a peaky exponential; 4+ reads
            // boxy. Thickness grows toward the rim via ArmHeightOuter and picks up
            // DiscFlare so arms stay inside the disc they live in.
            float armH = discR * max(ArmHeightRatio, 1e-6)
                       * lerp(1.0, max(ArmHeightOuter, 1e-6), tRadialArm)
                       * (1.0 + max(DiscFlare, 0.0) * saturate(rXY / max(discR, 1e-6)));

            float zn = abs(discPos.z) / armH;
            float vExpArm = max(ArmVerticalFalloff, 0.1);
            float verticalProfile = exp(-pow(max(zn, GALAXY_POW_EPSILON), vExpArm));

            // --- VERTICAL GATE ---
            // The arm merge loop is the single most expensive thing in the field, and
            // it was previously running at every step inside the disc radius --
            // including the large majority of samples far off the plane, where the
            // vertical profile annihilates the result anyway. Bail before the loop,
            // not after.
            if (verticalProfile > max(ArmVerticalCutoff, 0.0))
            {
                ArmDensity = SampleArmHorizontal(discPos, rXY) * verticalProfile * lopsided;
            }
        }

        float DiscDensity  = SampleDiscDensity(rXY, absZ) * lopsided;
        float BulgeDensity = SampleBulgeDensity(InNormPos);

        // --- MULTIPLICATIVE MODULATION ---
        // What actually makes the field read as gas: real arms are chains of star
        // forming knots, not smooth ribbons. NoiseAmount above 1 drives the modulator
        // negative in places, which is clamped to zero and breaks arms apart.
        //
        // Gated on there being something to modulate. Most of the volume is empty, so
        // this skips the octave loop for the majority of march steps -- the same
        // early-out structure the universe layer will need.
        if (NoiseAmount > 0.0 && (ArmDensity + DiscDensity) > 1e-4)
        {
            int oct = (int)clamp(NoiseOctaves, 1.0, (float)GALAXY_MAX_OCTAVES);
            float3 nf = NoiseFrame(gasPos, max(rXY, 1e-5), NoiseScale) + NoiseOffset;
            float n = FBm(nf, oct);

            float modulator = 1.0 + NoiseAmount * n;
            ArmDensity  *= max(lerp(1.0, modulator, saturate(NoiseArmMask)),  0.0);
            DiscDensity *= max(lerp(1.0, modulator, saturate(NoiseDiscMask)), 0.0);
        }

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
gd.ArmHeightRatio            = ArmHeightRatio;
gd.ArmHeightOuter            = ArmHeightOuter;
gd.ArmVerticalFalloff        = ArmVerticalFalloff;
gd.ArmProfileExponent        = ArmProfileExponent;
gd.ArmMergeSmooth            = ArmMergeSmooth;

gd.NoiseOffset               = NoiseOffset;
gd.NoiseAmount               = NoiseAmount;
gd.NoiseScale                = NoiseScale;
gd.NoiseVerticalScale        = NoiseVerticalScale;
gd.NoiseOctaves              = NoiseOctaves;
gd.NoiseLacunarity           = NoiseLacunarity;
gd.NoiseGain                 = NoiseGain;
gd.NoiseRidged               = NoiseRidged;
gd.NoiseArmMask              = NoiseArmMask;
gd.NoiseDiscMask             = NoiseDiscMask;
gd.WarpAmount                = WarpAmount;
gd.WarpScale                 = WarpScale;
gd.NoiseChannelWeights       = NoiseChannelWeights;
gd.ArmVerticalCutoff         = ArmVerticalCutoff;

gd.NoiseTex                  = NoiseTex;
gd.NoiseTexSampler           = NoiseTexSampler;
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

// --- HOIST PER-ARM CONSTANTS: ONCE PER PIXEL, NOT ONCE PER MARCH STEP ---
gd.PrepareArms();

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