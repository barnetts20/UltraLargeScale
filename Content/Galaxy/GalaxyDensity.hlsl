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
//   - layer composition via SmoothMaxPoly, replacing the C++ log-sum-exp SmoothMax
//     (which carried a log(N)/K floor); evaluation order is bulge, disc, arms
//   - per-arm asymmetry (ArmAsym*) and the all-arm merge that requires
//   - elliptical arm cross-section: ArmWidth + ArmVerticalRatio replace
//     ArmCoreThickness / ArmEnvelopeThickness / ArmHeightRatio / ArmHeightOuter /
//     ArmVerticalFalloff / ArmWidthPerp
//   - disc warp / flare / lopsidedness (DiscWarp*, DiscFlare, DiscLopsided*)
//   - noise modulation and positional warp from the tiling volume asset

// LAYER ORDER, everywhere in this file: x = arms, y = disc, z = bulge,
// w = background. SampleLayers' return, SampleLayer's index, the debug modes, and
// every float4 input pin all follow it. There is deliberately only one order.

#define GALAXY_PI 3.14159265358979323846
#define GALAXY_POW_EPSILON 1e-6
#define GALAXY_MAX_ARMS 16
#define GALAXY_MAX_OCTAVES 2

// Standard fractal ratios: each octave doubles frequency and halves amplitude.
// Constants rather than parameters -- with a pre-baked FBm asset the spectrum is
// already inside the texture, so at one or two octaves these had almost no effect,
// and no value other than the standard pair was ever worth reaching for.
// Separates the halo noise domain from the gas one so their structure does not
// correlate. Arbitrary, just needs to be large relative to feature size.
#define GALAXY_HALO_DOMAIN float3(53.7, 29.1, 71.3)

#define GALAXY_NOISE_LACUNARITY 2.0
#define GALAXY_NOISE_GAIN 0.5

struct GalaxyDensitySampler
{
    // --- NOISE VOLUME ---
    // An AUTHORED tiling asset, not a generated one. That distinction is what keeps
    // CPU parity possible: fixed data can be exported once and trilinear-sampled in
    // C++, where a per-galaxy generated texture could not be.
    Texture3D    NoiseTex;
    SamplerState NoiseTexSampler;

    float BoundsFadeStart;

    float BulgeScaleRadius;        // radius at which the layer reaches EXACTLY zero
    float BulgePeakDensity;        // density at the centre -- now literally that
    float BulgeVerticalSquash;
    float BulgeConcentration;      // profile exponent; see SampleSpheroid

    float DiscRadius;
    float DiscHeightRatio;
    float DiscBaseDensity;
    float DiscScaleLengthRatio; // MULTIPLE OF BulgeScaleRadius, not of DiscRadius
    float DiscVerticalFalloff;

    // --- DISC ASYMMETRY ---
    float DiscFlare;            // 0 = constant scale height; >0 thickens toward the rim
    float DiscWarpAmplitude;    // integral-sign vertical warp, in normalized z units
    float DiscWarpPhase;        // radians; azimuth of the warp's rising node at r = 0
    float DiscWarpTwist;        // radians of node-line precession across the disc
    float DiscLopsidedAmount;   // m=1 in-plane density mode; 0.2-0.4 is typical
    float DiscLopsidedPhase;    // radians; azimuth of the dense side

    float ArmCount;
    float ArmPitchAngle;        // degrees at the disc rim; sign sets chirality
    float ArmPitchTightening;   // 0 = constant pitch (true log spiral); >0 tightens inward
    float ArmPhaseOffset;       // radians; rotates the whole arm set
    float HaloTwistInherit;     // how much of the spiral twist the bulge/background
                                // noise frame picks up; see NoiseFrame
    float ArmWidth;             // TRUE perpendicular half-width at the arm's inner end
    float ArmVerticalRatio;     // H/W. 1 = circular tube, <1 = ribbon, >1 = vertical sheet
    float ArmProfileExponent;   // cross-section falloff. <1 flattens the top toward a
                                // plateau, 1 is a smoothstep bump, >1 sharpens the core
    float ArmRadialGrowth;
    float ArmDensityFalloffExponent;
    float ArmPeakDensity;
    float ArmMergeSmooth;       // fraction of the larger arm's density, not absolute.
                                // 0 = hard max between arms; ~0.1 blends crossings

    // --- NOISE FIELD SHAPE ---
    // FRAME CONVENTION, for NoiseScale and WarpScale only:
    //   x = disc lateral, y = disc vertical, z = halo lateral, w = halo vertical
    // This is NOT the layer convention (arm/disc/bulge/background) used by
    // LayerDensity, NoiseAmount, LateralScale and VerticalScale. Frames are a
    // property of the noise field; layers are what consume it.
    //
    // Splitting lateral from vertical per frame is what lets the warp field have
    // different anisotropy than the modulation field -- previously one shared
    // vertical scale was applied inside the frame, so both inherited it.
    float3 NoiseOffset;         // per-galaxy variation; a texture has no seed
    float4 NoiseScale;          // modulation frequency; frame convention above
    float4 WarpScale;           // warp frequency; frame convention above
    float NoiseOctaves;
    float NoiseRidged;          // 0 = fbm (clouds), 1 = ridged (filaments and lanes)

    // --- MASTER NOISE BYPASS ---
    // 0 skips every volume-texture read: both warps and both modulations. What
    // remains is the pure analytic field, which is EXACTLY what the C++ rejection
    // sampler will compute on the first pass of the port -- so this A/Bs shader
    // against future particle placement without disturbing any tuned value.
    //
    // Drive it from a StaticSwitchParameter between literal 0 and 1 and it resolves
    // to a compile-time constant, so the branches fold away and the texture reads are
    // dead-stripped. One node, no extra permutations.
    float NoiseEnable;
    float4 NoiseChannelWeights; // channel mix across the asset's frequencies (RGBA)

    // --- PER-LAYER NOISE CONTRIBUTION ---
    // Amounts are direct multiplicative depth: density *= 1 + Amount * n. Above 1 the
    // modulator goes negative in places, clamps to zero, and breaks the layer into
    // knots. This replaces NoiseAmount x NoiseLayerMask, which was two knobs doing
    // one job.
    float NoiseAmountArm;
    float NoiseAmountDisc;
    float NoiseAmountBulge;
    float NoiseAmountBackground;
    // Positional warp, in normalized units, one per layer. SIGNED: a negative value
    // flips the displacement direction for that layer, which is free variety --
    // the sampled warp vector is already centred on zero.
    float WarpAmountArms;
    float WarpAmountDisc;
    float WarpAmountBulge;
    float WarpAmountBackground;

    // --- CENTRAL VOID ---
    // The region cleared by the central black hole. Same profile family as
    // SampleSpheroid but SPHERICAL (no squash -- accretion is not disc-aligned) and
    // SUBTRACTIVE. Applied in Compose, so it carves every layer at once rather than
    // needing a term in each.
    float VoidRadius;           // radius at which the void stops removing anything
    float VoidStrength;         // 0 = off, 1 = fully empty at the centre
    float VoidConcentration;    // profile exponent; higher = tighter, harder-edged

    // --- PER-ARM ASYMMETRY ---
    float ArmAsymSeed;          // integer; changes which arm gets what
    float ArmAsymPitch;         // fractional pitch spread between arms
    float ArmAsymPhase;         // phase jitter, as a fraction of arm spacing
    float ArmAsymDensity;       // strength spread between arms
    float ArmAsymLength;        // fraction of disc radius an arm may end short by

    // --- PER-ARM RECORDS, FILLED ONCE IN THE DRIVER BLOCK BELOW ---
    // x = pitch factor (ki = k0 * x), y = phase, z = density multiplier,
    // w = end radius. These are invariant along a ray, so hashing them per march
    // step burned N hashes x MaxSteps per pixel for values that never changed.
    //
    // Filled by plain statement code rather than a member function: DXC (SM6) rejects
    // a void-returning member that mutates struct state on the function-scope struct
    // a Custom node produces, which FXC (SM5) accepted.
    int    ArmN;
    float4 ArmData[GALAXY_MAX_ARMS];

    float BackgroundDensity;        // density at the centre
    float BackgroundVerticalSquash;
    float BackgroundCutoffRadius;   // radius at which the layer reaches EXACTLY zero
    float BackgroundConcentration;  // profile exponent; see SampleSpheroid


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

    // --- VOLUME TEXTURE LOOKUP, SIGNED ---
    // SampleLevel, not Sample: inside a dynamic march loop the derivatives that drive
    // mip selection are undefined.
    //
    // Each channel is centred to [-1,1] FIRST, then weighted. That ordering is what
    // makes the weights behave like a mixer:
    //   positive w    that channel, scaled
    //   negative w    that channel inverted, scaled
    //   |w| > 1       gain -- the weights are not renormalised, deliberately
    //   all zero      returns 0, so the modulator is exactly 1. No special case.
    //
    // Weighting before centring (dot(W,t) then remap) cannot express this: it needs
    // a signed-sum normalisation that divides by ~zero for (1,-1,0,0) and pushes the
    // result outside [-1,1] for any mixed-sign pair.
    //
    // Output is NOT bounded to [-1,1] -- with sum|w| > 1 it exceeds that by design.
    // The modulator downstream is 1 + Amount * n, so large |n| can spike density as
    // well as clamp it to zero. Keep sum|w| near 1 unless gain is what you want.
    //
    // Useful combinations for a Perlin-Worley style asset:
    //   (1,0,0,0)      one channel, unmodified
    //   (-1,0,0,0)     that channel inverted
    //   (0.6,0.3,0.1,0) approximates an FBm in a single fetch
    //   (1,-1,0,0)     difference of two channels -- subtracting a Worley channel
    //                  from a Perlin one carves cell-boundary voids out of a smooth
    //                  field, closer to real dust than either channel alone
    float SampleNoiseTex(float3 P)
    {
        float4 t = Texture3DSampleLevel(NoiseTex, NoiseTexSampler, P, 0);

        // Centre each channel to [-1,1].
        float4 c = t * 2.0 - 1.0;

        // --- RIDGE, PER CHANNEL, BEFORE WEIGHTING ---
        // 1 - 2|c| folds the field about zero, so peaks land where the base noise
        // CROSSES zero -- that is what turns blobs into filaments and dust lanes.
        //
        // It must happen here, not on the weighted sum: the fold only maps back into
        // [-1,1] when its input is already bounded there. Each centred channel is by
        // construction; the weighted sum is NOT, since sum|w| > 1 is allowed as gain.
        // Ridging afterwards would fold at half amplitude and shift the output range.
        // Vectorises 4-wide, so per channel costs the same as one.
        c = lerp(c, 1.0 - 2.0 * abs(c), saturate(NoiseRidged));

        return dot(NoiseChannelWeights, c);
    }

    // --- FRACTAL SUM ---
    // Range follows SampleNoiseTex: [-1,1] when sum|w| <= 1, wider when weights carry
    // gain. Ridging is applied per channel inside SampleNoiseTex, not here.
    float FBm(float3 P, int InOctaves)
    {
        float sum = 0.0;
        float amp = 1.0;
        float norm = 0.0;

        for (int o = 0; o < GALAXY_MAX_OCTAVES; o++)
        {
            if (o >= InOctaves) { break; }

            // A texture has no seed, so octaves are decorrelated by offset. Ridging
            // happens inside SampleNoiseTex, per channel -- still once per octave,
            // which is what keeps ridges coherent across scales.
            float n = SampleNoiseTex(P + float3(17.3, 11.7, 23.1) * float(o));

            sum += n * amp;
            norm += amp;
            P *= GALAXY_NOISE_LACUNARITY;
            amp *= GALAXY_NOISE_GAIN;
        }

        return sum / max(norm, 1e-6);
    }

    // --- SPIRAL TWIST OF THE NOISE FRAME ---
    // The base spiral angle, without any per-arm phase. Rotating a sample position by
    // -twist before evaluating noise makes noise features wind WITH the spiral and
    // shear inward as the twist grows -- world-space noise instead cuts across arms
    // at arbitrary angles and reads as melted rather than turbulent.
    //
    // InInherit scales it. 1 is the full gas twist; the halo uses a fraction.
    //
    // THE RAMP IS WHY THE HALO CAN HAVE ANY TWIST AT ALL. Raw twist grows as
    // ln(R/r), so at r = 0.05R it is already 11 radians -- nearly two full turns,
    // which would smear bulge noise into azimuthal streaks. Multiplying by r/R turns
    // the product into a BUMP: zero at the centre, zero at the rim, peaking at
    // r = R/e ~ 0.37R at about 1.4 rad. That peak sits exactly at the bulge/disc
    // transition, where the shared kinematics actually are -- pseudobulges are
    // rotationally supported and built from disc material, and boxy bulges are
    // buckled bars that co-rotate. Zero at the centre is also what dispersion
    // support implies, and what the numerics want.
    //
    // Hoisted to once per sample. It used to run inside every frame construction --
    // twice per sample for warp and modulation -- each paying a log, a tan and a
    // sincos.
    float SpiralTwistAt(float rXY, float InInherit)
    {
        if (InInherit == 0.0) { return 0.0; }

        float discR = max(DiscRadius, 1e-6);
        float rn = saturate(rXY / discR);

        float u = min(log(discR / max(rXY, 1e-5)), 6.0);
        float T = max(ArmPitchTightening, 0.0);

        float pitchDeg = clamp(ArmPitchAngle, -89.0, 89.0);
        float tanP = tan(radians(max(abs(pitchDeg), 1.0)));
        float k0 = ((pitchDeg < 0.0) ? -1.0 : 1.0) / tanP;

        return k0 * (u + 0.5 * T * u * u) * rn * InInherit;
    }

    // --- NOISE FRAME: UN-TWIST, THEN SCALE ANISOTROPICALLY ---
    // One function for both frames. They were separate while the halo had no twist;
    // now that it inherits a fraction, "halo" is just a smaller InTwist and a domain
    // offset, not a different code path.
    //
    // InScale is (lateral, vertical). A vertical larger than lateral compresses z, so
    // features read as sheets rather than blobs -- appropriate for the rotationally
    // supported gas layers, and best left near 1:1 for the pressure supported ones,
    // which apply their own flattening downstream.
    //
    // InOffset separates the two frames' domains so halo structure does not
    // correlate with gas structure.
    float3 NoiseFrame(float3 InPos, float InTwist, float2 InScale, float3 InOffset)
    {
        float st, ct;
        sincos(InTwist, st, ct);

        float lat  = max(InScale.x, 1e-6);
        float vert = max(InScale.y, 1e-6);

        return float3((InPos.x * ct + InPos.y * st) * lat,
                     (-InPos.x * st + InPos.y * ct) * lat,
                       InPos.z * vert) + InOffset;
    }

    // --- POLYNOMIAL SMOOTH MAXIMUM, RELATIVE BAND ---
    // No pow or exp: this runs once per arm per march step, and again per layer.
    // K = 0 degenerates to a plain max.
    //
    // K is a FRACTION of the larger operand, not an absolute density. With an
    // absolute band the blend region was as wide at zero as at peak density, so
    // smax(0,0) returned K/4 out of nothing -- and that floor COMPOUNDED across the
    // arm loop's up-to-16 merges and Compose's chained pair, manufacturing background
    // density in empty space. Exactly the failure the log-sum-exp had, just quieter.
    //
    // Scaling the band by max(A,B) makes the operator exact at zero and exact
    // whenever either operand is zero, while still blending genuine overlaps. It is
    // also scale-invariant, so K keeps its meaning as peak densities are retuned.
    float SmoothMaxPoly(float A, float B, float K)
    {
        float m = max(A, B);

        float k = K * m;
        if (k <= 0.0) { return m; }

        float h = saturate(0.5 + 0.5 * (A - B) / k);
        return lerp(B, A, h) + k * h * (1.0 - h);
    }

    // --- COMBINED ARM DENSITY ---
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
    // CROSS-SECTION IS A NORMALIZED ELLIPSE. Perpendicular and vertical offsets are
    // each divided by their own half-extent, then combined into one coordinate q that
    // reaches 1 at the arm surface. A single profile applied to q is what makes the
    // arm genuinely tubular -- two separate profile functions cannot be round even
    // when their extents match, which is why the old ArmVerticalFalloff and
    // ArmCoreThickness are gone. Vertical extent is DERIVED as W * ArmVerticalRatio,
    // so the arm thickens vertically as ArmRadialGrowth widens it, with no second
    // flare control to fight the first.
    float SampleArmDensity(float3 InNormPos, float rXY)
    {
        float discR = DiscRadius;

        if (rXY < 1e-6 || rXY > discR) { return 0.0; }

        // --- CROSS-SECTION HALF-EXTENTS ---
        // The guard above bounds rXY to (0, discR], so this needs no saturate.
        float tRadial = rXY / max(discR, 1e-6);
        float growthFactor = lerp(1.0, ArmRadialGrowth, tRadial);
        float W = max(ArmWidth * growthFactor, 1e-6);
        float H = max(W * max(ArmVerticalRatio, 1e-4), 1e-6);

        // --- EXACT VERTICAL GATE ---
        // q >= qz always, and the arm surface is q = 1, so qz >= 1 proves the density
        // is zero for every arm at this sample. Bailing here skips the whole merge
        // loop for the large majority of samples, which sit off the disc plane. Exact
        // rather than a tuned cutoff, so it costs no accuracy at all.
        float qz = abs(InNormPos.z) / H;
        if (qz >= 1.0) { return 0.0; }

        float qzSq = qz * qz;

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

        float fadeW = max(0.2 * discR, 1e-6);
        float mergeK = max(ArmMergeSmooth, 0.0);

        const float TwoPi = 2.0 * GALAXY_PI;
        const float InvTwoPi = 1.0 / TwoPi;

        float acc = 0.0;

        for (int i = 0; i < GALAXY_MAX_ARMS; i++)
        {
            if (i >= ArmN) { break; }

            float4 a = ArmData[i];
            float ki = k0 * a.x;

            // Signed angular offset from this arm, wrapped to [-PI, PI]. Wrapping to
            // +/-PI rather than +/-armSpacing/2 is what makes uneven spacing safe.
            // Inlined and branch-free: round-to-nearest beats fmod plus two compares,
            // and this runs ArmN times per march step.
            float dRaw = theta - (ki * uTerm + a.y);
            float d = dRaw - TwoPi * round(dRaw * InvTwoPi);

            // --- CHORD AT CONSTANT RADIUS -> TRUE PERPENDICULAR DISTANCE ---
            // The chord is purely tangential; the spiral tangent makes angle p with
            // that direction, so perpendicular distance is chord / sqrt(1+k^2).
            // Unconditional now: a fractional blend toward the tangential measure had
            // no geometric meaning, and with the correction always applied ArmWidth is
            // a true perpendicular half-width that no longer shifts when
            // ArmPitchAngle changes.
            float kLocal = ki * uRate;
            float dPerp = 2.0 * rXY * abs(sin(0.5 * d)) * rsqrt(1.0 + kLocal * kLocal);

            // --- NORMALIZED ELLIPTICAL CROSS-SECTION COORDINATE ---
            float qp = dPerp / W;
            float q = sqrt(qp * qp + qzSq);

            float w = 0.0;
            if (q < 1.0)
            {
                w = 1.0 - q * q * (3.0 - 2.0 * q);
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

        // --- CROSS-SECTION SHAPE ---
        // Applied once to the merged result rather than per arm, so it stays a single
        // pow per sample. Below 1 the core flattens toward a plateau; above 1 it
        // sharpens and the tail lengthens.
        acc = PowSafe(acc, max(ArmProfileExponent, GALAXY_POW_EPSILON));

        // Peak density drops as the arm widens (mass conservation).
        float densityScale = pow(max(growthFactor, GALAXY_POW_EPSILON),
                                 ArmDensityFalloffExponent);
        return acc * (ArmPeakDensity / max(densityScale, 1e-6));
    }

    // --- COMPACT SQUASHED SPHEROID, SHARED BY BULGE AND BACKGROUND ---
    //     d = InPeak * (1 - x^2)^p,   x = r_oblate / InRadius
    //
    // Bulge and background are the SAME OBJECT at different scales: a squashed
    // spheroid with a density, a radius and a concentration. Sharing one function is
    // the unified layer paradigm made literal, and halves what has to be ported.
    //
    // This replaced a Hernquist profile on the bulge, which was cusped -- rho ~ 1/x,
    // reaching 776x peak at its x = 0.01 clamp. The saturate() that hid that was load
    // bearing, and it meant BulgePeakDensity did not set a peak at all: it set the
    // RADIUS OF A SATURATED BALL (0.04 -> solid out to x = 0.19; 0.3 -> out to 0.59).
    // That is the "outsized, more geometric than density" behaviour the original C++
    // params flagged. Here the centre value is exactly InPeak, with no clamp.
    //
    // Losing the cusp costs nothing architecturally: real bulges have extended wings,
    // but the BACKGROUND already supplies those. Bulge is the concentrated core,
    // background the extended halo, and they sum. Neither has to span both jobs.
    //
    // Three properties that matter in layers evaluated on nearly every march step:
    //   COMPACT SUPPORT   exactly zero at x >= 1, self-terminating, free early-out
    //   NO SQRT           works on r^2 directly
    //   ONE EXPONENT      p < 1 fills the volume and drops at the rim; 1 parabolic;
    //                     2 smooth bump; >2 concentrates toward the centre
    float SampleSpheroid(float3 InNormPos, float InPeak, float InRadius,
                         float InSquash, float InConcentration)
    {
        if (InPeak <= 0.0) { return 0.0; }

        float a = max(InRadius, 1e-6);
        float squashedZ = InNormPos.z / max(InSquash, 1e-3);

        float rSq = InNormPos.x * InNormPos.x
                  + InNormPos.y * InNormPos.y
                  + squashedZ * squashedZ;

        float x2 = rSq / (a * a);
        if (x2 >= 1.0) { return 0.0; }

        return InPeak * PowSafe(1.0 - x2, max(InConcentration, GALAXY_POW_EPSILON));
    }

    // --- SEPARABLE DISC PROFILE, LENS CROSS-SECTION ---
    // Radial exp(-r/scaleLength) x vertical (1 - (z/h)^2)^falloff, with h tapering to
    // zero at the rim.
    //
    // This had THREE hard cuts, all of which read as a machined slab:
    //   RIM      exp(-r/scaleL) is still 0.135 of peak at r = discR (scaleL = 0.5R),
    //            then cut -- a vertical cylinder wall
    //   TOP/BOT  exp(-zNorm^v) is 0.368 at zNorm = 1, then cut -- a 37% cliff, and
    //            the larger of the two artefacts
    //   SHAPE    h did not depend on radius except through flare, so the vertical
    //            cross-section was a RECTANGLE rather than a lens
    //
    // Same failure the bulge and background had: an exponential that never reaches
    // zero, terminated by a boundary. Same fix -- compact support.
    //
    //   vertical  (1 - zNorm^2)^v reaches zero AT h with zero derivative, so the
    //             surfaces round off. Matches BulgeConcentration's semantics and
    //             drops an exp.
    //   taper     h *= sqrt(1 - rn^2), an ellipsoidal envelope. This is the one that
    //             kills "cylindrical" -- the cross-section becomes a lens. DiscFlare
    //             still thickens the mid-disc on top of it.
    //   radial    x (1 - rn^2), because with h -> 0 the plane z = 0 exactly would
    //             otherwise leave a zero-thickness sheet of nonzero density at the rim.
    float SampleDiscDensity(float rXY, float absZ)
    {
        if (DiscBaseDensity <= 0.0) { return 0.0; }

        float discR = max(DiscRadius, 1e-6);
        if (rXY >= discR) { return 0.0; }

        float rn = rXY / discR;
        float edge = 1.0 - rn * rn;             // 1 at the centre, 0 at the rim

        // --- SCALE HEIGHT: FLARE OUTWARD, THEN TAPER TO ZERO AT THE RIM ---
        float h = discR * max(DiscHeightRatio, 1e-6)
                * (1.0 + max(DiscFlare, 0.0) * rn)
                * sqrt(edge);

        if (absZ >= h) { return 0.0; }

        // --- SCALE LENGTH IS DERIVED FROM THE BULGE, NOT THE DISC RADIUS ---
        // These were two numbers describing one relationship. The bulge-to-disc scale
        // ratio is a real and well-measured quantity (R_e / h ~ 0.2-0.3), whereas
        // DiscRadius is just where the disc is truncated -- tying the brightness
        // profile to the truncation meant enlarging a galaxy also stretched its
        // profile, which is not what happens.
        //
        // MULTIPLICATIVE so the relation is scale-invariant: double the bulge and the
        // scale length follows, preserving the structure. An additive handle would
        // break as soon as overall galaxy size changed.
        //
        // Keeping it as a ratio rather than collapsing it entirely leaves a handle
        // for bulge-dominated vs disc-dominated galaxies, which is a real axis.
        float scaleL = max(BulgeScaleRadius, 1e-6) * max(DiscScaleLengthRatio, 1e-6);
        float radialProfile = exp(-rXY / scaleL) * edge;

        float zNorm = absZ / h;
        float vExp = max(DiscVerticalFalloff, 0.1);
        float verticalProfile = PowSafe(1.0 - zNorm * zNorm, vExp);

        return DiscBaseDensity * radialProfile * verticalProfile;
    }

    // --- EVALUATE ALL FOUR LAYERS INDEPENDENTLY ---
    // Returns (arm, disc, bulge, background). No composition, no bounds fade.
    //
    // Two noise frames: gas (arms + disc) and halo (bulge + background). Same
    // function, differing in twist and domain offset -- see SpiralTwistAt for why the
    // halo inherits a ramped fraction of the twist rather than none or all of it.
    float4 SampleLayers(float3 InNormPos)
    {
        float discR = DiscRadius;
        int oct = (int)clamp(NoiseOctaves, 1.0, (float)GALAXY_MAX_OCTAVES);

        // --- GAS POSITIONAL WARP ---
        // One fetch, not an FBm call: warp wants a single low frequency, and RGB
        // supplies a three-component vector directly. Arms and disc share the FETCH
        // but scale it independently, so they can warp by different amounts and in
        // opposite directions for the price of one lookup.
        // --- TWIST, ONCE PER SAMPLE ---
        // Shared by the warp and modulation frames of each family. The halo takes a
        // ramped fraction of the same field, so bulge and background pick up spiral
        // shear near the disc transition without inheriting the runaway winding that
        // the raw twist has toward the centre.
        bool bNoise = NoiseEnable > 0.5;

        float rXY0 = sqrt(InNormPos.x * InNormPos.x + InNormPos.y * InNormPos.y);
        float gasTwist  = bNoise ? SpiralTwistAt(rXY0, 1.0) : 0.0;
        float haloTwist = bNoise ? SpiralTwistAt(rXY0, HaloTwistInherit) : 0.0;

        float3 armPos  = InNormPos;
        float3 discPos = InNormPos;
        if (bNoise && (WarpAmountArms != 0.0 || WarpAmountDisc != 0.0))
        {
            float3 wf = NoiseFrame(InNormPos, gasTwist, WarpScale.xy, NoiseOffset);
            float3 wv = Texture3DSampleLevel(NoiseTex, NoiseTexSampler, wf, 0).rgb - 0.5;

            armPos  += wv * WarpAmountArms;
            discPos += wv * WarpAmountDisc;
        }

        // --- HALO POSITIONAL WARP ---
        // Taken from the ORIGINAL position, not a gas-warped one, so the two
        // displacements stay independent rather than compounding.
        float3 bulgePos = InNormPos;
        float3 bgPos    = InNormPos;
        if (bNoise && (WarpAmountBulge != 0.0 || WarpAmountBackground != 0.0))
        {
            float3 hf = NoiseFrame(InNormPos, haloTwist, WarpScale.zw,
                                   NoiseOffset + GALAXY_HALO_DOMAIN);
            float3 hv = Texture3DSampleLevel(NoiseTex, NoiseTexSampler, hf, 0).rgb - 0.5;

            bulgePos += hv * WarpAmountBulge;
            bgPos    += hv * WarpAmountBackground;
        }

        // --- GALAXY-SCALE m=1 MODES ---
        // Computed once from the UNWARPED position and shared by both gas layers.
        // These are properties of the galaxy, not of a layer: deriving them per layer
        // would let positional warp rotate the lopsided axis differently for arms and
        // disc, and would cost a second atan2 on the hot path.
        float rn0 = rXY0 / max(discR, 1e-6);
        float theta0 = atan2(InNormPos.y, InNormPos.x);

        // Integral-sign vertical warp. Grows as r^2 so the inner disc stays put --
        // the bulge holds the inner disc rigid, which is exactly why the bulge itself
        // does not warp: it is the plane the warp bends away from.
        //
        // STRICTLY m = 1. That is what makes it an integral sign, one side up and one
        // down. An m = 2 mode would give a saddle, which is rare and reads as a bent
        // potato rather than the recognisable S-curve.
        //
        // DiscWarpTwist precesses the LINE OF NODES with radius. Real warps are not
        // planar: the bending wave differentially precesses, so the rising node
        // rotates outward and the warp reads as a twisted ribbon rather than a bent
        // card. Zero reproduces the planar warp exactly. Physically this is the same
        // differential rotation that sets the arm pitch, so driving it from
        // SpiralTwist.x is a defensible coupling.
        float warpZ = DiscWarpAmplitude * rn0 * rn0
                    * sin(theta0 - DiscWarpPhase - DiscWarpTwist * rn0);
        armPos.z  -= warpZ;
        discPos.z -= warpZ;

        // Lopsidedness: one side of the disc is simply denser. Very common in real
        // galaxies, and one of the cheapest asymmetries available.
        //
        // RAMPED BY rn, like the warp is by rn^2. Without a ramp this is an
        // AZIMUTHAL SEAM ON THE AXIS: theta0 sweeps the full circle as r -> 0 while
        // the disc's radial profile is at its maximum there, so density jumps between
        // 1-A and 1+A across a single point. Currently invisible only because the
        // bulge is far denser at the centre and Compose takes a max. It is also the
        // more physical form -- the m=1 amplitude genuinely grows outward.
        float lopsided = max(1.0 + DiscLopsidedAmount * rn0
                                 * cos(theta0 - DiscLopsidedPhase), 0.0);

        // --- ARMS ---
        // Gated on ArmPeakDensity, which is how the other layers already early-out.
        // Zeroing a component of LayerDensity therefore both removes the layer and
        // skips its cost. The arm merge is the most expensive thing in the field, so
        // keeping this gate matters. The vertical gate lives inside SampleArmDensity,
        // where it is exact.
        float ArmDensity = 0.0;
        if (ArmPeakDensity > 0.0)
        {
            float rXYArm = sqrt(armPos.x * armPos.x + armPos.y * armPos.y);
            ArmDensity = SampleArmDensity(armPos, rXYArm) * lopsided;
        }

        // --- DISC ---
        float rXYDisc = sqrt(discPos.x * discPos.x + discPos.y * discPos.y);
        float DiscDensity = SampleDiscDensity(rXYDisc, abs(discPos.z)) * lopsided;

        // --- GAS MODULATION ---
        // What makes the field read as gas: real arms are chains of star forming
        // knots, not smooth ribbons. Sampled at the UNWARPED position -- one fetch
        // feeds two layers that are now warped differently, so picking either one's
        // warped position would have been arbitrary, and sampling both would double
        // the cost for no visible gain in a static field.
        //
        // Gated on there being something to modulate: most of the volume is empty, so
        // this skips the octave loop for the majority of march steps.
        // != 0.0, not > 0.0: a NEGATIVE amount is a valid inversion of the
        // modulation, and testing for positive silently dropped it whenever every
        // amount in the pair happened to be negative.
        if (bNoise && (NoiseAmountArm != 0.0 || NoiseAmountDisc != 0.0)
            && (ArmDensity + DiscDensity) > 1e-4)
        {
            float3 nf = NoiseFrame(InNormPos, gasTwist, NoiseScale.xy, NoiseOffset);
            float n = FBm(nf, oct);

            ArmDensity  *= max(1.0 + NoiseAmountArm  * n, 0.0);
            DiscDensity *= max(1.0 + NoiseAmountDisc * n, 0.0);
        }

        // --- BULGE AND BACKGROUND ---
        // Same function, different scale: a squashed spheroid with a density, a
        // radius and a concentration.
        float BulgeDensity = SampleSpheroid(bulgePos, BulgePeakDensity,
                                            BulgeScaleRadius, BulgeVerticalSquash,
                                            BulgeConcentration);

        float BgDensity = SampleSpheroid(bgPos, BackgroundDensity,
                                         BackgroundCutoffRadius, BackgroundVerticalSquash,
                                         BackgroundConcentration);

        // --- HALO MODULATION ---
        // Separate fetch in the isotropic frame, also at the unwarped position.
        // Leaving the bulge and halo as clean analytic functions while the arms and
        // disc are broken up reads as incongruous -- the smooth components give the
        // whole galaxy away.
        if (bNoise && (NoiseAmountBulge != 0.0 || NoiseAmountBackground != 0.0)
            && (BulgeDensity + BgDensity) > 1e-4)
        {
            float3 hf = NoiseFrame(InNormPos, haloTwist, NoiseScale.zw,
                                   NoiseOffset + GALAXY_HALO_DOMAIN);
            float nh = FBm(hf, oct);

            BulgeDensity *= max(1.0 + NoiseAmountBulge      * nh, 0.0);
            BgDensity    *= max(1.0 + NoiseAmountBackground * nh, 0.0);
        }

        // Layer order: arms, disc, bulge, background.
        return float4(ArmDensity, DiscDensity, BulgeDensity, BgDensity);
    }

    // --- CENTRAL VOID: DENSITY REMOVED BY THE CENTRAL BLACK HOLE ---
    // Returns a multiplier in [1 - VoidStrength, 1]. Deliberately spherical: the
    // cleared region is set by accretion, not by disc rotation, so it should not
    // inherit any layer's vertical squash.
    //
    // Compact support like SampleSpheroid, so it is exactly 1 outside VoidRadius and
    // costs nothing there. Because it lives in Compose it also applies to the CPU
    // particle path once this ports -- star systems should not spawn in the void
    // either, and that falls out for free rather than needing its own rejection test.
    float VoidFactor(float rBounds)
    {
        if (VoidStrength <= 0.0) { return 1.0; }

        float a = max(VoidRadius, 1e-6);
        float x2 = (rBounds * rBounds) / (a * a);
        if (x2 >= 1.0) { return 1.0; }

        return 1.0 - saturate(VoidStrength)
                   * PowSafe(1.0 - x2, max(VoidConcentration, GALAXY_POW_EPSILON));
    }

    // --- SPHERICAL BOUNDS FADE ---
    float BoundsFade(float rBounds)
    {
        if (rBounds <= BoundsFadeStart) { return 1.0; }
        float t = (rBounds - BoundsFadeStart) / (1.0 - BoundsFadeStart);
        return 1.0 - t * t * (3.0 - 2.0 * t);
    }

    // --- COMPOSE LAYERS ---
    // Union by max, per your local edit. The central void is applied here rather than
    // per layer: it is a property of the galaxy, so carving it once after the union
    // costs one multiply instead of four.
    float Compose(float4 L, float rBounds)
    {
        float Density = max(max(max(L.x, L.y), L.z), L.w);

        Density *= VoidFactor(rBounds);
        Density *= BoundsFade(rBounds);

        // NOT saturated: raw density out while the layers are still being tuned in
        // isolation. Clamping belongs in the compositing pass, once that exists.
        return Density;
    }

    float Sample(float3 InNormPos)
    {
        float rBounds = length(InNormPos);
        if (rBounds >= 1.0) { return 0.0; }

        return Compose(SampleLayers(InNormPos), rBounds);
    }

    // --- RAYMARCH: ANALYTIC SPHERE-BOUNDED DENSITY ACCUMULATION ---
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
    // InNoisePower   - render-side shaping, no CPU counterpart
    //
    // Layer isolation is done by zeroing a component of LayerDensity: each layer
    // guards on its own density, so that both removes it and skips its cost.
    //
    // Returns emissive in RGB, transmittance in A.
    float4 RayMarch(float3 InStartPos, float3 InViewVec, int InMaxSteps,
                    float InDensityScale, float InJitter, float InNoisePower)
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

        for (int i = 0; i < steps; i++)
        {
            float3 normPos = o + dir * t;

            float density = Sample(normPos);
            density = pow(max(density, GALAXY_POW_EPSILON), InNoisePower);

            // --- OPACITY ---
            float sigma = density * InDensityScale;
            float alpha = 1.0 - exp(-sigma * localStep);

            // --- ACCUMULATE: FLAT WHITE EMISSION, STRUCTURE ONLY ---
            volumeColor += alpha * transmittance;
            transmittance *= 1.0 - alpha;

            if (transmittance < 0.001) { break; }

            t += normStep;
        }

        return float4(volumeColor, transmittance);
    }
};

// --- SETUP ---
GalaxyDensitySampler gd;
gd.BoundsFadeStart           = BoundsFadeStart;

gd.BulgeConcentration        = BulgeConcentration;

gd.DiscScaleLengthRatio      = DiscRadialScaleLength;
gd.DiscVerticalFalloff       = DiscVerticalFalloff;
gd.DiscFlare                 = DiscFlare;
gd.DiscWarpAmplitude         = DiscWarpAmplitude;
gd.DiscWarpPhase             = DiscWarpPhase;
gd.DiscWarpTwist             = DiscWarpTwist;
gd.DiscLopsidedAmount        = DiscLopsidedAmount;
gd.DiscLopsidedPhase         = DiscLopsidedPhase;

gd.ArmCount                  = ArmCount;
// x = pitch angle (deg), y = tightening, z = phase offset (rad),
// w = how much of the twist the halo frame inherits
gd.ArmPitchAngle             = SpiralTwist.x;
gd.ArmPitchTightening        = SpiralTwist.y;
gd.ArmPhaseOffset            = SpiralTwist.z;
gd.HaloTwistInherit          = SpiralTwist.w;
gd.ArmProfileExponent        = ArmProfileExponent;
gd.ArmMergeSmooth            = ArmMergeSmooth;

gd.NoiseOffset               = NoiseOffset;
// FRAME CONVENTION: x = disc lateral, y = disc vertical,
//                   z = halo lateral, w = halo vertical
gd.NoiseScale                = NoiseScale;
gd.WarpScale                 = WarpScale;
gd.NoiseOctaves              = NoiseOctaves;
gd.NoiseRidged               = NoiseRidged;
gd.NoiseEnable               = NoiseEnable;
gd.NoiseChannelWeights       = NoiseChannelWeights;

// --- PER-LAYER FAMILIES, PACKED x = arms, y = disc, z = bulge, w = background ---
//
// Every layer is a body with a horizontal scale S and a vertical extent S * V.
// The four vertical controls are all AXIS RATIOS despite looking different: the
// bulge and background write it as z/squash, which puts the constant-density
// surface at z = squash * a, so the parameter is c/a exactly as the disc's h/R and
// the arm's H/W are. Smaller flattens, in all four.
//
// NOTE .y does double duty: DiscRadius is also the arm system's radial reference
// (the spiral u = log(discR/rXY), tRadial, and the outer bound all read it). Arms
// deliberately have no independent radius -- they live in the disc.
gd.ArmWidth                  = LateralScale.x;
gd.DiscRadius                = LateralScale.y;
gd.BulgeScaleRadius          = LateralScale.z;
gd.BackgroundCutoffRadius    = LateralScale.w;

gd.ArmVerticalRatio          = VerticalScale.x;
gd.DiscHeightRatio           = VerticalScale.y;
gd.BulgeVerticalSquash       = VerticalScale.z;
gd.BackgroundVerticalSquash  = VerticalScale.w;

gd.NoiseAmountArm            = NoiseAmount.x;
gd.NoiseAmountDisc           = NoiseAmount.y;
gd.NoiseAmountBulge          = NoiseAmount.z;
gd.NoiseAmountBackground     = NoiseAmount.w;

gd.ArmPeakDensity            = LayerDensity.x;
gd.DiscBaseDensity           = LayerDensity.y;
gd.BulgePeakDensity          = LayerDensity.z;
gd.BackgroundDensity         = LayerDensity.w;

// ArmAsymSeed stays a separate pin: it is an index, not an amount.
gd.ArmAsymPitch              = ArmAsym.x;
gd.ArmAsymPhase              = ArmAsym.y;
gd.ArmAsymDensity            = ArmAsym.z;
gd.ArmAsymLength             = ArmAsym.w;

gd.WarpAmountArms            = WarpAmount.x;
gd.WarpAmountDisc            = WarpAmount.y;
gd.WarpAmountBulge           = WarpAmount.z;
gd.WarpAmountBackground      = WarpAmount.w;

// x = radius, y = strength, z = concentration
gd.VoidRadius                = CentralVoid.x;
gd.VoidStrength              = CentralVoid.y;
gd.VoidConcentration         = CentralVoid.z;

gd.NoiseTex                  = NoiseTex;
gd.NoiseTexSampler           = NoiseTexSampler;
gd.ArmRadialGrowth           = ArmRadialGrowth;
gd.ArmDensityFalloffExponent = ArmDensityFalloffExponent;
gd.ArmAsymSeed               = ArmAsymSeed;

gd.BackgroundConcentration   = BackgroundConcentration;


// --- HOIST PER-ARM CONSTANTS: ONCE PER PIXEL, NOT ONCE PER MARCH STEP ---
// Every slot is filled, not just the first ArmN, so none is ever read
// uninitialised. Cost is a handful of hashes once, against N x MaxSteps before.
gd.ArmN = (int)clamp(ArmCount, 1.0, (float)GALAXY_MAX_ARMS);

float armSpacing = 2.0 * GALAXY_PI / float(gd.ArmN);
int armSeed = (int)ArmAsymSeed;

for (int ai = 0; ai < GALAXY_MAX_ARMS; ai++)
{
    float4 h = gd.ArmHash(ai, armSeed);

    gd.ArmData[ai] = float4(
        1.0 + gd.ArmAsymPitch * (2.0 * h.x - 1.0),
        gd.ArmPhaseOffset + float(ai) * armSpacing
            + gd.ArmAsymPhase * armSpacing * (2.0 * h.y - 1.0),
        max(1.0 + gd.ArmAsymDensity * (2.0 * h.z - 1.0), 0.0),
        gd.DiscRadius * (1.0 - gd.ArmAsymLength * h.w));
}

// --- DITHER: ENTRY-POINT OFFSET, RESOLVES THROUGH TEMPORAL AA ---
int3 randpos = int3(Parameters.SvPosition.xy, View.StateFrameIndexMod8);
float rand = float(Rand3DPCG16(randpos).x) / 0xffff;

return gd.RayMarch(
    CurPos,
    LocalCamVec,      // raw, not normalized -- the method handles it
    (int)MaxSteps,
    MasterDensityScale,   // InDensityScale
    rand,                 // InJitter; pass 0.0 to disable dither
    MasterDensityPower    // InNoisePower
);