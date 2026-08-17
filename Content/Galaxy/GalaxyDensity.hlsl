// GalaxyDensity.ush
// Procedural galaxy density field. Mirrored by GalaxyDataGenerator::SampleDensity
// on the C++ side, which rejection-samples it for star placement.
//
// PARITY RULES -- any edit must be made on both sides together:
//   - Guard every pow() base away from zero. HLSL evaluates pow(x,y) as
//     exp2(y*log2(x)); log2(0) is -INF and some compilers yield NaN.
//   - Bound anything that can overflow float32. exp overflows near 88 on GPU
//     against 709 for double on CPU, so the GPU fails first and over a wider region.
//   - No branch that exists on only one side.
//
// SPACE: normalized galaxy space, [-1,1] per axis, where 1.0 is Params.Extent.
// From a unit-box proxy whose CurPos runs [0,1]:  NormPos = 2.0 * (CurPos - 0.5)
//
// LAYER CONVENTION: x = arms, y = disc, z = bulge, w = background. SampleLayers'
// return and every per-layer float4 pin follow it.
//
// FRAME CONVENTION, for NoiseScale and WarpScale ONLY: x = disc lateral,
// y = disc vertical, z = halo lateral, w = halo vertical. Distinct from the layer
// convention above -- frames are a property of the noise field, layers consume it.
//
// UNIFORM HOISTING: every struct field arrives PRE-CLAMPED and, where a division by
// it would otherwise recur, pre-inverted. Do not re-clamp inside the sample
// functions -- those run per march step, and the guards sit behind early-returns so
// the compiler cannot hoist them speculatively. Anything named Inv* or *OverPath, and
// all of ArmSpiralK0, ArmPitchFactorMax and the *Sq fields, is derived in the driver.

#define GALAXY_PI 3.14159265358979323846
#define GALAXY_POW_EPSILON 1e-6
#define GALAXY_MAX_ARMS 16
#define GALAXY_MAX_OCTAVES 2

// Each octave doubles frequency and halves amplitude. Constants because a pre-baked
// FBm asset already carries the spectrum, so at one or two octaves nothing else was
// ever worth reaching for.
#define GALAXY_NOISE_LACUNARITY 2.0
#define GALAXY_NOISE_GAIN 0.5

// Separates the halo noise domain from the gas one so their structure does not
// correlate. Arbitrary; only needs to be large against feature size.
#define GALAXY_HALO_DOMAIN float3(53.7, 29.1, 71.3)

struct GalaxyDensitySampler
{
    // Authored tiling asset, not generated at runtime. That distinction is what
    // keeps CPU parity reachable: fixed data can be exported once and sampled in
    // C++, where a per-galaxy generated texture could not be.
    Texture3D    NoiseTex;
    SamplerState NoiseTexSampler;

    float BoundsFadeStart;
    float BoundsFadeStartSq;
    float InvBoundsFadeSpan;
    int   NoiseOctaveCount;

    // --- BULGE ---
    float BulgeInvRadiusSq;         // 1 / r^2 at which the layer reaches exactly zero
    float BulgeDensityOverPath;     // optical depth through the centre / path length
    float BulgeInvSquash;           // 1 / (c/a axis ratio); larger flattens
    float BulgeConcentration;       // profile exponent; see SampleSpheroid

    // --- DISC ---
    float DiscRadius;
    float InvDiscRadius;
    float DiscHeightRatio;          // h/R at the centre
    float DiscDensityOverPath;      // face-on optical depth / (half the profile integral)
    float InvDiscScaleL;            // 1 / radial scale length
    float DiscVerticalFalloff;      // profile exponent, same meaning as concentration
    float DiscFlare;                // 0 = constant scale height; >0 thickens outward
    float DiscWarpAmplitude;        // m=1 vertical warp, normalized z units
    float DiscWarpPhase;            // radians; azimuth of the rising node at r = 0
    float DiscWarpTwist;            // radians of node-line precession across the disc
    float DiscLopsidedAmount;       // m=1 in-plane density mode; 0.2-0.4 typical
    float DiscLopsidedPhase;        // radians; azimuth of the dense side

    // --- ARMS ---
    float ArmSpiralK0;              // signed 1/tan(pitch); sign sets chirality
    float ArmPitchFactorMax;        // 1 + |ArmAsymPitch|; bounds the angular reject
    float ArmPitchTightening;       // 0 = constant pitch; >0 tightens inward
    float HaloTwistInherit;         // fraction of the spiral twist the halo frame takes
    float ArmWidth;                 // true perpendicular half-width at the inner end
    float InvArmVerticalRatio;      // 1 / (H/W). H/W of 1 is a circular tube
    float ArmProfileExponent;       // cross-section falloff. <1 plateaus, >1 sharpens
    float ArmRadialGrowth;          // width multiplier at the rim
    float ArmHostFalloff;           // how strongly arms inherit the disc radial profile
    float ArmDensityOverPath;       // perpendicular optical depth / path length
    float InvArmFadeW;              // 1 / width of the per-arm end fade
    float ArmMergeSmooth;           // FRACTION of the larger arm, not absolute density
    int    ArmN;
    float4 ArmData[GALAXY_MAX_ARMS]; // x pitch factor, y phase, z strength,
                                     // w radius at which the end fade STARTS

    // --- BACKGROUND ---
    float BackgroundInvRadiusSq;
    float BackgroundDensityOverPath;
    float BackgroundInvSquash;
    float BackgroundConcentration;

    // --- NOISE FIELD ---
    float3 NoiseOffset;             // per-galaxy variation; a texture has no seed
    float4 NoiseScale;              // modulation frequency, frame convention
    float4 WarpScale;               // warp frequency, frame convention
    float NoiseRidged;              // 0 = fbm, 1 = ridged filaments and lanes
    float InvNoiseNorm;             // 1 / sum of octave amplitudes
    float NoiseEnable;              // 0 skips every texture read; see SampleLayers
    float4 NoiseChannelWeights;     // signed channel mix; see SampleNoiseTex

    // Multiplicative depth per layer: density *= 1 + Amount * n. Above 1 the
    // modulator goes negative in places, clamps to zero, and breaks the layer apart.
    float NoiseAmountArm;
    float NoiseAmountDisc;
    float NoiseAmountBulge;
    float NoiseAmountBackground;

    // Positional warp per layer, normalized units. Signed: negative flips the
    // displacement direction, and the sampled vector is already centred on zero.
    float WarpAmountArms;
    float WarpAmountDisc;
    float WarpAmountBulge;
    float WarpAmountBackground;

    // --- CENTRAL VOID ---
    float InvVoidRadiusSq;
    float VoidStrength;             // 0 = off, 1 = fully empty at the centre
    float VoidConcentration;

    // pow() with the zero base branched out rather than clamped: max(x, EPSILON)
    // still leaves EPSILON^p behind, which is enough to floor a field.
    float PowSafe(float x, float p)
    {
        return (x > GALAXY_POW_EPSILON) ? pow(x, p) : 0.0;
    }

    // Integral of (1 - x^2)^c over [-1,1]. Exactly B(1/2, c+1); this closed form is
    // within 1.4% at c = 0.25 and under 0.5% above c = 1. Driver-only.
    float CompactProfileIntegral(float InConcentration)
    {
        return sqrt(GALAXY_PI / (max(InConcentration, 0.0) + 0.75));
    }

    // Integer hash, one float4 per arm index. uint arithmetic wraps identically in
    // C++ and HLSL, so this is bit-exact across both. Driver-only.
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
    // SampleLevel, not Sample: inside a dynamic loop the derivatives driving mip
    // selection are undefined.
    //
    // Channels are centred to [-1,1] BEFORE weighting, which is what lets a weight
    // act as a mixer: positive takes the channel, negative inverts it, |w| > 1 is
    // gain, all-zero returns 0. Weighting first would need a signed-sum divide that
    // blows up for (1,-1,0,0).
    //
    // Output is deliberately NOT bounded to [-1,1] -- sum|w| > 1 exceeds it. The
    // downstream modulator is 1 + Amount * n, so large |n| spikes density as well as
    // clamping it to zero. Keep sum|w| near 1 unless gain is wanted.
    //
    // Ridging happens HERE, per channel, not on the weighted sum: the 1-2|c| fold
    // only maps back into [-1,1] when its input is already bounded there, which each
    // centred channel is and the weighted sum is not.
    float SampleNoiseTex(float3 P)
    {
        float4 t = Texture3DSampleLevel(NoiseTex, NoiseTexSampler, P, 0);
        float4 c = t * 2.0 - 1.0;

        c = lerp(c, 1.0 - 2.0 * abs(c), NoiseRidged);

        return dot(NoiseChannelWeights, c);
    }

    float FBm(float3 P)
    {
        float sum = 0.0;
        float amp = 1.0;

        for (int o = 0; o < GALAXY_MAX_OCTAVES; o++)
        {
            if (o >= NoiseOctaveCount) { break; }

            // A texture has no seed, so octaves decorrelate by offset.
            sum += SampleNoiseTex(P + float3(17.3, 11.7, 23.1) * float(o)) * amp;
            P *= GALAXY_NOISE_LACUNARITY;
            amp *= GALAXY_NOISE_GAIN;
        }

        // The amplitude sum depends only on the octave count, so it is inverted once.
        return sum * InvNoiseNorm;
    }

    // --- SPIRAL TWIST OF THE NOISE FRAME ---
    // Rotating a sample by -twist before evaluating noise makes features wind with
    // the spiral and shear inward. InInherit scales it; 1 is the full gas twist.
    //
    // The r/R ramp is load bearing. Raw twist grows as ln(R/r), reaching 11 radians
    // at r = 0.05R, which would smear halo noise into azimuthal streaks. Ramping
    // turns the product into a bump peaking near 0.37R at about 1.4 rad -- at the
    // bulge/disc transition, which is also where the shared kinematics are.
    //
    // InU is min(-log(rn), 6) and InRn is rXY/discR, both from the caller: the gas
    // and halo twists share a radius, so deriving them here would repeat the log.
    float SpiralTwistAt(float InU, float InRn, float InInherit)
    {
        if (InInherit == 0.0) { return 0.0; }

        float T = ArmPitchTightening;
        return ArmSpiralK0 * (InU + 0.5 * T * InU * InU) * InRn * InInherit;
    }

    // InScale is (lateral, vertical). Vertical above lateral compresses z, so
    // features read as sheets. InOffset separates the two frames' domains.
    float3 NoiseFrame(float3 InPos, float InTwist, float2 InScale, float3 InOffset)
    {
        float st, ct;
        sincos(InTwist, st, ct);

        return float3((InPos.x * ct + InPos.y * st) * InScale.x,
                     (-InPos.x * st + InPos.y * ct) * InScale.x,
                       InPos.z * InScale.y) + InOffset;
    }

    // --- POLYNOMIAL SMOOTH MAXIMUM, RELATIVE BAND ---
    // K is a FRACTION of the larger operand, not an absolute density. Do not make it
    // absolute: the blend band would then be as wide at zero as at peak, so
    // smax(0,0) returns K/4 out of nothing, and that floor compounds across the arm
    // loop's merges into visible background haze.
    //
    // K must also stay <= 1, which the driver enforces. Above 1 the band is wider
    // than the value and smax(a, 0) no longer returns a -- merging a
    // zero-contribution arm would inflate the accumulator by up to 56% at K = 4.
    float SmoothMaxPoly(float A, float B, float K)
    {
        float m = max(A, B);
        float k = K * m;
        if (k <= 0.0) { return m; }

        float h = saturate(0.5 + 0.5 * (A - B) / k);
        return lerp(B, A, h) + k * h * (1.0 - h);
    }

    // Disc radial profile, shared with the arms. Truncated by edge = 1 - rn^2 so it
    // reaches zero at the rim rather than being cut there. The caller passes edge
    // because both call sites already have it.
    float DiscRadialProfile(float rXY, float edge)
    {
        return exp(-rXY * InvDiscScaleL) * edge;
    }

    // --- COMBINED ARM DENSITY ---
    // Every arm contributes and the contributions are merged, rather than a
    // nearest-arm search. Do not revert to nearest-arm: the distance it returns is
    // continuous but the WINNER'S attributes flip instantly where two arms are
    // equidistant, so any per-arm strength variation produces a hard density step.
    //
    // Affordable per arm because the closest point lies at the SAME RADIUS as the
    // query, so the in-plane distance is the chord subtending the angular offset:
    // 2 r sin(d/2). One sin, no cos or sqrt.
    //
    // Cross-section is a normalized ellipse: perpendicular and vertical offsets are
    // each divided by their own half-extent and combined into one coordinate q that
    // reaches 1 at the surface. One profile on one metric is what makes the arm
    // tubular -- two separate profiles cannot be round even at matching extents.
    float SampleArmDensity(float3 InNormPos, float rXY)
    {
        if (rXY < 1e-6 || rXY >= DiscRadius) { return 0.0; }

        // The guard above bounds rXY to (0, DiscRadius), so no saturate is needed.
        float tRadial = rXY * InvDiscRadius;
        float growthFactor = lerp(1.0, ArmRadialGrowth, tRadial);
        float invW = 1.0 / max(ArmWidth * growthFactor, 1e-6);

        // Exact vertical gate: q >= qz always and the surface is q = 1, so qz >= 1
        // proves every arm is zero here. Skips the merge loop for the large majority
        // of samples, which sit off the disc plane. H = W * ratio, so its reciprocal
        // comes from invW rather than a second divide.
        float qz = abs(InNormPos.z) * invW * InvArmVerticalRatio;
        if (qz >= 1.0) { return 0.0; }
        float qzSq = qz * qz;

        // Arms are a compression wave in the disc, so their strength follows the disc
        // hosting them. Evaluated before the loop so a negligible host skips it.
        float edge = 1.0 - tRadial * tRadial;
        float host = PowSafe(DiscRadialProfile(rXY, edge), ArmHostFalloff);
        if (host <= 0.0) { return 0.0; }

        // Logarithmic spiral: constant pitch p means tan(p) = dr/(r dTheta), which
        // integrates to Theta = ln(r/R)/tan(p). Winding rate grows as 1/r inward, so
        // the spiral tightens by construction and ln() is self-bounding.
        // ArmPitchTightening adds k(u) = k0(1 + Tu), so Theta = k0(u + Tu^2/2).
        // -log(tRadial) is log(discR/rXY) without a second divide.
        float u = -log(max(tRadial, 1e-5));
        float T = ArmPitchTightening;
        float uTerm = u + 0.5 * T * u * u;
        float uRate = 1.0 + T * u;

        float theta = atan2(InNormPos.y, InNormPos.x);

        const float TwoPi = 2.0 * GALAXY_PI;
        const float InvTwoPi = 1.0 / TwoPi;

        // --- ANGULAR EARLY REJECT ---
        // dPerp = 2 r |sin(d/2)| * corr, and corr = rsqrt(1 + kLocal^2) is at least
        // corrMin, taken at the largest kLocal any arm can have here. So an arm
        // whose |d| exceeds this threshold cannot reach within W and contributes
        // nothing. One asin per sample replaces the sin, rsqrt, sqrt and merge for
        // the majority of arms, which at typical arm widths is most of them.
        float kMax = abs(ArmSpiralK0) * ArmPitchFactorMax * uRate;
        float corrMin = rsqrt(1.0 + kMax * kMax);
        float sinHalf = 1.0 / max(2.0 * rXY * corrMin * invW, 1e-6);
        float dReject = (sinHalf >= 1.0) ? GALAXY_PI : 2.0 * asin(sinHalf);

        float acc = 0.0;

        for (int i = 0; i < GALAXY_MAX_ARMS; i++)
        {
            if (i >= ArmN) { break; }

            float4 a = ArmData[i];
            float ki = ArmSpiralK0 * a.x;

            // Signed angular offset, wrapped to [-PI,PI]. Wrapping to +/-PI rather
            // than +/-armSpacing/2 is what makes uneven arm spacing safe.
            float dRaw = theta - (ki * uTerm + a.y);
            float d = dRaw - TwoPi * round(dRaw * InvTwoPi);

            if (abs(d) >= dReject) { continue; }

            // The chord is purely tangential; the spiral tangent makes angle p with
            // it, so perpendicular distance is chord / sqrt(1+k^2). Unconditional, so
            // ArmWidth is a true perpendicular half-width and does not shift when
            // ArmPitchAngle changes.
            float kLocal = ki * uRate;
            float dPerp = 2.0 * rXY * abs(sin(0.5 * d)) * rsqrt(1.0 + kLocal * kLocal);

            float qp = dPerp * invW;
            float q = sqrt(qp * qp + qzSq);
            if (q >= 1.0) { continue; }

            float w = 1.0 - q * q * (3.0 - 2.0 * q);

            // a.w is the radius at which the end fade STARTS, folded in the driver.
            float mult = a.z;
            if (rXY > a.w)
            {
                float tf = saturate((rXY - a.w) * InvArmFadeW);
                mult *= 1.0 - tf * tf * (3.0 - 2.0 * tf);
            }

            acc = SmoothMaxPoly(acc, w * max(mult, 0.0), ArmMergeSmooth);
        }

        // Shape applied once to the merged result, so it stays a single pow.
        acc = PowSafe(acc, ArmProfileExponent);

        return acc * ArmDensityOverPath * host;
    }

    // --- COMPACT SQUASHED SPHEROID, SHARED BY BULGE AND BACKGROUND ---
    //     d = InDensityOverPath * (1 - x^2)^p,   x^2 = r_oblate^2 * InInvRadiusSq
    //
    // Compact support: exactly zero at x >= 1, so the layer terminates itself and the
    // test is a free early-out. Works on r^2, so no sqrt.
    //
    // p < 1 fills the volume and drops at the rim; 1 parabolic; 2 smooth bump;
    // >2 concentrates toward the centre.
    //
    // Do not swap this for a cusped profile such as Hernquist. It diverges at the
    // centre, needs a clamp to stay finite, and the clamp turns the peak parameter
    // into a control over the radius of a saturated ball instead of a density.
    float SampleSpheroid(float3 InNormPos, float InDensityOverPath, float InInvRadiusSq,
                         float InInvSquash, float InConcentration)
    {
        if (InDensityOverPath <= 0.0) { return 0.0; }

        float squashedZ = InNormPos.z * InInvSquash;

        float rSq = InNormPos.x * InNormPos.x
                  + InNormPos.y * InNormPos.y
                  + squashedZ * squashedZ;

        float x2 = rSq * InInvRadiusSq;
        if (x2 >= 1.0) { return 0.0; }

        return InDensityOverPath * PowSafe(1.0 - x2, InConcentration);
    }

    // --- DISC, LENS CROSS-SECTION ---
    // Radial exp(-r/scaleLength) x vertical (1 - (z/h)^2)^falloff, with h tapering to
    // zero at the rim.
    //
    // All three terms are compact for a reason. An exponential vertical profile sits
    // at 0.368 of peak where it is cut, and the radial one at 0.135, so both produce
    // cliffs; and without the sqrt(1-rn^2) taper the cross-section is a rectangle,
    // which reads as a machined cylinder rather than a disc.
    //
    // DiscDensityOverPath divides by the LOCAL h, which is correct: volume density is
    // surface density over thickness, so where the disc thins the volume density
    // rises. The h cancels from the face-on integral exactly. At the rim h goes as
    // sqrt(edge) against radial's edge, so density falls as sqrt(edge) rather than
    // diverging.
    float SampleDiscDensity(float rXY, float absZ)
    {
        if (DiscDensityOverPath <= 0.0) { return 0.0; }

        if (rXY >= DiscRadius) { return 0.0; }

        float rn = rXY * InvDiscRadius;
        float edge = 1.0 - rn * rn;

        float h = DiscRadius * DiscHeightRatio * (1.0 + DiscFlare * rn) * sqrt(edge);
        if (absZ >= h) { return 0.0; }

        // One reciprocal: h divides both the vertical coordinate and the density.
        float invH = 1.0 / h;

        float zNorm = absZ * invH;
        float verticalProfile = PowSafe(1.0 - zNorm * zNorm, DiscVerticalFalloff);

        return DiscDensityOverPath * invH * DiscRadialProfile(rXY, edge) * verticalProfile;
    }

    // Returns (arm, disc, bulge, background). No composition, no bounds fade.
    //
    // Two noise frames: gas (arms + disc) and halo (bulge + background), differing in
    // twist and domain offset. The halo must not take the full twist -- see
    // SpiralTwistAt for why it is ramped.
    float4 SampleLayers(float3 InNormPos)
    {
        bool bNoise = NoiseEnable > 0.5;

        // Split so neither twist is built when nothing consumes it.
        bool bGasNoise  = bNoise && (WarpAmountArms != 0.0 || WarpAmountDisc != 0.0
                                  || NoiseAmountArm != 0.0 || NoiseAmountDisc != 0.0);
        bool bHaloNoise = bNoise && (WarpAmountBulge != 0.0 || WarpAmountBackground != 0.0
                                  || NoiseAmountBulge != 0.0 || NoiseAmountBackground != 0.0);

        float rXY0 = sqrt(InNormPos.x * InNormPos.x + InNormPos.y * InNormPos.y);

        // One log for both twists: they share this radius.
        float gasTwist  = 0.0;
        float haloTwist = 0.0;
        if (bGasNoise || bHaloNoise)
        {
            float rn0t = saturate(rXY0 * InvDiscRadius);
            float u0 = min(-log(max(rn0t, 1e-5)), 6.0);

            gasTwist  = bGasNoise  ? SpiralTwistAt(u0, rn0t, 1.0) : 0.0;
            haloTwist = bHaloNoise ? SpiralTwistAt(u0, rn0t, HaloTwistInherit) : 0.0;
        }

        // One fetch, not an FBm call: warp wants a single low frequency, and RGB
        // supplies a three-component vector directly. Arms and disc share the fetch
        // but scale it independently, so they can warp in opposite directions for the
        // price of one lookup.
        float3 armPos  = InNormPos;
        float3 discPos = InNormPos;
        if (bNoise && (WarpAmountArms != 0.0 || WarpAmountDisc != 0.0))
        {
            float3 wf = NoiseFrame(InNormPos, gasTwist, WarpScale.xy, NoiseOffset);
            float3 wv = Texture3DSampleLevel(NoiseTex, NoiseTexSampler, wf, 0).rgb - 0.5;

            armPos  += wv * WarpAmountArms;
            discPos += wv * WarpAmountDisc;
        }

        // From the ORIGINAL position, not a gas-warped one, so the two displacements
        // stay independent rather than compounding.
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
        // Derived once from the UNWARPED position and shared by both gas layers:
        // these belong to the galaxy, not a layer, and per-layer derivation would let
        // positional warp rotate the lopsided axis differently for arms and disc.
        //
        // Both ramp with radius. The warp goes as rn^2 because the bulge holds the
        // inner disc rigid -- which is also why the bulge itself does not warp, being
        // the plane the warp bends away from. Lopsidedness ramps as rn to avoid an
        // azimuthal seam on the axis, where theta sweeps the full circle while the
        // disc profile is at its maximum.
        float warpZ = 0.0;
        float lopsided = 1.0;
        if (DiscWarpAmplitude != 0.0 || DiscLopsidedAmount != 0.0)
        {
            float theta0 = atan2(InNormPos.y, InNormPos.x);
            float rn0 = saturate(rXY0 * InvDiscRadius);

            warpZ = DiscWarpAmplitude * rn0 * rn0
                  * sin(theta0 - DiscWarpPhase - DiscWarpTwist * rn0);

            lopsided = max(1.0 + DiscLopsidedAmount * rn0
                               * cos(theta0 - DiscLopsidedPhase), 0.0);

            armPos.z  -= warpZ;
            discPos.z -= warpZ;
        }

        // Gated on its own density, as every layer is: zeroing a LayerDensity
        // component both removes the layer and skips its cost.
        float ArmDensity = 0.0;
        if (ArmDensityOverPath > 0.0)
        {
            float rXYArm = sqrt(armPos.x * armPos.x + armPos.y * armPos.y);
            ArmDensity = SampleArmDensity(armPos, rXYArm) * lopsided;
        }

        float rXYDisc = sqrt(discPos.x * discPos.x + discPos.y * discPos.y);
        float DiscDensity = SampleDiscDensity(rXYDisc, abs(discPos.z)) * lopsided;

        // Sampled at the UNWARPED position: one fetch feeds two layers that are
        // warped differently, so either warped position would be an arbitrary choice
        // and sampling both would double the cost for no visible gain.
        //
        // != 0.0 rather than > 0.0, so a negative amount -- a valid inversion of the
        // modulation -- is not silently dropped.
        if (bNoise && (NoiseAmountArm != 0.0 || NoiseAmountDisc != 0.0)
            && (ArmDensity + DiscDensity) > 1e-4)
        {
            float3 nf = NoiseFrame(InNormPos, gasTwist, NoiseScale.xy, NoiseOffset);
            float n = FBm(nf);

            ArmDensity  *= max(1.0 + NoiseAmountArm  * n, 0.0);
            DiscDensity *= max(1.0 + NoiseAmountDisc * n, 0.0);
        }

        float BulgeDensity = SampleSpheroid(bulgePos, BulgeDensityOverPath,
                                            BulgeInvRadiusSq, BulgeInvSquash,
                                            BulgeConcentration);

        float BgDensity = SampleSpheroid(bgPos, BackgroundDensityOverPath,
                                         BackgroundInvRadiusSq, BackgroundInvSquash,
                                         BackgroundConcentration);

        if (bNoise && (NoiseAmountBulge != 0.0 || NoiseAmountBackground != 0.0)
            && (BulgeDensity + BgDensity) > 1e-4)
        {
            float3 hf = NoiseFrame(InNormPos, haloTwist, NoiseScale.zw,
                                   NoiseOffset + GALAXY_HALO_DOMAIN);
            float nh = FBm(hf);

            BulgeDensity *= max(1.0 + NoiseAmountBulge      * nh, 0.0);
            BgDensity    *= max(1.0 + NoiseAmountBackground * nh, 0.0);
        }

        return float4(ArmDensity, DiscDensity, BulgeDensity, BgDensity);
    }

    // Region cleared by the central black hole. Returns a multiplier in
    // [1 - VoidStrength, 1]. Spherical rather than squashed: what accretion clears is
    // not set by disc rotation.
    float VoidFactor(float rBoundsSq)
    {
        if (VoidStrength <= 0.0) { return 1.0; }

        float x2 = rBoundsSq * InvVoidRadiusSq;
        if (x2 >= 1.0) { return 1.0; }

        return 1.0 - VoidStrength * PowSafe(1.0 - x2, VoidConcentration);
    }

    float BoundsFade(float rBoundsSq)
    {
        if (rBoundsSq <= BoundsFadeStartSq) { return 1.0; }

        float t = (sqrt(rBoundsSq) - BoundsFadeStart) * InvBoundsFadeSpan;
        return 1.0 - t * t * (3.0 - 2.0 * t);
    }

    // Density is additive: arms are an overdensity riding on the disc, and the bulge
    // and disc occupy the same space. Do not use max() here -- it makes the layers
    // compete, so the disc vanishes inside the arms and every authored value inflates
    // to out-bid its neighbours.
    //
    // Not clamped. The marcher consumes this as sigma and wants the full range; the
    // CPU rejection sampler maps it to a probability with its own response curve.
    float Compose(float4 L, float rBoundsSq)
    {
        float Density = L.x + L.y + L.z + L.w;

        Density *= VoidFactor(rBoundsSq);
        Density *= BoundsFade(rBoundsSq);

        return Density;
    }

    float Sample(float3 InNormPos)
    {
        // Squared, so out-of-bounds samples never pay for the sqrt.
        float rBoundsSq = dot(InNormPos, InNormPos);
        if (rBoundsSq >= 1.0) { return 0.0; }

        return Compose(SampleLayers(InNormPos), rBoundsSq);
    }

    // --- RAYMARCH: ANALYTIC SPHERE-BOUNDED DENSITY ACCUMULATION ---
    // The proxy mesh decides which pixels run; the unit sphere decides where to
    // march. Step length adapts to chord length, so quality does not vary with view
    // angle and no trailing partial step is needed.
    //
    // Do not add a box bounds test on the marched position. InStartPos sits exactly
    // on the proxy surface, so a component is exactly 0 or 1 before any FP error, and
    // such a test breaks on iteration zero wherever rounding pushes it outside --
    // view-direction dependent, so it shows as angle-dependent dropout at silhouettes.
    //
    // InViewVec is the LOCAL camera vector, surface toward camera, need not be
    // normalized. Returns emissive in RGB, transmittance in A.
    float4 RayMarch(float3 InStartPos, float3 InViewVec, int InMaxSteps,
                    float InDensityScale, float InJitter, float InNoisePower)
    {
        // Unit box [0,1] -> normalized galaxy space [-1,1]. Uniform scale, so a
        // direction is identical in both.
        float3 o = 2.0 * (InStartPos - 0.5);
        float3 dir = normalize(-InViewVec);

        float b = dot(o, dir);
        float c = dot(o, o) - 1.0;
        float disc = b * b - c;
        if (disc <= 0.0) { return float4(0.0, 0.0, 0.0, 1.0); }

        float sq = sqrt(disc);

        // Origin inside the sphere: clamp the near end. Outside: take the whole chord
        // WITHOUT clamping to zero, because with an inverted-normal proxy the shaded
        // point is the far face and the chord lies at negative t.
        float tEnter = (c < 0.0) ? 0.0 : (-b - sq);
        float tExit  = -b + sq;
        if (tExit <= tEnter) { return float4(0.0, 0.0, 0.0, 1.0); }

        int steps = max(InMaxSteps, 1);
        float normStep = (tExit - tEnter) / float(steps);

        // Optical path is in LOCAL units and normalized space is 2x local.
        float localStep = normStep * 0.5;

        // Dither the entry point, not the step length, so every ray keeps an
        // identical total path and density does not shimmer with the noise.
        float t = tEnter + InJitter * normStep;

        bool bShape = (InNoisePower != 1.0);

        float transmittance = 1.0;
        float emission = 0.0;

        for (int i = 0; i < steps; i++)
        {
            float density = Sample(o + dir * t);
            if (bShape) { density = PowSafe(density, InNoisePower); }

            float alpha = 1.0 - exp(-density * InDensityScale * localStep);

            // Flat white emission: structure only.
            emission += alpha * transmittance;
            transmittance *= 1.0 - alpha;

            if (transmittance < 0.001) { break; }

            t += normStep;
        }

        return float4(emission.xxx, transmittance);
    }
};

// --- SETUP ---
GalaxyDensitySampler gd;

gd.NoiseTex                  = NoiseTex;
gd.NoiseTexSampler           = NoiseTexSampler;

// Fields arrive PRE-CLAMPED: the sample functions run per march step and must not
// repeat guards that depend only on uniforms.
gd.BoundsFadeStart           = BoundsFadeStart;
gd.BoundsFadeStartSq         = BoundsFadeStart * BoundsFadeStart;
gd.InvBoundsFadeSpan         = 1.0 / max(1.0 - BoundsFadeStart, 1e-6);

// --- PER-LAYER FAMILIES: x = arms, y = disc, z = bulge, w = background ---
// Every layer is a body with a lateral scale S and a vertical extent S * V. The four
// vertical controls are all axis ratios despite differing in form; smaller flattens.
//
// LateralScale.y does double duty: DiscRadius is also the arm system's radial
// reference. Arms deliberately have no independent radius -- they live in the disc.
gd.ArmWidth                  = max(LateralScale.x, 1e-6);
gd.DiscRadius                = max(LateralScale.y, 1e-6);

gd.InvArmVerticalRatio       = 1.0 / max(VerticalScale.x, 1e-4);
gd.DiscHeightRatio           = max(VerticalScale.y, 1e-6);

gd.NoiseAmountArm            = NoiseAmount.x;
gd.NoiseAmountDisc           = NoiseAmount.y;
gd.NoiseAmountBulge          = NoiseAmount.z;
gd.NoiseAmountBackground     = NoiseAmount.w;

gd.WarpAmountArms            = WarpAmount.x;
gd.WarpAmountDisc            = WarpAmount.y;
gd.WarpAmountBulge           = WarpAmount.z;
gd.WarpAmountBackground      = WarpAmount.w;

gd.BulgeConcentration        = max(BulgeConcentration, GALAXY_POW_EPSILON);
gd.BackgroundConcentration   = max(BackgroundConcentration, GALAXY_POW_EPSILON);
gd.DiscVerticalFalloff       = max(DiscVerticalFalloff, 0.1);

// Saturated, not just floored: above 1 the blend band exceeds the value and
// SmoothMaxPoly stops returning A for smax(A, 0), so merging a zero-contribution arm
// would inflate the accumulator.
gd.ArmMergeSmooth            = saturate(ArmMergeSmooth);

gd.NoiseOctaveCount          = (int)clamp(NoiseOctaves, 1.0, (float)GALAXY_MAX_OCTAVES);
gd.InvNoiseNorm              = 1.0 / ((gd.NoiseOctaveCount >= 2) ? 1.5 : 1.0);

gd.DiscFlare                 = max(DiscFlare, 0.0);
gd.DiscWarpAmplitude         = DiscWarpAmplitude;
gd.DiscWarpPhase             = DiscWarpPhase;
gd.DiscWarpTwist             = DiscWarpTwist;
gd.DiscLopsidedAmount        = DiscLopsidedAmount;
gd.DiscLopsidedPhase         = DiscLopsidedPhase;

gd.ArmPitchTightening        = max(SpiralTwist.y, 0.0);   // x pitch, y tightening,
gd.HaloTwistInherit          = SpiralTwist.w;             // z phase, w halo inherit
gd.ArmProfileExponent        = max(ArmProfileExponent, GALAXY_POW_EPSILON);
gd.ArmRadialGrowth           = ArmRadialGrowth;
gd.ArmHostFalloff            = max(ArmHostFalloff, 0.0);

gd.NoiseOffset               = NoiseOffset;
gd.NoiseScale                = max(NoiseScale, 1e-6);   // frame convention, see header
gd.WarpScale                 = max(WarpScale, 1e-6);
gd.NoiseRidged               = saturate(NoiseRidged);
gd.NoiseEnable               = NoiseEnable;
gd.NoiseChannelWeights       = NoiseChannelWeights;

gd.VoidStrength              = saturate(CentralVoid.y);  // x radius, y strength,
gd.VoidConcentration         = max(CentralVoid.z, GALAXY_POW_EPSILON);  // z conc

// --- HOISTED UNIFORMS ---
// Everything below is invariant across the march. These sit behind early-returns in
// the sample functions, so the compiler cannot hoist them speculatively.
float pitchDeg = clamp(SpiralTwist.x, -89.0, 89.0);
float tanP = tan(radians(max(abs(pitchDeg), 1.0)));
gd.ArmSpiralK0 = ((pitchDeg < 0.0) ? -1.0 : 1.0) / tanP;

gd.ArmPitchFactorMax = 1.0 + abs(ArmAsym.x);

gd.InvDiscRadius  = 1.0 / gd.DiscRadius;
gd.InvDiscScaleL  = 1.0 / max(max(LateralScale.z, 1e-6) * max(DiscScaleLengthRatio, 1e-6), 1e-6);

float armFadeW = max(0.2 * gd.DiscRadius, 1e-6);
gd.InvArmFadeW = 1.0 / armFadeW;

float voidR = max(CentralVoid.x, 1e-6);
gd.InvVoidRadiusSq = 1.0 / (voidR * voidR);

// Reciprocals for the two spheroid layers: SampleSpheroid ran a divide for each of
// these on every sample, for values that never change.
float bulgeR = max(LateralScale.z, 1e-6);
float bgR    = max(LateralScale.w, 1e-6);
gd.BulgeInvRadiusSq      = 1.0 / (bulgeR * bulgeR);
gd.BackgroundInvRadiusSq = 1.0 / (bgR * bgR);
gd.BulgeInvSquash        = 1.0 / max(VerticalScale.z, 1e-3);
gd.BackgroundInvSquash   = 1.0 / max(VerticalScale.w, 1e-3);

// LayerDensity is an OPTICAL DEPTH per layer, not a density: what you would measure
// looking through that layer along its natural axis. Dividing by the layer's own path
// length here is what makes the four values comparable and independent of MaxSteps.
gd.ArmDensityOverPath = LayerDensity.x / (0.5 * gd.ArmWidth);

gd.DiscDensityOverPath = LayerDensity.y
    / max(0.5 * gd.CompactProfileIntegral(gd.DiscVerticalFalloff), 1e-6);

gd.BulgeDensityOverPath = LayerDensity.z
    / max(0.5 * bulgeR * gd.CompactProfileIntegral(gd.BulgeConcentration), 1e-6);

gd.BackgroundDensityOverPath = LayerDensity.w
    / max(0.5 * bgR * gd.CompactProfileIntegral(gd.BackgroundConcentration), 1e-6);

// --- PER-ARM RECORDS ---
// Invariant along a ray, so hashing them per march step would burn N hashes x
// MaxSteps per pixel. Every slot is filled, not just the first ArmN, so none is read
// uninitialised. Statement code rather than a member function: DXC rejects a
// void-returning member that mutates a function-scope struct.
gd.ArmN = (int)clamp(ArmCount, 1.0, (float)GALAXY_MAX_ARMS);

float armSpacing = 2.0 * GALAXY_PI / float(gd.ArmN);
int armSeed = (int)ArmAsymSeed;

for (int ai = 0; ai < GALAXY_MAX_ARMS; ai++)
{
    float4 h = gd.ArmHash(ai, armSeed);

    gd.ArmData[ai] = float4(
        1.0 + ArmAsym.x * (2.0 * h.x - 1.0),
        SpiralTwist.z + float(ai) * armSpacing
            + ArmAsym.y * armSpacing * (2.0 * h.y - 1.0),
        max(1.0 + ArmAsym.z * (2.0 * h.z - 1.0), 0.0),
        gd.DiscRadius * (1.0 - ArmAsym.w * h.w) - armFadeW);
}

int3 randpos = int3(Parameters.SvPosition.xy, View.StateFrameIndexMod8);
float rand = float(Rand3DPCG16(randpos).x) / 0xffff;

return gd.RayMarch(
    CurPos,
    LocalCamVec,
    (int)MaxSteps,
    MasterDensityScale,
    rand,                 // pass 0.0 to disable dither
    MasterDensityPower
);
