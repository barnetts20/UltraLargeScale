#pragma once

// GalaxyHLSLShim.h
// The HLSL surface GalaxyDensityCore.ush uses, in C++. Include this FIRST, then
// the .ush, and one source compiles for both targets:
//
//     #include "GalaxyHLSLShim.h"
//     #include "GalaxyDensityCore.ush"
//
// The .ush deliberately stays inside this subset: no resources, no multi-component
// swizzles, no integer vectors. Anything added there that is not defined here breaks
// the C++ build, which is the intended failure -- a compile error rather than a
// silent divergence between star placement and the rendered field.
//
// PRECISION: float32 throughout, matching the shader. The core suffixes every float
// literal (0.5f, not 0.5) because C++ types an unsuffixed literal as double, which
// would promote whole expressions and round differently from the GPU. Keep the
// suffixes when editing; HLSL accepts them.
//
// THE NAMESPACE IS NEVER OPENED. Include the .ush INSIDE it instead:
//
//     #include "GalaxyHLSLShim.h"
//     namespace GalaxyHLSL { #include "GalaxyDensityCore.ush" }
//
// MSVC's <cmath> declares float overloads of sqrt/abs/exp/pow/... at GLOBAL scope,
// which libstdc++ does not. Opening this namespace would put both sets in the same
// scope and every call would be a genuine tie -- an ambiguity, not a resolution.
// Compiling the field inside the namespace makes ordinary unqualified lookup find
// GalaxyHLSL::sqrt and STOP, never reaching ::sqrt. Name hiding, not overloading.

#include <cmath>

// IDENTIFIES THE C++ SIDE to the shared .ush files. They need it for exactly one thing:
// the resource stubs below are C++ TYPES, so a .ush that names a stub type has to alias
// that name to a real HLSL resource type when the shader compiler is reading it. See
// Texture3DSigned. Nothing else in the shared files may branch on this -- a divergence
// gated on the compiler is precisely what this whole arrangement exists to prevent, and
// the one exception is a type name rather than any behaviour.
#define GALAXY_HLSL_SHIM 1

#ifdef _MSC_VER
#ifndef NOMINMAX
#define NOMINMAX    // else Windows.h turns max/min into macros
#endif
#endif

typedef unsigned int uint;

namespace GalaxyHLSL
{

    // ---------------------------------------------------------------------------
    // VECTOR TYPES
    // Lowercase members to match HLSL. Only the operators the core actually uses are
    // provided; anything missing is a compile error by design.
    // ---------------------------------------------------------------------------

    struct float2
    {
        float x, y;
        float2() : x(0.0f), y(0.0f) {}
        float2(float InX, float InY) : x(InX), y(InY) {}
    };

    struct float3
    {
        float x, y, z;
        float3() : x(0.0f), y(0.0f), z(0.0f) {}
        float3(float InX, float InY, float InZ) : x(InX), y(InY), z(InZ) {}

        float3& operator+=(const float3& R) { x += R.x; y += R.y; z += R.z; return *this; }
        float3& operator-=(const float3& R) { x -= R.x; y -= R.y; z -= R.z; return *this; }
        float3& operator*=(float S) { x *= S;   y *= S;   z *= S;   return *this; }
    };

    inline float3 operator+(const float3& A, const float3& B) { return float3(A.x + B.x, A.y + B.y, A.z + B.z); }
    inline float3 operator-(const float3& A, const float3& B) { return float3(A.x - B.x, A.y - B.y, A.z - B.z); }
    inline float3 operator*(const float3& A, float S) { return float3(A.x * S, A.y * S, A.z * S); }
    inline float3 operator*(float S, const float3& A) { return float3(A.x * S, A.y * S, A.z * S); }
    inline float3 operator-(const float3& A) { return float3(-A.x, -A.y, -A.z); }

    struct float4
    {
        float x, y, z, w;
        float4() : x(0.0f), y(0.0f), z(0.0f), w(0.0f) {}
        float4(float InX, float InY, float InZ, float InW) : x(InX), y(InY), z(InZ), w(InW) {}
    };

    inline float4 operator*(const float4& A, float S) { return float4(A.x * S, A.y * S, A.z * S, A.w * S); }
    inline float4 operator*(float S, const float4& A) { return float4(A.x * S, A.y * S, A.z * S, A.w * S); }

    // ELEMENTWISE. HLSL's float4 * float4 is componentwise, not a dot product, and the
    // universe core's WeightedVector3 relies on it to gain each displacement axis
    // independently. Added because that core had never compiled on this side -- the gap was
    // known and recorded in UniverseParams.h, and it is one line.
    inline float4 operator*(const float4& A, const float4& B)
    {
        return float4(A.x * B.x, A.y * B.y, A.z * B.z, A.w * B.w);
    }

    // ---------------------------------------------------------------------------
    // RESOURCE STUBS
    //
    // The core samples a volume texture, and C++ has none. These make it COMPILE, not
    // work: SampleLevel returns the neutral value, 0.5 in every channel, which is the
    // exact midpoint the core centres to zero. Every noise term derived from it --
    // channel mix, FBm, warp displacement -- comes out zero, and GalaxySample reduces
    // to SampleAnalytic term for term.
    //
    // THAT IS THE ONLY THING THE C++ SIDE MAY BE USED FOR: parameter derivation,
    // authoring and diagnostics against the analytic field. It MUST NEVER PLACE
    // ENTITIES. Placement against a texture-free approximation of a textured field was
    // the defect this entire migration existed to remove, and the failure mode is
    // silent -- a perfectly ordinary-looking galaxy that simply ignores the volume
    // texture the material draws.
    //
    // Returning 0.5 rather than 0 matters. Zero would centre to -1, drive modulation to
    // max(1 - Amount, 0) and warp by half a domain, so the C++ field would differ from
    // the analytic one instead of equalling it.
    //
    // TWO STUBS, BECAUSE THERE ARE NOW TWO TEXTURE CONVENTIONS AND THEY HAVE DIFFERENT
    // NEUTRALS. Splitting the universe field's one packed volume into four purpose-baked
    // assets split the decode with it:
    //
    //   Texture3D        UNORM, values in [0,1], neutral 0.5. The variance fetches, whose
    //                    channels feed VarianceT as a [0,1] lerp factor directly.
    //   Texture3DSigned  values already in [-1,1], neutral 0. The curl/gradient vector
    //                    fields the warp octaves read, which carry their own sign and are
    //                    never centred by the shader.
    //
    // GETTING THE NEUTRAL WRONG HERE IS SILENT AND STRUCTURAL. A signed asset stubbed at
    // 0.5 displaces the C++ field by half the warp amplitude on every axis, so the CPU
    // field stops being the unwarped analytic web and becomes a uniformly-sheared version
    // of it -- which evaluates cleanly, looks like a cosmic web, and matches what the shader
    // draws nowhere.
    // ---------------------------------------------------------------------------

    struct SamplerState {};

    struct Texture3D
    {
        float4 SampleLevel(const SamplerState&, const float3&, float) const
        {
            return float4(0.5f, 0.5f, 0.5f, 0.5f);
        }
    };

    // Distinct TYPE rather than a flag on the one above, so a fetch site cannot be given
    // the wrong convention without a compile error. On the HLSL side this name aliases
    // straight back to Texture3D -- see the guard at the top of UniverseDensityCore.ush --
    // because a shader has no use for the distinction: it is the decode that differs, and
    // the decode is written out at the call site.
    struct Texture3DSigned
    {
        float4 SampleLevel(const SamplerState&, const float3&, float) const
        {
            return float4(0.0f, 0.0f, 0.0f, 0.0f);
        }
    };

    // ---------------------------------------------------------------------------
    // INTRINSICS
    // Concrete float overloads, never templates: an exact match must win against the
    // int and double candidates the standard library contributes.
    // ---------------------------------------------------------------------------

    inline float max(float A, float B) { return A > B ? A : B; }
    inline float min(float A, float B) { return A < B ? A : B; }

    // Componentwise against a scalar. HLSL does this implicitly; C++ needs it spelled out.
    inline float4 max(const float4& A, float S)
    {
        return float4(max(A.x, S), max(A.y, S), max(A.z, S), max(A.w, S));
    }

    inline float abs(float X) { return ::fabsf(X); }
    inline float sqrt(float X) { return ::sqrtf(X); }
    inline float rsqrt(float X) { return 1.0f / ::sqrtf(X); }
    inline float exp(float X) { return ::expf(X); }

    // Base two, not a scaled exp(). Both are the hardware primitive on the GPU side,
    // and expf(X * ln2) would introduce a rounding step the shader does not take --
    // which is precisely the class of silent CPU/GPU divergence the shared cores exist
    // to avoid. UniverseDensityCore's node weight is the consumer.
    inline float exp2(float X) { return ::exp2f(X); }

    inline float log(float X) { return ::logf(X); }
    inline float log2(float X) { return ::log2f(X); }
    inline float pow(float X, float Y) { return ::powf(X, Y); }
    inline float sin(float X) { return ::sinf(X); }
    inline float cos(float X) { return ::cosf(X); }
    inline float tan(float X) { return ::tanf(X); }
    inline float asin(float X) { return ::asinf(X); }
    inline float atan2(float Y, float X) { return ::atan2f(Y, X); }
    inline float floor(float X) { return ::floorf(X); }

    // HLSL round() is round-half-to-even. nearbyintf honours the current rounding mode,
    // which is half-to-even by default; roundf would round half away from zero and
    // disagree with the GPU on exact .5 cases.
    inline float round(float X) { return ::nearbyintf(X); }

    inline float clamp(float X, float A, float B) { return X < A ? A : (X > B ? B : X); }
    inline float saturate(float X) { return clamp(X, 0.0f, 1.0f); }
    inline float lerp(float A, float B, float T) { return A + (B - A) * T; }
    inline float radians(float Deg) { return Deg * 0.01745329251994329577f; }

    inline void sincos(float X, float& OutSin, float& OutCos)
    {
        OutSin = ::sinf(X);
        OutCos = ::cosf(X);
    }

    inline float dot(const float3& A, const float3& B) { return A.x * B.x + A.y * B.y + A.z * B.z; }
    inline float dot(const float4& A, const float4& B) { return A.x * B.x + A.y * B.y + A.z * B.z + A.w * B.w; }

} // namespace GalaxyHLSL