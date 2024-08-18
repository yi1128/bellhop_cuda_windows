#pragma once
#include "../common_setup.hpp"
#include "modemodule.hpp"

#include "../trace.hpp"
#include "../module/title.hpp"
#include <vector>

namespace bhc { namespace mode {

template<bool O3D, bool R3D> bool RunRay(
    RayInfo<O3D, R3D> *rayinfo, const bhcParams<O3D> &params, int32_t job, int32_t worker,
    RayInitInfo &rinit, int32_t &Nsteps, ErrState *errState);
extern template bool RunRay<false, false>(
    RayInfo<false, false> *rayinfo, const bhcParams<false> &params, int32_t job,
    int32_t worker, RayInitInfo &rinit, int32_t &Nsteps, ErrState *errState);
extern template bool RunRay<true, false>(
    RayInfo<true, false> *rayinfo, const bhcParams<true> &params, int32_t job,
    int32_t worker, RayInitInfo &rinit, int32_t &Nsteps, ErrState *errState);
extern template bool RunRay<true, true>(
    RayInfo<true, true> *rayinfo, const bhcParams<true> &params, int32_t job,
    int32_t worker, RayInitInfo &rinit, int32_t &Nsteps, ErrState *errState);

template<bool O3D, bool R3D> void RunRayMode(
    bhcParams<O3D> &params, bhcOutputs<O3D, R3D> &outputs);
extern template void RunRayMode<false, false>(
    bhcParams<false> &params, bhcOutputs<false, false> &outputs);
extern template void RunRayMode<true, false>(
    bhcParams<true> &params, bhcOutputs<true, false> &outputs);
extern template void RunRayMode<true, true>(
    bhcParams<true> &params, bhcOutputs<true, true> &outputs);

template<bool O3D, bool R3D> void ReadOutRay(
    bhcParams<O3D> &params, bhcOutputs<O3D, R3D> &outputs, const char *FileRoot);
extern template void ReadOutRay<false, false>(
    bhcParams<false> &params, bhcOutputs<false, false> &outputs, const char *FileRoot);
extern template void ReadOutRay<true, false>(
    bhcParams<true> &params, bhcOutputs<true, false> &outputs, const char *FileRoot);
extern template void ReadOutRay<true, true>(
    bhcParams<true> &params, bhcOutputs<true, true> &outputs, const char *FileRoot);

template<bool O3D, bool R3D> class Ray : public ModeModule<O3D, R3D> {
public:
    Ray() {}
    virtual ~Ray() {}

    virtual void Init(bhcOutputs<O3D, R3D> &outputs) const override
    {
        outputs.rayinfo->results    = nullptr;
        outputs.rayinfo->RayMem     = nullptr;
        outputs.rayinfo->WorkRayMem = nullptr;

        outputs.rayinfo->RayMemCapacity  = 0;
        outputs.rayinfo->RayMemPoints    = 0;
        outputs.rayinfo->MaxPointsPerRay = 0;
        outputs.rayinfo->NRays           = 0;
    }

    virtual void Preprocess(
        bhcParams<O3D> &params, bhcOutputs<O3D, R3D> &outputs) const override
    {
        RayInfo<O3D, R3D> *rayinfo = outputs.rayinfo;

        trackdeallocate(params, rayinfo->RayMem);
        trackdeallocate(params, rayinfo->WorkRayMem);
        rayinfo->NRays = IsEigenraysRun(params.Beam)
            ? outputs.eigen->neigen
            : GetNumJobs<O3D>(params.Pos, params.Angles);
        trackallocate(params, "ray metadata", rayinfo->results, rayinfo->NRays);
        // Clear because will check pointers
        memset(rayinfo->results, 0, rayinfo->NRays * sizeof(RayResult<O3D, R3D>));

        rayinfo->MaxPointsPerRay = MaxN;
        rayinfo->isCopyMode      = false;
        size_t needtotalsize = (size_t)rayinfo->NRays * (size_t)MaxN * sizeof(rayPt<R3D>);
        if(GetInternal(params)->usedMemory + needtotalsize
           <= GetInternal(params)->maxMemory) {
            rayinfo->RayMemCapacity = (size_t)rayinfo->NRays * (size_t)MaxN;
        } else if(GetInternal(params)->useRayCopyMode) {
            trackallocate(
                params, "work rays for copy mode", rayinfo->WorkRayMem,
                GetInternal(params)->numThreads * MaxN);
            rayinfo->RayMemCapacity = (GetInternal(params)->maxMemory
                                       - GetInternal(params)->usedMemory)
                / sizeof(rayPt<R3D>);
            rayinfo->isCopyMode = true;
        } else {
            rayinfo->MaxPointsPerRay = (int32_t)std::min(
                (GetInternal(params)->maxMemory - GetInternal(params)->usedMemory)
                    / ((size_t)rayinfo->NRays * sizeof(rayPt<R3D>)),
                (size_t)0x7FFFFFFF);
            if(rayinfo->MaxPointsPerRay == 0) {
                EXTERR("Insufficient memory to allocate any rays at all");
            } else if(rayinfo->MaxPointsPerRay < 500) {
                EXTWARN(
                    "There is only enough memory to allocate %d points per ray",
                    rayinfo->MaxPointsPerRay);
            }
            rayinfo->RayMemCapacity = (size_t)rayinfo->NRays
                * (size_t)rayinfo->MaxPointsPerRay;
        }
        trackallocate(params, "rays", rayinfo->RayMem, rayinfo->RayMemCapacity);
        rayinfo->RayMemPoints = 0;
    }

    virtual void Run(bhcParams<O3D> &params, bhcOutputs<O3D, R3D> &outputs) const override
    {
        RunRayMode<O3D, R3D>(params, outputs);
    }

    virtual void Postprocess(
        bhcParams<O3D> &params, bhcOutputs<O3D, R3D> &outputs) const override
    {
        RayInfo<O3D, R3D> *rayinfo = outputs.rayinfo;
        for(int r = 0; r < rayinfo->NRays; ++r) {
            RayResult<O3D, R3D> *res = &rayinfo->results[r];
            if(res->ray == nullptr) continue;
            CompressRay(res, params.Bdry);
        }
    }

    virtual void Writeout(
        const bhcParams<O3D> &params, const bhcOutputs<O3D, R3D> &outputs) const override
    {
        RayInfo<O3D, R3D> *rayinfo = outputs.rayinfo;
        LDOFile RAYFile;
        OpenRAYFile(RAYFile, GetInternal(params)->FileRoot, params);
        for(int r = 0; r < rayinfo->NRays; ++r) {
            const RayResult<O3D, R3D> *res = &rayinfo->results[r];
            if(res->ray == nullptr) continue;
            WriteRay(RAYFile, res);
        }
    }

    virtual void Readout(
        bhcParams<O3D> &params, bhcOutputs<O3D, R3D> &outputs,
        const char *FileRoot) const override
    {
        ReadOutRay<O3D, R3D>(params, outputs, FileRoot);
    }

    virtual void Finalize(
        bhcParams<O3D> &params, bhcOutputs<O3D, R3D> &outputs) const override
    {
        trackdeallocate(params, outputs.rayinfo->results);
        trackdeallocate(params, outputs.rayinfo->RayMem);
        trackdeallocate(params, outputs.rayinfo->WorkRayMem);
    }

private:
    // LP: These are small enough that it's not really necessary to compile
    // them separately.

    inline void OpenRAYFile(
        LDOFile &RAYFile, std::string FileRoot, const bhcParams<O3D> &params) const
    {
        if(!IsRayRun(params.Beam) && !IsEigenraysRun(params.Beam)) {
            EXTERR("OpenRAYFile not in ray trace or eigenrays mode");
        }
        RAYFile.open(FileRoot + ".ray");
        RAYFile << params.Title << '\n';
        RAYFile << params.freqinfo->freq0 << '\n';
        RAYFile << params.Pos->NSx << params.Pos->NSy << params.Pos->NSz << '\n';
        RAYFile << params.Angles->alpha.n << params.Angles->beta.n << '\n';
        RAYFile << params.Bdry->Top.hs.Depth << '\n';
        RAYFile << params.Bdry->Bot.hs.Depth << '\n';
        RAYFile << (O3D ? "xyz" : "rz") << '\n';
    }

    /**
     * Compress the ray data keeping every iSkip point, points near surface or bottom, and
     * last point.
     *
     * The 2D version is for ray traces in (r,z) coordinates
     * The 3D version is for ray traces in (x,y,z) coordinates
     */
    inline void CompressRay(RayResult<O3D, R3D> *res, const BdryType *Bdry) const
    {
        // this is the maximum length of the ray vector that is written out
        constexpr int32_t MaxNRayPoints = 500000;

        // compression
        // LP: This is silly for two reasons:
        // 1) MaxN (maximum number of steps for a ray) is 100000, but MaxNRayPoints
        //    is 500000. Therefore iSkip will always be 1, and the whole vector will
        //    always be written.
        // 2) Even if these constants were changed, the formula for iSkip is not
        //    ideal: iSkip will only become 2 once the number of steps in the ray is
        //    more than 2x MaxNRayPoints. If it's less than this, it'll just be
        //    truncated, which is arguably worse than skipping every other step.
        // So we'll just make sure this doesn't run unless the constants are changed.
        if constexpr(MaxN > MaxNRayPoints) {
            int32_t n2    = 1;
            int32_t iSkip = bhc::max(res->Nsteps / MaxNRayPoints, 1);
            if constexpr(R3D) iSkip = 1; // LP: overrides line above

            for(int32_t is = 1; is < res->Nsteps; ++is) {
                // ensure that we always write ray points near bdry reflections (2D only:
                // works only for flat bdry)
                if(bhc::min(
                       Bdry->Bot.hs.Depth - DEP(res->ray[is].x),
                       DEP(res->ray[is].x) - Bdry->Top.hs.Depth)
                       < FL(0.2)
                   || (is % iSkip) == 0 || is == res->Nsteps - 1) {
                    ++n2;
                    res->ray[n2 - 1].x = res->ray[is].x;
                }
            }
            res->Nsteps = n2;
        }
    }

    /**
     * Write to RAYFile.
     */
    inline void WriteRay(LDOFile &RAYFile, const RayResult<O3D, R3D> *res) const
    {
        // take-off angle of this ray [LP: 2D: degrees, 3D: radians]
        real alpha0 = res->SrcDeclAngle;
        if constexpr(O3D) alpha0 *= DegRad;
        RAYFile << alpha0 << '\n';

        RAYFile << res->Nsteps;
        RAYFile << res->ray[res->Nsteps - 1].NumTopBnc;
        RAYFile << res->ray[res->Nsteps - 1].NumBotBnc << '\n';
        for(int32_t is = 0; is < res->Nsteps; ++is) {
            RAYFile << RayToOceanX(res->ray[is].x, res->org) << '\n';
        }
    }
};

}} // namespace bhc::mode

namespace bhc {
    namespace mode {

        template<bool O3D, bool R3D> bool RunRay(
            RayInfo<O3D, R3D>* rayinfo, const bhcParams<O3D>& params, int32_t job, int32_t worker,
            RayInitInfo& rinit, int32_t& Nsteps, ErrState* errState)
        {
            if (job >= rayinfo->NRays || worker >= GetInternal(params)->numThreads) {
                RunError(errState, BHC_ERR_JOBNUM);
                return false;
            }
            rayPt<R3D>* ray;
            if (rayinfo->isCopyMode) {
                ray = &rayinfo->WorkRayMem[worker * rayinfo->MaxPointsPerRay];
            }
            else {
                ray = &rayinfo->RayMem[(size_t)job * rayinfo->MaxPointsPerRay];
            }
#ifdef BHC_DEBUG
            // Set to garbage values for debugging
            memset(ray, 0xFE, rayinfo->MaxPointsPerRay * sizeof(rayPt<R3D>));
#endif

            Origin<O3D, R3D> org;
            char st = params.ssp->Type;
            if (st == 'N') {
                MainRayMode<CfgSel<'R', 'G', 'N'>, O3D, R3D>(
                    rinit, ray, Nsteps, rayinfo->MaxPointsPerRay, org, params.Bdry, params.bdinfo,
                    params.refl, params.ssp, params.Pos, params.Angles, params.freqinfo,
                    params.Beam, params.sbp, errState);
            }
            else if (st == 'C') {
                MainRayMode<CfgSel<'R', 'G', 'C'>, O3D, R3D>(
                    rinit, ray, Nsteps, rayinfo->MaxPointsPerRay, org, params.Bdry, params.bdinfo,
                    params.refl, params.ssp, params.Pos, params.Angles, params.freqinfo,
                    params.Beam, params.sbp, errState);
            }
            else if (st == 'S') {
                MainRayMode<CfgSel<'R', 'G', 'S'>, O3D, R3D>(
                    rinit, ray, Nsteps, rayinfo->MaxPointsPerRay, org, params.Bdry, params.bdinfo,
                    params.refl, params.ssp, params.Pos, params.Angles, params.freqinfo,
                    params.Beam, params.sbp, errState);
            }
            else if (st == 'P') {
                MainRayMode<CfgSel<'R', 'G', 'P'>, O3D, R3D>(
                    rinit, ray, Nsteps, rayinfo->MaxPointsPerRay, org, params.Bdry, params.bdinfo,
                    params.refl, params.ssp, params.Pos, params.Angles, params.freqinfo,
                    params.Beam, params.sbp, errState);
            }
            else if (st == 'Q') {
                MainRayMode<CfgSel<'R', 'G', 'Q'>, O3D, R3D>(
                    rinit, ray, Nsteps, rayinfo->MaxPointsPerRay, org, params.Bdry, params.bdinfo,
                    params.refl, params.ssp, params.Pos, params.Angles, params.freqinfo,
                    params.Beam, params.sbp, errState);
            }
            else if (st == 'H') {
                MainRayMode<CfgSel<'R', 'G', 'H'>, O3D, R3D>(
                    rinit, ray, Nsteps, rayinfo->MaxPointsPerRay, org, params.Bdry, params.bdinfo,
                    params.refl, params.ssp, params.Pos, params.Angles, params.freqinfo,
                    params.Beam, params.sbp, errState);
            }
            else if (st == 'A') {
                MainRayMode<CfgSel<'R', 'G', 'A'>, O3D, R3D>(
                    rinit, ray, Nsteps, rayinfo->MaxPointsPerRay, org, params.Bdry, params.bdinfo,
                    params.refl, params.ssp, params.Pos, params.Angles, params.freqinfo,
                    params.Beam, params.sbp, errState);
            }
            else {
                RunError(errState, BHC_ERR_INVALID_SSP_TYPE);
                return false;
            }
            if (HasErrored(errState)) return false;

            bool ret = true;
            if (rayinfo->isCopyMode) {
                size_t p = AtomicFetchAdd(&rayinfo->RayMemPoints, (size_t)Nsteps);
                if (p + (size_t)Nsteps > rayinfo->RayMemCapacity) {
                    RunWarning(errState, BHC_WARN_RAYS_OUTOFMEMORY);
                    rayinfo->results[job].ray = nullptr;
                    ret = false;
                }
                else {
                    rayinfo->results[job].ray = &rayinfo->RayMem[p];
                    memcpy(rayinfo->results[job].ray, ray, Nsteps * sizeof(rayPt<R3D>));
                }
            }
            else {
                rayinfo->results[job].ray = ray;
            }
            rayinfo->results[job].org = org;
            rayinfo->results[job].SrcDeclAngle = rinit.SrcDeclAngle;
            rayinfo->results[job].Nsteps = Nsteps;

            return ret;
        }

#if BHC_ENABLE_2D
        template bool RunRay<false, false>(
            RayInfo<false, false>* rayinfo, const bhcParams<false>& params, int32_t job,
            int32_t worker, RayInitInfo& rinit, int32_t& Nsteps, ErrState* errState);
#endif
#if BHC_ENABLE_NX2D
        template bool RunRay<true, false>(
            RayInfo<true, false>* rayinfo, const bhcParams<true>& params, int32_t job,
            int32_t worker, RayInitInfo& rinit, int32_t& Nsteps, ErrState* errState);
#endif
#if BHC_ENABLE_3D
        template bool RunRay<true, true>(
            RayInfo<true, true>* rayinfo, const bhcParams<true>& params, int32_t job,
            int32_t worker, RayInitInfo& rinit, int32_t& Nsteps, ErrState* errState);
#endif

        template<bool O3D, bool R3D> void RayModeWorker(
            const bhcParams<O3D>& params, bhcOutputs<O3D, R3D>& outputs, int32_t worker,
            ErrState* errState)
        {
            SetupThread();
            while (true) {
                int32_t job = GetInternal(params)->sharedJobID++;
                int32_t Nsteps = -1;
                RayInitInfo rinit;
                if (!GetJobIndices<O3D>(rinit, job, params.Pos, params.Angles)) break;
                if (!RunRay<O3D, R3D>(
                    outputs.rayinfo, params, job, worker, rinit, Nsteps, errState)) {
                    break;
                }
            }
        }

#if BHC_ENABLE_2D
        template void RayModeWorker<false, false>(
            const bhcParams<false>& params, bhcOutputs<false, false>& outputs, int32_t worker,
            ErrState* errState);
#endif
#if BHC_ENABLE_NX2D
        template void RayModeWorker<true, false>(
            const bhcParams<true>& params, bhcOutputs<true, false>& outputs, int32_t worker,
            ErrState* errState);
#endif
#if BHC_ENABLE_3D
        template void RayModeWorker<true, true>(
            const bhcParams<true>& params, bhcOutputs<true, true>& outputs, int32_t worker,
            ErrState* errState);
#endif

        template<bool O3D, bool R3D> void RunRayMode(
            bhcParams<O3D>& params, bhcOutputs<O3D, R3D>& outputs)
        {
            ErrState errState;
            ResetErrState(&errState);
            GetInternal(params)->sharedJobID = 0;
            int32_t numThreads = GetInternal(params)->numThreads;
            std::vector<std::thread> threads;
            for (int32_t i = 0; i < numThreads; ++i)
                threads.push_back(std::thread(
                    RayModeWorker<O3D, R3D>, std::ref(params), std::ref(outputs), i, &errState));
            for (int32_t i = 0; i < numThreads; ++i) threads[i].join();
            CheckReportErrors(GetInternal(params), &errState);
        }

#if BHC_ENABLE_2D
        template void RunRayMode<false, false>(
            bhcParams<false>& params, bhcOutputs<false, false>& outputs);
#endif
#if BHC_ENABLE_NX2D
        template void RunRayMode<true, false>(
            bhcParams<true>& params, bhcOutputs<true, false>& outputs);
#endif
#if BHC_ENABLE_3D
        template void RunRayMode<true, true>(
            bhcParams<true>& params, bhcOutputs<true, true>& outputs);
#endif

        template<bool O3D, bool R3D> void ReadOutRay(
            bhcParams<O3D>& params, bhcOutputs<O3D, R3D>& outputs, const char* FileRoot)
        {
            RayInfo<O3D, R3D>* rayinfo = outputs.rayinfo;
            if (!IsRayRun(params.Beam) && !IsEigenraysRun(params.Beam)) {
                EXTERR("ReadOutRay not in ray trace or eigenrays mode");
            }
            LDIFile RAYFile(GetInternal(params), std::string(FileRoot) + ".ray");

            std::string TempTitle;
            LIST(RAYFile);
            RAYFile.Read(TempTitle);
            bhc::module::Title<O3D> title;
            title.SetTitle(params, TempTitle);

            LIST(RAYFile);
            RAYFile.Read(params.freqinfo->freq0);

            int32_t NSx, NSy, NSz;
            LIST(RAYFile);
            RAYFile.Read(NSx);
            RAYFile.Read(NSy);
            RAYFile.Read(NSz);
            if (NSx != params.Pos->NSx || NSy != params.Pos->NSy || NSz != params.Pos->NSz) {
                EXTWARN(
                    "NSx = %d, NSy = %d, NSz = %d in RAYFile being loaded, but "
                    "%d, %d, %d in env file",
                    NSx, NSy, NSz, params.Pos->NSx, params.Pos->NSy, params.Pos->NSz);
            }

            int32_t alphaN, betaN;
            LIST(RAYFile);
            RAYFile.Read(alphaN);
            RAYFile.Read(betaN);
            if (alphaN != params.Angles->alpha.n) {
                EXTWARN(
                    "Warning, RAYFile has alphaN = %d, but env file has %d", alphaN,
                    params.Angles->alpha.n);
            }
            if (betaN != params.Angles->beta.n) {
                EXTWARN(
                    "Warning, RAYFile has betaN = %d, but env file has %d", betaN,
                    params.Angles->beta.n);
            }

            real TopDepth, BotDepth;
            LIST(RAYFile);
            RAYFile.Read(TopDepth);
            RAYFile.Read(BotDepth);
            if (TopDepth != params.Bdry->Top.hs.Depth) {
                EXTWARN(
                    "Warning, RAYFile has top depth = %f, but env file has %f", TopDepth,
                    params.Bdry->Top.hs.Depth);
            }
            if (BotDepth != params.Bdry->Bot.hs.Depth) {
                EXTWARN(
                    "Warning, RAYFile has bot depth = %f, but env file has %f", BotDepth,
                    params.Bdry->Bot.hs.Depth);
            }

            std::string dim;
            LIST(RAYFile);
            RAYFile.Read(dim);
            if constexpr (O3D) {
                if (dim != "xyz") {
                    EXTERR(
                        "Dimensionality in RAYFile is '%s', must be 'xyz' for 3D/Nx2D",
                        dim.c_str());
                }
            }
            else {
                if (dim != "rz") {
                    EXTERR("Dimensionality in RAYFile is '%s', must be 'rz' for 2D", dim.c_str());
                }
            }

            trackdeallocate(params, rayinfo->RayMem);
            trackdeallocate(params, rayinfo->WorkRayMem);

            [[maybe_unused]] std::vector<VEC23<O3D>> firstpoints;
            [[maybe_unused]] std::vector<VEC23<O3D>> lastpoints;

            auto pre_rays_pos = RAYFile.StateSave();
            int32_t NRays = 0;
            size_t TotalPoints = 0;
            while (!RAYFile.EndOfFile()) {
                real alpha0 = NAN;
                LIST(RAYFile);
                RAYFile.Read(alpha0);
                if (!std::isfinite(alpha0)) break; // Nothing written, out of data

                int32_t Nsteps = -1, NumTopBnc = -1, NumBotBnc = -1;
                LIST(RAYFile);
                RAYFile.Read(Nsteps);
                RAYFile.Read(NumTopBnc);
                RAYFile.Read(NumBotBnc);
                if (Nsteps <= 0) EXTERR("<= 0 points in ray in RAYFile");
                if (NumTopBnc < 0 || NumBotBnc < 0) {
                    EXTERR("Invalid number of bounces in ray in RAYFile");
                }
                TotalPoints += (size_t)Nsteps;
                ++NRays;

                VEC23<O3D> v;
                for (int32_t is = 0; is < Nsteps; ++is) {
                    LIST(RAYFile);
                    RAYFile.Read(v);
                    if constexpr (O3D && !R3D) {
                        if (is == 0) firstpoints.push_back(v);
                    }
                }
                if constexpr (O3D && !R3D) { lastpoints.push_back(v); }
            }

            rayinfo->NRays = NRays;
            rayinfo->RayMemPoints = rayinfo->RayMemCapacity = TotalPoints;
            rayinfo->MaxPointsPerRay = MaxN;
            trackallocate(params, "ray metadata", rayinfo->results, rayinfo->NRays);
            trackallocate(params, "rays", rayinfo->RayMem, rayinfo->RayMemCapacity);
            memset(rayinfo->RayMem, 0, rayinfo->RayMemCapacity * sizeof(rayPt<R3D>));

            RAYFile.StateLoad(pre_rays_pos);
            NRays = 0;
            TotalPoints = 0;
            while (!RAYFile.EndOfFile()) {
                real alpha0 = NAN;
                LIST(RAYFile);
                RAYFile.Read(alpha0);
                if (!std::isfinite(alpha0)) break; // Nothing written, out of data
                if constexpr (O3D) alpha0 *= RadDeg;
                rayinfo->results[NRays].SrcDeclAngle = alpha0;

                int32_t Nsteps = -1, NumTopBnc = -1, NumBotBnc = -1;
                LIST(RAYFile);
                RAYFile.Read(Nsteps);
                RAYFile.Read(NumTopBnc);
                RAYFile.Read(NumBotBnc);
                if (Nsteps <= 0) EXTERR("Internal error in RAYFile read second pass");
                if (NumTopBnc < 0 || NumBotBnc < 0) {
                    EXTERR("Internal error in RAYFile read second pass");
                }
                rayinfo->results[NRays].Nsteps = Nsteps;
                rayinfo->results[NRays].ray = &rayinfo->RayMem[TotalPoints];
                VEC23<R3D> t(RL(0.0));
                if constexpr (O3D && !R3D) {
                    rayinfo->results[NRays].org.xs = firstpoints[NRays];
                    t = XYCOMP(lastpoints[NRays] - firstpoints[NRays]);
                    t *= RL(1.0) / glm::length(t);
                    rayinfo->results[NRays].org.tradial = t;
                }
                rayinfo->results[NRays].ray[Nsteps - 1].NumTopBnc = NumTopBnc;
                rayinfo->results[NRays].ray[Nsteps - 1].NumBotBnc = NumBotBnc;

                ErrState errState;
                ResetErrState(&errState);
                for (int32_t is = 0; is < Nsteps; ++is) {
                    VEC23<O3D> v;
                    LIST(RAYFile);
                    RAYFile.Read(v);
                    rayinfo->results[NRays].ray[is].x
                        = OceanToRayX(v, rayinfo->results[NRays].org, t, -1, &errState);
                }
                CheckReportErrors(GetInternal(params), &errState);

                TotalPoints += (size_t)Nsteps;
                ++NRays;
            }
        }

#if BHC_ENABLE_2D
        template void ReadOutRay<false, false>(
            bhcParams<false>& params, bhcOutputs<false, false>& outputs, const char* FileRoot);
#endif
#if BHC_ENABLE_NX2D
        template void ReadOutRay<true, false>(
            bhcParams<true>& params, bhcOutputs<true, false>& outputs, const char* FileRoot);
#endif
#if BHC_ENABLE_3D
        template void ReadOutRay<true, true>(
            bhcParams<true>& params, bhcOutputs<true, true>& outputs, const char* FileRoot);
#endif

    }
} // namespace bhc::mode
