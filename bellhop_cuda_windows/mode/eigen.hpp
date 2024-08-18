
#pragma once
#include "../common_setup.hpp"
#include "ray.hpp"
#include "field.hpp"

#include "../common_run.hpp"

namespace bhc { namespace mode {

template<bool O3D, bool R3D> void PostProcessEigenrays(
    bhcParams<O3D> &params, bhcOutputs<O3D, R3D> &outputs);
extern template void PostProcessEigenrays<false, false>(
    bhcParams<false> &params, bhcOutputs<false, false> &outputs);
extern template void PostProcessEigenrays<true, false>(
    bhcParams<true> &params, bhcOutputs<true, false> &outputs);
extern template void PostProcessEigenrays<true, true>(
    bhcParams<true> &params, bhcOutputs<true, true> &outputs);

template<bool O3D, bool R3D> class Eigen : public Field<O3D, R3D> {
public:
    Eigen() {}
    virtual ~Eigen() {}

    virtual void Init(bhcOutputs<O3D, R3D> &outputs) const override
    {
        outputs.eigen->hits    = nullptr;
        outputs.eigen->neigen  = 0;
        outputs.eigen->memsize = 0;
    }

    virtual void Preprocess(
        bhcParams<O3D> &params, bhcOutputs<O3D, R3D> &outputs) const override
    {
        Field<O3D, R3D>::Preprocess(params, outputs);

        EigenInfo *eigen = outputs.eigen;
        trackdeallocate(params, eigen->hits); // Free memory if previously run
        // Use 1 / hitsMemFraction of the available memory for eigenray hits
        // (the rest for rays).
        constexpr size_t hitsMemFraction = 500;
        size_t mem     = GetInternal(params)->maxMemory - GetInternal(params)->usedMemory;
        eigen->memsize = (int32_t)
            std::min(mem / (hitsMemFraction * sizeof(EigenHit)), (size_t)0x7FFFFFFF);
        if(eigen->memsize == 0) {
            EXTERR("Insufficient memory to allocate any eigen hits at all");
        } else if(eigen->memsize < 10000) {
            EXTWARN(
                "There is only enough memory to allocate %d eigen hits, using 1/%dth "
                "of the total available memory for eigen hits",
                eigen->memsize, hitsMemFraction);
        }
        trackallocate(params, "eigenray hits", eigen->hits, eigen->memsize);
        eigen->neigen = 0;
    }

    virtual void Postprocess(
        bhcParams<O3D> &params, bhcOutputs<O3D, R3D> &outputs) const override
    {
        PostProcessEigenrays<O3D, R3D>(params, outputs);
    }

    virtual void Writeout(
        const bhcParams<O3D> &params, const bhcOutputs<O3D, R3D> &outputs) const override
    {
        Ray<O3D, R3D> raymode;
        raymode.Writeout(params, outputs);
    }

    virtual void Finalize(
        bhcParams<O3D> &params, bhcOutputs<O3D, R3D> &outputs) const override
    {
        trackdeallocate(params, outputs.eigen->hits);
    }
};

}} // namespace bhc::mode

namespace bhc {
    namespace mode {

        template<bool O3D, bool R3D> void EigenModePostWorker(
            const bhcParams<O3D>& params, bhcOutputs<O3D, R3D>& outputs, int32_t worker,
            ErrState* errState)
        {
            SetupThread();
            while (true) {
                int32_t job = GetInternal(params)->sharedJobID++;
                if (job >= bhc::min(outputs.eigen->neigen, outputs.eigen->memsize)) break;
                EigenHit* hit = &outputs.eigen->hits[job];
                int32_t Nsteps = hit->is;
                RayInitInfo rinit;
                rinit.isx = hit->isx;
                rinit.isy = hit->isy;
                rinit.isz = hit->isz;
                rinit.ialpha = hit->ialpha;
                rinit.ibeta = hit->ibeta;
                if (!RunRay<O3D, R3D>(
                    outputs.rayinfo, params, job, worker, rinit, Nsteps, errState)) {
                    // Already gave out of memory error; that is the only condition leading
                    // here printf("EigenModePostWorker RunRay failed\n");
                    break;
                }
            }
        }

#if BHC_ENABLE_2D
        template void EigenModePostWorker<false, false>(
            const bhcParams<false>& params, bhcOutputs<false, false>& outputs, int32_t worker,
            ErrState* errState);
#endif
#if BHC_ENABLE_NX2D
        template void EigenModePostWorker<true, false>(
            const bhcParams<true>& params, bhcOutputs<true, false>& outputs, int32_t worker,
            ErrState* errState);
#endif
#if BHC_ENABLE_3D
        template void EigenModePostWorker<true, true>(
            const bhcParams<true>& params, bhcOutputs<true, true>& outputs, int32_t worker,
            ErrState* errState);
#endif

        template<bool O3D, bool R3D> void PostProcessEigenrays(
            bhcParams<O3D>& params, bhcOutputs<O3D, R3D>& outputs)
        {
            Ray<O3D, R3D> raymode;
            raymode.Preprocess(params, outputs);

            if (outputs.eigen->neigen > outputs.eigen->memsize) {
                EXTWARN(
                    "Would have had %d eigenrays but only %d metadata fit in memory\n",
                    outputs.eigen->neigen, outputs.eigen->memsize);
            }
            else {
                EXTWARN("%d eigenrays\n", (int)outputs.eigen->neigen);
            }

            ErrState errState;
            ResetErrState(&errState);
            GetInternal(params)->sharedJobID = 0;
            int32_t numThreads = GetInternal(params)->numThreads;
            std::vector<std::thread> threads;
            for (int32_t i = 0; i < numThreads; ++i)
                threads.push_back(std::thread(
                    EigenModePostWorker<O3D, R3D>, std::cref(params), std::ref(outputs), i,
                    &errState));
            for (int32_t i = 0; i < numThreads; ++i) threads[i].join();
            CheckReportErrors(GetInternal(params), &errState);

            raymode.Postprocess(params, outputs);
        }

#if BHC_ENABLE_2D
        template void PostProcessEigenrays<false, false>(
            bhcParams<false>& params, bhcOutputs<false, false>& outputs);
#endif
#if BHC_ENABLE_NX2D
        template void PostProcessEigenrays<true, false>(
            bhcParams<true>& params, bhcOutputs<true, false>& outputs);
#endif
#if BHC_ENABLE_3D
        template void PostProcessEigenrays<true, true>(
            bhcParams<true>& params, bhcOutputs<true, true>& outputs);
#endif

    }
} // namespace bhc::mode
