/*
#include "eigen.hpp"
#include "../common_run.hpp"


namespace bhc { namespace mode {

template<bool O3D, bool R3D> void EigenModePostWorker(
    const bhcParams<O3D> &params, bhcOutputs<O3D, R3D> &outputs, int32_t worker,
    ErrState *errState)
{
    SetupThread();
    while(true) {
        int32_t job = GetInternal(params)->sharedJobID++;
        if(job >= bhc::min(outputs.eigen->neigen, outputs.eigen->memsize)) break;
        EigenHit *hit  = &outputs.eigen->hits[job];
        int32_t Nsteps = hit->is;
        RayInitInfo rinit;
        rinit.isx    = hit->isx;
        rinit.isy    = hit->isy;
        rinit.isz    = hit->isz;
        rinit.ialpha = hit->ialpha;
        rinit.ibeta  = hit->ibeta;
        if(!RunRay<O3D, R3D>(
               outputs.rayinfo, params, job, worker, rinit, Nsteps, errState)) {
            // Already gave out of memory error; that is the only condition leading
            // here printf("EigenModePostWorker RunRay failed\n");
            break;
        }
    }
}

#if BHC_ENABLE_2D
template void EigenModePostWorker<false, false>(
    const bhcParams<false> &params, bhcOutputs<false, false> &outputs, int32_t worker,
    ErrState *errState);
#endif
#if BHC_ENABLE_NX2D
template void EigenModePostWorker<true, false>(
    const bhcParams<true> &params, bhcOutputs<true, false> &outputs, int32_t worker,
    ErrState *errState);
#endif
#if BHC_ENABLE_3D
template void EigenModePostWorker<true, true>(
    const bhcParams<true> &params, bhcOutputs<true, true> &outputs, int32_t worker,
    ErrState *errState);
#endif

template<bool O3D, bool R3D> void PostProcessEigenrays(
    bhcParams<O3D> &params, bhcOutputs<O3D, R3D> &outputs)
{
    Ray<O3D, R3D> raymode;
    raymode.Preprocess(params, outputs);

    if(outputs.eigen->neigen > outputs.eigen->memsize) {
        EXTWARN(
            "Would have had %d eigenrays but only %d metadata fit in memory\n",
            outputs.eigen->neigen, outputs.eigen->memsize);
    } else {
        EXTWARN("%d eigenrays\n", (int)outputs.eigen->neigen);
    }

    ErrState errState;
    ResetErrState(&errState);
    GetInternal(params)->sharedJobID = 0;
    int32_t numThreads               = GetInternal(params)->numThreads;
    std::vector<std::thread> threads;
    for(int32_t i = 0; i < numThreads; ++i)
        threads.push_back(std::thread(
            EigenModePostWorker<O3D, R3D>, std::cref(params), std::ref(outputs), i,
            &errState));
    for(int32_t i = 0; i < numThreads; ++i) threads[i].join();
    CheckReportErrors(GetInternal(params), &errState);

    raymode.Postprocess(params, outputs);
}

#if BHC_ENABLE_2D
template void PostProcessEigenrays<false, false>(
    bhcParams<false> &params, bhcOutputs<false, false> &outputs);
#endif
#if BHC_ENABLE_NX2D
template void PostProcessEigenrays<true, false>(
    bhcParams<true> &params, bhcOutputs<true, false> &outputs);
#endif
#if BHC_ENABLE_3D
template void PostProcessEigenrays<true, true>(
    bhcParams<true> &params, bhcOutputs<true, true> &outputs);
#endif

}} // namespace bhc::mode
*/