
#include "fieldimpl.hpp"
#include "./bellhop_cuda_windows/trace.hpp"

#include <vector>

#ifndef BHC_BUILD_CUDA
namespace bhc {
    namespace mode {

        using GENCFG = CfgSel<BHCGENRUN, BHCGENINFL, BHCGENSSP>;

        template<typename CFG, bool O3D, bool R3D> void FieldModesWorker(
            bhcParams<BHCGENO3D>& params,
            bhcOutputs<BHCGENO3D, BHCGENR3D>& outputs,
            ErrState* errState)
        {
            SetupThread();
            while (true) {
                int32_t job = GetInternal(params)->sharedJobID++;
                RayInitInfo rinit;
                if (!GetJobIndices<BHCGENO3D>(rinit, job, params.Pos, params.Angles)) break;

                MainFieldModes<GENCFG, BHCGENO3D, BHCGENR3D>(
                    rinit, outputs.uAllSources, params.Bdry, params.bdinfo, params.refl,
                    params.ssp, params.Pos, params.Angles, params.freqinfo, params.Beam,
                    params.sbp, outputs.eigen, outputs.arrinfo, errState);
            }
        }

        template<typename CFG, bool O3D, bool R3D> void RunFieldModesImpl(
            bhcParams<BHCGENO3D>& params,
            bhcOutputs<BHCGENO3D, BHCGENR3D>& outputs)
        {
            ErrState errState;
            ResetErrState(&errState);
            GetInternal(params)->sharedJobID = 0;
            int32_t numThreads = GetInternal(params)->numThreads;
            std::vector<std::thread> threads;
            for (int32_t i = 0; i < numThreads; ++i)
                threads.push_back(std::thread([&]() { FieldModesWorker<GENCFG, BHCGENO3D, BHCGENR3D>(std::ref(params),
                    std::ref(outputs), &errState); })
                    );
            for (int32_t i = 0; i < numThreads; ++i) threads[i].join();
            CheckReportErrors(GetInternal(params), &errState);
        }

        void RunFieldModesImplLaunch(
            bhcParams<BHCGENO3D>& params, bhcOutputs<BHCGENO3D, BHCGENR3D>& outputs)
        {
            RunFieldModesImpl<GENCFG, BHCGENO3D, BHCGENR3D>(params, outputs);
        }

    }
} // namespace bhc::mode
#endif