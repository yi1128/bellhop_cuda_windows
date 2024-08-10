// For CUDA

#include "cuda_runtime.h"
#include "device_launch_parameters.h"

#include "../trace.hpp"
#include "fieldimpl.h"

namespace bhc { namespace mode {

#define NUM_THREADS 256
#define LAUNCH_BOUNDS __launch_bounds__(NUM_THREADS, 1)

    // 현재 사용안함
    using GENCFG = CfgSel<BHCGENRUN, BHCGENINFL, BHCGENSSP>;

    template<typename CFG, bool O3D, bool R3D> __global__ void
        FieldModesKernel(bhcParams<O3D> params, bhcOutputs<O3D, R3D> outputs,
            ErrState* errState);

    template<typename CFG, bool O3D, bool R3D> __global__ void
        FieldModesKernel(
            bhcParams<O3D> params,
            bhcOutputs<O3D, R3D> outputs,
            ErrState* errState)
    {
        for (int32_t job = blockIdx.x * blockDim.x + threadIdx.x; true;
            job += gridDim.x * blockDim.x) {
            RayInitInfo rinit;
            if (!GetJobIndices<O3D>(rinit, job, params.Pos, params.Angles)) break;

            MainFieldModes<CFG, O3D, R3D>(
                rinit, outputs.uAllSources, params.Bdry, params.bdinfo, params.refl,
                params.ssp, params.Pos, params.Angles, params.freqinfo, params.Beam,
                params.sbp, outputs.eigen, outputs.arrinfo, errState);
        }
    }

    template<typename CFG, bool O3D, bool R3D> void FieldModesWorker(
        bhcParams<O3D>& params, bhcOutputs<O3D, R3D>& outputs, ErrState* errState)
    {
        for (int32_t job = blockIdx.x * blockDim.x + threadIdx.x; true;
            job += gridDim.x * blockDim.x) {
            RayInitInfo rinit;
            if (!GetJobIndices<O3D>(rinit, job, params.Pos, params.Angles)) break;

            MainFieldModes<CFG, O3D, R3D>(
                rinit, outputs.uAllSources, params.Bdry, params.bdinfo, params.refl,
                params.ssp, params.Pos, params.Angles, params.freqinfo, params.Beam,
                params.sbp, outputs.eigen, outputs.arrinfo, errState);
        }
    }



    template<typename CFG, bool O3D, bool R3D> void RunFieldModesImpl(
        bhcParams<O3D>& params, bhcOutputs<O3D, R3D>& outputs)
    {
        ErrState* errState;
        checkCudaErrors(cudaMallocManaged(&errState, sizeof(ErrState)));
        ResetErrState(errState);
        FieldModesKernel<GENCFG, BHCGENO3D, BHCGENR3D> << <GetInternal(params)->d_multiprocs, NUM_THREADS >> > (params, outputs, errState);

        /*
        std::string __str = std::string("FieldModesKernel<") + std::string(BHCGENRUN) + std::string(",") +
            std::string(BHCGENINFL) + std::string(",") + std::string(BHCGENSSP) + std::string(",") +
            std::string(O3D) + std::string(",") + std::string(BHCGENR3D) + std::string(">");
        syncAndCheckKernelErrors(__str.c_str());
        */
        CheckReportErrors(GetInternal(params), errState);
        checkCudaErrors(cudaFree(errState));
    }


/*
using GENCFG = CfgSel<BHCGENRUN, BHCGENINFL, BHCGENSSP>;


template<typename CFG, bool O3D, bool R3D> __global__ void 
FieldModesKernel(bhcParams<O3D> params, bhcOutputs<O3D, R3D> outputs,
    ErrState *errState);

template<> __global__ void 
FieldModesKernel<GENCFG, BHCGENO3D, BHCGENR3D>(
    bhcParams<BHCGENO3D> params,
    bhcOutputs<BHCGENO3D, BHCGENR3D> outputs,
    ErrState *errState)
{
    for(int32_t job = blockIdx.x * blockDim.x + threadIdx.x; true;
        job += gridDim.x * blockDim.x) {
        RayInitInfo rinit;
        if(!GetJobIndices<BHCGENO3D>(rinit, job, params.Pos, params.Angles)) break;

        MainFieldModes<GENCFG, BHCGENO3D, BHCGENR3D>(
            rinit, outputs.uAllSources, params.Bdry, params.bdinfo, params.refl,
            params.ssp, params.Pos, params.Angles, params.freqinfo, params.Beam,
            params.sbp, outputs.eigen, outputs.arrinfo, errState);
    }
}

template<> void RunFieldModesImpl<GENCFG, BHCGENO3D, BHCGENR3D>(
    bhcParams<BHCGENO3D> &params,
    bhcOutputs<BHCGENO3D, BHCGENR3D> &outputs)
{
    ErrState *errState;
    checkCudaErrors(cudaMallocManaged(&errState, sizeof(ErrState)));
    ResetErrState(errState);
    FieldModesKernel<GENCFG, BHCGENO3D, BHCGENR3D>
        <<<GetInternal(params)->d_multiprocs, NUM_THREADS>>>(params, outputs, errState);
    syncAndCheckKernelErrors("FieldModesKernel<@BHCGENRUN@, @BHCGENINFL@, @BHCGENSSP@, "
                             "@BHCGENO3D@, @BHCGENR3D@>");
    CheckReportErrors(GetInternal(params), errState);
    checkCudaErrors(cudaFree(errState));
}
*/
}} // namespace bhc::mode

