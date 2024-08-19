#pragma once
#include "./bellhop_cuda_windows/common.hpp"

#ifndef BHC_BUILD_CUDA
namespace bhc { namespace mode {

template<typename CFG, bool O3D, bool R3D> void FieldModesWorker(
    bhcParams<O3D>& params, bhcOutputs<O3D, R3D>& outputs, ErrState* errState);

template<typename CFG, bool O3D, bool R3D> void RunFieldModesImpl(
    bhcParams<O3D> &params, bhcOutputs<O3D, R3D> &outputs);
    
void RunFieldModesImplLaunch(bhcParams<BHCGENO3D>& params, bhcOutputs<BHCGENO3D, BHCGENR3D>& outputs);

template<bool O3D, bool R3D> void RunFieldModesSelInfl(
    bhcParams<O3D> &params, bhcOutputs<O3D, R3D> &outputs);
extern template void RunFieldModesSelInfl<false, false>(
    bhcParams<false> &params, bhcOutputs<false, false> &outputs);
extern template void RunFieldModesSelInfl<true, false>(
    bhcParams<true> &params, bhcOutputs<true, false> &outputs);
extern template void RunFieldModesSelInfl<true, true>(
    bhcParams<true> &params, bhcOutputs<true, true> &outputs);

}} // namespace bhc::mode
#endif