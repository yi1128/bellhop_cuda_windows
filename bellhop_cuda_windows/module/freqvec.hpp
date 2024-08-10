#pragma once
#include "../common_setup.hpp"
#include "paramsmodule.hpp"

namespace bhc { namespace module {

/**
 * Optionally reads a vector of source frequencies for a broadband run
 * If the broadband option is not selected, then the input freq (a scalar) is stored in
 * the frequency vector
 * LP: This is not properly supported in BELLHOP(3D). The broadband option can
 * never be selected because that letter is used to select dev mode (single beam),
 * and putting 'B' there is considered invalid. Plus, freqVec is never read during
 * the beam trace or influence. However, this can't be removed, as the frequency
 * vector must be written out to the shade file.
 */
template<bool O3D> class FreqVec : public ParamsModule<O3D> {
public:
    FreqVec() {}
    virtual ~FreqVec() {}

    virtual void Init(bhcParams<O3D> &params) const override
    {
        params.freqinfo->freqVec = nullptr;
    }
    virtual void SetupPre(bhcParams<O3D> &params) const override
    {
        params.freqinfo->Nfreq = 1;
    }
    virtual void Default(bhcParams<O3D> &params) const override
    {
        trackallocate(
            params, Description, params.freqinfo->freqVec, params.freqinfo->Nfreq);
        params.freqinfo->freqVec[0] = params.freqinfo->freq0;
    }
    virtual void Read(bhcParams<O3D> &params, LDIFile &ENVFile, HSInfo &) const override
    {
        if(params.Bdry->Top.hs.Opt[5] == 'B') {
            ReadVector(
                params, params.freqinfo->freqVec, params.freqinfo->Nfreq, ENVFile,
                Description);
        } else {
            Default(params);
        }
    }
    virtual void Write(bhcParams<O3D> &params, LDOFile &ENVFile) const
    {
        if(params.Bdry->Top.hs.Opt[5] == 'B') {
            UnSubTab(
                ENVFile, params.freqinfo->freqVec, params.freqinfo->Nfreq, "Nfreq",
                "freqVec (Hz)");
        }
    }
    void ExtSetup(bhcParams<O3D> &params, int32_t Nfreq) const
    {
        params.freqinfo->Nfreq = Nfreq;
        trackallocate(
            params, Description, params.freqinfo->freqVec, params.freqinfo->Nfreq);
    }
    virtual void Validate(bhcParams<O3D> &params) const override
    {
        ValidateVector(
            params, params.freqinfo->freqVec, params.freqinfo->Nfreq, Description);
    }
    virtual void Echo(bhcParams<O3D> &params) const override
    {
        if(params.freqinfo->Nfreq != 1
           || params.freqinfo->freqVec[0] != params.freqinfo->freq0) {
            EchoVectorWDescr(
                params, params.freqinfo->freqVec, params.freqinfo->Nfreq, RL(1.0),
                Description, Units);
        }
    }
    virtual void Finalize(bhcParams<O3D> &params) const override
    {
        trackdeallocate(params, params.freqinfo->freqVec);
    }

private:
    constexpr static const char *Description = "Frequencies";
    constexpr static const char *Units       = "Hz";
};

}} // namespace bhc::module
