#pragma once
// 0, 1, 2, 3, 4
// 1 : single thread
// 2 : 2D - BHC_ENABLE_2D
// 3 : 3D - BHC_ENABLE_3D
// 4 : NX2D - BHC_ENABLE_NX2D
#define BHC_DIM_ONLY			2

#define BHC_ENABLE_2D			TRUE
//#define BHC_ENABLE_NX2D		TRUE
//#define BHC_ENABLE_3D			TRUE

#ifdef BHC_ENABLE_2D
#define BHCGENO3D FALSE
#define BHCGENR3D FALSE
#endif
#ifdef BHC_ENABLE_3D
#define BHCGENO3D TRUE
#define BHCGENR3D TRUE
#endif
#ifdef BHC_ENABLE_NX2D
#define BHCGENO3D TRUE
#define BHCGENR3D FALSE
#endif
// 선언시 float 사용
// 미선언시 double 사용
//#define BHC_USE_FLOATS

#define BHC_BUILD_CUDA			TRUE

// Limit bellhopcxx/bellhopcuda to only features supported by BELLHOP/BELLHOP3D
//#define BHC_LIMIT_FEATURES

//****************** SSP Type ******************//
// 설명 : Enable N2-linear     1D SSP (ssp->Type == 'N')
#define BHC_SSP_ENABLE_N2LINEAR
#ifdef BHC_SSP_ENABLE_N2LINEAR
#define BHCGENSSP				'N'
#endif
// 설명 : Enable C-linear      1D SSP (ssp->Type == 'C')
//#define BHC_SSP_ENABLE_CLINEAR
#ifdef BHC_SSP_ENABLE_CLINEAR
#define BHCGENSSP				'C'
#endif
// 설명 : Enable cubic spline  1D SSP (ssp->Type == 'S')
//#define BHC_SSP_ENABLE_CUBIC
#ifdef BHC_SSP_ENABLE_CUBIC
#define BHCGENSSP				'S'
#endif
// 설명 : Enable PCHIP         1D SSP (ssp->Type == 'P')
//#define BHC_SSP_ENABLE_PCHIP
#ifdef BHC_SSP_ENABLE_PCHIP
#define BHCGENSSP				'P'
#endif
// 설명 : Enable quadrilateral 2D SSP (ssp->Type == 'Q')
//#define BHC_SSP_ENABLE_QUAD
#ifdef BHC_SSP_ENABLE_QUAD
#define BHCGENSSP				'Q'
#endif
// 설명 : Enable hexahedral    3D SSP (ssp->Type == 'H')
//#define BHC_SSP_ENABLE_HEXAHEDRAL
#ifdef BHC_SSP_ENABLE_HEXAHEDRAL
#define BHCGENSSP				'H'
#endif
// 설명 : Enable analytic   2D/3D SSP (ssp->Type == 'A')
//#define BHC_SSP_ENABLE_ANALYTIC
#ifdef BHC_SSP_ENABLE_ANALYTIC
#define BHCGENSSP				'A'
#endif

//****************** INFLUENCE Type ******************//
// 설명 : Enable Cerveny     ray-centered influence (Type == 'R')
#define BHC_INFL_ENABLE_CERVENY_RAYCEN
#ifdef BHC_INFL_ENABLE_CERVENY_RAYCEN
#define BHCGENINFL				'R'
#endif
// 설명 : Enable Cerveny     Cartesian    influence (Type == 'C')
//#define BHC_INFL_ENABLE_CERVENY_CART
#ifdef BHC_INFL_ENABLE_CERVENY_CART
#define BHCGENINFL				'C'
#endif
// 설명 : Enable geometrical ray-centered influence (Type == 'g' hat, 'b' Gaussian)
//#define BHC_INFL_ENABLE_GEOM_RAYCEN
#ifdef BHC_INFL_ENABLE_GEOM_RAYCEN
#define BHCGENINFL				'g'
#endif
// 설명 : Enable geometrical Cartesian    influence (Type == 'G' hat, 'B' Gaussian)
//#define BHC_INFL_ENABLE_GEOM_CART
#ifdef BHC_INFL_ENABLE_GEOM_CART
#define BHCGENINFL				'G'
#endif
// 설명 : Enable simple Gaussian beams    influence (Type == 'S')
//#define BHC_INFL_ENABLE_SGB
#ifdef BHC_INFL_ENABLE_SGB
#define BHCGENINFL				'S'
#endif

//****************** RUN Type ******************//
// 설명 : Enable TL runs        (RunType == 'C' or 'S' or 'I')
#define BHC_RUN_ENABLE_TL
#ifdef BHC_RUN_ENABLE_TL
#define BHCGENRUN				'C'
#endif
// 설명 : Enable eigenrays runs (RunType == 'E')
//#define BHC_RUN_ENABLE_EIGENRAYS
#ifdef BHC_RUN_ENABLE_EIGENRAYS
#define BHCGENRUN				'E'
#endif
// 설명 : Enable arrivals runs  (RunType == 'A' or 'a')
//#define BHC_RUN_ENABLE_ARRIVALS
#ifdef BHC_RUN_ENABLE_ARRIVALS
#define BHCGENRUN				'A'
#endif


//#define BHC_USE_HIGH_PRIORITY_THREADS