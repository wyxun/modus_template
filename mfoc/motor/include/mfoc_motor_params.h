/**
 * @file mfoc_motor_params.h
 * @brief Shared motor constants on fixed per-unit bases.
 * @author Codex
 * @date 2026-10-23
 */
#ifndef MFOC_MOTOR_PARAMS_H
#define MFOC_MOTOR_PARAMS_H
#include "mfoc_numeric.h"

/** @brief Motor constants; convert physical units once before Init.
 * Use amplitude-invariant Clarke and peak phase-current base Ib.
 * Vb is the voltage-vector base used by MFOC; wb is electrical rad/s, not Hz.
 * rs=Rs*Ib/Vb; ld/lq=Ld/Lq*wb*Ib/Vb; flux=psi*wb/Vb.
 * With p=pole_pairs: tb=1/wb, wmb=wb/p, psib=Vb/wb,
 * Tb=1.5*p*psib*Ib, Jb=Tb*tb/wmb; inertia=J/Jb.
 * Caller supplies finite values; rs/ld/lq/flux positive, inertia >= 0. */
typedef struct {
    uint16_t pole_pairs;
    mfoc_scalar_t rs;      /* Per-phase stator resistance, R/Rb. */
    mfoc_scalar_t ld;      /* d-axis inductance, Ld/Lb. */
    mfoc_scalar_t lq;      /* q-axis inductance, Lq/Lb. */
    mfoc_scalar_t flux;    /* Permanent-magnet flux linkage, psi/psib. */
    mfoc_scalar_t inertia; /* Rotor inertia, J/Jb; zero if unspecified. */
} mfoc_motor_params_t;
#endif
