/**
 * @file mfoc_abi.h
 * @brief Link guard for the MFOC configuration seen by each object.
 * @author Codex
 * @date 2026-11-23
 */
#ifndef MFOC_ABI_H
#define MFOC_ABI_H

#if defined(MFOC_NUMERIC_FLOAT)
#define MFOC_ABI_NUM f
#else
#define MFOC_ABI_NUM q
#endif

#if MFOC_FEEDBACK_BACKEND == MFOC_FEEDBACK_NONE
#define MFOC_ABI_BACKEND 0
#elif MFOC_FEEDBACK_BACKEND == MFOC_FEEDBACK_POSITION
#define MFOC_ABI_BACKEND 1
#elif MFOC_FEEDBACK_BACKEND == MFOC_FEEDBACK_SMO
#define MFOC_ABI_BACKEND 2
#elif MFOC_FEEDBACK_BACKEND == MFOC_FEEDBACK_HFI
#define MFOC_ABI_BACKEND 3
#elif MFOC_FEEDBACK_BACKEND == MFOC_FEEDBACK_FLUX
#define MFOC_ABI_BACKEND 4
#elif MFOC_FEEDBACK_BACKEND == MFOC_FEEDBACK_EKF
#define MFOC_ABI_BACKEND 5
#else
#error "Select a supported MFOC feedback backend"
#endif

#if MFOC_FEEDBACK_ENABLE_POSITION
#define MFOC_ABI_POS 1
#else
#define MFOC_ABI_POS 0
#endif

#if MFOC_FEEDBACK_ENABLE_SMO
#define MFOC_ABI_SMO 1
#else
#define MFOC_ABI_SMO 0
#endif

#if MFOC_FEEDBACK_ENABLE_HFI
#define MFOC_ABI_HFI 1
#else
#define MFOC_ABI_HFI 0
#endif

#if MFOC_FEEDBACK_ENABLE_FLUX
#define MFOC_ABI_FLUX 1
#else
#define MFOC_ABI_FLUX 0
#endif

#if MFOC_FEEDBACK_ENABLE_EKF
#define MFOC_ABI_EKF 1
#else
#define MFOC_ABI_EKF 0
#endif

#if MFOC_IDENTIFICATION_ENABLE
#define MFOC_ABI_ID 1
#else
#define MFOC_ABI_ID 0
#endif

#if MFOC_MOTOR_MODEL_UPDATE_ENABLE
#define MFOC_ABI_MODEL 1
#else
#define MFOC_ABI_MODEL 0
#endif

#if MFOC_HFI_ENABLE_POLARITY
#define MFOC_ABI_POLARITY 1
#else
#define MFOC_ABI_POLARITY 0
#endif

#if MFOC_PWM_PRELOAD_DELAY
#define MFOC_ABI_PRELOAD 1
#else
#define MFOC_ABI_PRELOAD 0
#endif

#if defined(MFOC_USE_PERFC_FSM) && MFOC_USE_PERFC_FSM
#define MFOC_ABI_PERFC 1
#else
#define MFOC_ABI_PERFC 0
#endif

#if MFOC_TRIG_BACKEND == MFOC_TRIG_BACKEND_LIBM
#define MFOC_ABI_TRIG 0
#elif MFOC_TRIG_BACKEND == MFOC_TRIG_BACKEND_LUT
#define MFOC_ABI_TRIG 1
#elif MFOC_TRIG_BACKEND == MFOC_TRIG_BACKEND_EXTERNAL
#define MFOC_ABI_TRIG 2
#else
#error "Select a supported MFOC trig backend"
#endif

#define MFOC_ABI_TAG(n,b,p,s,h,f,e,i,m,w,d,pt,t) \
    mfoc_abi_v1_##n##_b##b##_p##p##_s##s##_h##h##_f##f##_e##e \
        ##_i##i##m##w##d##pt##_t##t
#define MFOC_ABI_EXPAND(n,b,p,s,h,f,e,i,m,w,d,pt,t) \
    MFOC_ABI_TAG(n,b,p,s,h,f,e,i,m,w,d,pt,t)
#define MFOC_ABI_SIGNATURE MFOC_ABI_EXPAND(MFOC_ABI_NUM, MFOC_ABI_BACKEND, \
    MFOC_ABI_POS, MFOC_ABI_SMO, MFOC_ABI_HFI, MFOC_ABI_FLUX, MFOC_ABI_EKF, \
    MFOC_ABI_ID, MFOC_ABI_MODEL, MFOC_ABI_POLARITY, MFOC_ABI_PRELOAD, \
    MFOC_ABI_PERFC, MFOC_ABI_TRIG)

#if defined(__ELF__) && (defined(__GNUC__) || defined(__clang__))
#define MFOC_ABI_ELF
#define MFOC_ABI_STRING_INNER(symbol) #symbol
#define MFOC_ABI_STRING(symbol) MFOC_ABI_STRING_INNER(symbol)

#if __SIZEOF_POINTER__ == 8
/* A native-width relocation also works under default 64-bit PIE linking. */
#define MFOC_ABI_REFERENCE ".balign 8\n.quad "
#else
#define MFOC_ABI_REFERENCE ".balign 4\n.long "
#endif

/* Retain one relocation per signature even with --gc-sections.  A unique
 * group per signature ensures an incompatible reference is never folded
 * into a compatible one.  The linked image holds one read-only pointer. */
__asm__(".pushsection .rodata.mfoc_abi,\"aGR\",%progbits,mfoc_ref_"
        MFOC_ABI_STRING(MFOC_ABI_SIGNATURE) ",comdat\n"
        MFOC_ABI_REFERENCE MFOC_ABI_STRING(MFOC_ABI_SIGNATURE) "\n"
        ".popsection");
#else
/* Object formats without ELF section retention bind the signature to
 * public call names, including Step when it is called from a separate TU.
 * Macro expansion changes only the link labels, never the call sequence. */
#define MFOC_ABI_ENTRY_INNER(name, signature) name##_##signature
#define MFOC_ABI_ENTRY_EXPAND(name, signature) \
    MFOC_ABI_ENTRY_INNER(name, signature)
#define MFOC_ABI_ENTRY(name) MFOC_ABI_ENTRY_EXPAND(name, MFOC_ABI_SIGNATURE)

/* Numerical and current-control APIs. */
#define mfoc_from_float MFOC_ABI_ENTRY(mfoc_from_float)
#define mfoc_div_checked MFOC_ABI_ENTRY(mfoc_div_checked)
#define mfoc_angle_from_turns MFOC_ABI_ENTRY(mfoc_angle_from_turns)
#define mfoc_angle_from_scalar MFOC_ABI_ENTRY(mfoc_angle_from_scalar)
#define mfoc_angle_to_turns MFOC_ABI_ENTRY(mfoc_angle_to_turns)
#define mfoc_angle_add_scalar MFOC_ABI_ENTRY(mfoc_angle_add_scalar)
#define mfoc_angle_diff MFOC_ABI_ENTRY(mfoc_angle_diff)
#define mfoc_angle_sincos MFOC_ABI_ENTRY(mfoc_angle_sincos)
#define mfoc_angle_atan2 MFOC_ABI_ENTRY(mfoc_angle_atan2)
#define mfoc_sqrt MFOC_ABI_ENTRY(mfoc_sqrt)
#define mfoc_clarke MFOC_ABI_ENTRY(mfoc_clarke)
#define mfoc_park_cached MFOC_ABI_ENTRY(mfoc_park_cached)
#define mfoc_ipark_cached MFOC_ABI_ENTRY(mfoc_ipark_cached)
#define mfoc_lut_sincos MFOC_ABI_ENTRY(mfoc_lut_sincos)
#define mfoc_trig_port_sincos MFOC_ABI_ENTRY(mfoc_trig_port_sincos)
#define mfoc_trig_port_atan2 MFOC_ABI_ENTRY(mfoc_trig_port_atan2)
#define mfoc_svpwm MFOC_ABI_ENTRY(mfoc_svpwm)
#define mfoc_spwm MFOC_ABI_ENTRY(mfoc_spwm)
#define mfoc_modulation_ModelVoltageAlphaBeta \
    MFOC_ABI_ENTRY(mfoc_modulation_ModelVoltageAlphaBeta)
#define mfoc_pi_Init MFOC_ABI_ENTRY(mfoc_pi_Init)
#define mfoc_pi_Reset MFOC_ABI_ENTRY(mfoc_pi_Reset)
#define mfoc_pi_ApplyLimit MFOC_ABI_ENTRY(mfoc_pi_ApplyLimit)
#define mfoc_pi_Track MFOC_ABI_ENTRY(mfoc_pi_Track)
#define mfoc_pi_Step MFOC_ABI_ENTRY(mfoc_pi_Step)
#define mfoc_core_Init MFOC_ABI_ENTRY(mfoc_core_Init)
#define mfoc_core_PrepareBus MFOC_ABI_ENTRY(mfoc_core_PrepareBus)
#define mfoc_core_BenchHalfVoltageLimit \
    MFOC_ABI_ENTRY(mfoc_core_BenchHalfVoltageLimit)
#define mfoc_core_SetBus MFOC_ABI_ENTRY(mfoc_core_SetBus)
#define mfoc_core_Reset MFOC_ABI_ENTRY(mfoc_core_Reset)
#define mfoc_core_Step MFOC_ABI_ENTRY(mfoc_core_Step)

/* Motor and Feedback APIs. */
#define mfoc_motor_Init MFOC_ABI_ENTRY(mfoc_motor_Init)
#define mfoc_motor_ApplyModel MFOC_ABI_ENTRY(mfoc_motor_ApplyModel)
#define mfoc_motor_Start MFOC_ABI_ENTRY(mfoc_motor_Start)
#define mfoc_motor_StartIdentification \
    MFOC_ABI_ENTRY(mfoc_motor_StartIdentification)
#define mfoc_motor_GetIdentificationResult \
    MFOC_ABI_ENTRY(mfoc_motor_GetIdentificationResult)
#define mfoc_motor_Stop MFOC_ABI_ENTRY(mfoc_motor_Stop)
#define mfoc_motor_SetCurrent MFOC_ABI_ENTRY(mfoc_motor_SetCurrent)
#define mfoc_motor_SetBus MFOC_ABI_ENTRY(mfoc_motor_SetBus)
#define mfoc_motor_Run MFOC_ABI_ENTRY(mfoc_motor_Run)
#define mfoc_motor_AlignStart MFOC_ABI_ENTRY(mfoc_motor_AlignStart)
#define mfoc_motor_OpenLoopStart MFOC_ABI_ENTRY(mfoc_motor_OpenLoopStart)
#define mfoc_motor_SensorlessStart MFOC_ABI_ENTRY(mfoc_motor_SensorlessStart)
#define mfoc_motor_HfiStart MFOC_ABI_ENTRY(mfoc_motor_HfiStart)
#define mfoc_motor_SetSpeed MFOC_ABI_ENTRY(mfoc_motor_SetSpeed)
#define mfoc_motor_Step MFOC_ABI_ENTRY(mfoc_motor_Step)
#define mfoc_motor_UpdateBus MFOC_ABI_ENTRY(mfoc_motor_UpdateBus)
#define mfoc_motor_Trip MFOC_ABI_ENTRY(mfoc_motor_Trip)
#define mfoc_motor_ClearFault MFOC_ABI_ENTRY(mfoc_motor_ClearFault)
#define mfoc_motor_GetStatus MFOC_ABI_ENTRY(mfoc_motor_GetStatus)
#define mfoc_feedback_Init MFOC_ABI_ENTRY(mfoc_feedback_Init)
#define mfoc_feedback_SetMotorParamModel \
    MFOC_ABI_ENTRY(mfoc_feedback_SetMotorParamModel)
#define mfoc_feedback_HfiStart MFOC_ABI_ENTRY(mfoc_feedback_HfiStart)
#define mfoc_feedback_Step MFOC_ABI_ENTRY(mfoc_feedback_Step)
#define mfoc_openloop_Init MFOC_ABI_ENTRY(mfoc_openloop_Init)
#define mfoc_openloop_Reset MFOC_ABI_ENTRY(mfoc_openloop_Reset)
#define mfoc_openloop_Start MFOC_ABI_ENTRY(mfoc_openloop_Start)
#define mfoc_openloop_SetSpeed MFOC_ABI_ENTRY(mfoc_openloop_SetSpeed)
#define mfoc_openloop_Step MFOC_ABI_ENTRY(mfoc_openloop_Step)
#define mfoc_sensorless_start_Init MFOC_ABI_ENTRY(mfoc_sensorless_start_Init)
#define mfoc_sensorless_start_Start \
    MFOC_ABI_ENTRY(mfoc_sensorless_start_Start)
#define mfoc_sensorless_start_Step MFOC_ABI_ENTRY(mfoc_sensorless_start_Step)

/* Board-specific entry points that carry configured control data. */
#define mfoc_port_init MFOC_ABI_ENTRY(mfoc_port_init)
#define mfoc_port_bus MFOC_ABI_ENTRY(mfoc_port_bus)
#define mfoc_port_position_run MFOC_ABI_ENTRY(mfoc_port_position_run)
#define mfoc_port_position MFOC_ABI_ENTRY(mfoc_port_position)
#define mfoc_port_position_calibrate_zero \
    MFOC_ABI_ENTRY(mfoc_port_position_calibrate_zero)
#define mfoc_port_current MFOC_ABI_ENTRY(mfoc_port_current)
#define mfoc_port_pwm MFOC_ABI_ENTRY(mfoc_port_pwm)
#define mfoc_port_pwm_start MFOC_ABI_ENTRY(mfoc_port_pwm_start)
#define mfoc_port_pwm_stop MFOC_ABI_ENTRY(mfoc_port_pwm_stop)

/* Feedback provider APIs. */
#define mfoc_encoder_Init MFOC_ABI_ENTRY(mfoc_encoder_Init)
#define mfoc_encoder_Update MFOC_ABI_ENTRY(mfoc_encoder_Update)
#define mfoc_encoder_CalibrateZero MFOC_ABI_ENTRY(mfoc_encoder_CalibrateZero)
#define mfoc_encoder_Step MFOC_ABI_ENTRY(mfoc_encoder_Step)
#define mfoc_hall_Init MFOC_ABI_ENTRY(mfoc_hall_Init)
#define mfoc_hall_OnEdge MFOC_ABI_ENTRY(mfoc_hall_OnEdge)
#define mfoc_hall_Step MFOC_ABI_ENTRY(mfoc_hall_Step)
#define mfoc_hfi_Init MFOC_ABI_ENTRY(mfoc_hfi_Init)
#define mfoc_hfi_Step MFOC_ABI_ENTRY(mfoc_hfi_Step)
#define mfoc_smo_Init MFOC_ABI_ENTRY(mfoc_smo_Init)
#define mfoc_smo_Reset MFOC_ABI_ENTRY(mfoc_smo_Reset)
#define mfoc_smo_Step MFOC_ABI_ENTRY(mfoc_smo_Step)
#define mfoc_flux_Init MFOC_ABI_ENTRY(mfoc_flux_Init)
#define mfoc_flux_Reset MFOC_ABI_ENTRY(mfoc_flux_Reset)
#define mfoc_flux_Step MFOC_ABI_ENTRY(mfoc_flux_Step)
#define mfoc_ekf_Init MFOC_ABI_ENTRY(mfoc_ekf_Init)
#define mfoc_ekf_Reset MFOC_ABI_ENTRY(mfoc_ekf_Reset)
#define mfoc_ekf_Step MFOC_ABI_ENTRY(mfoc_ekf_Step)

/* Identification APIs. */
#define mfoc_identification_Init MFOC_ABI_ENTRY(mfoc_identification_Init)
#define mfoc_identification_Start MFOC_ABI_ENTRY(mfoc_identification_Start)
#define mfoc_identification_Stop MFOC_ABI_ENTRY(mfoc_identification_Stop)
#define mfoc_identification_Run MFOC_ABI_ENTRY(mfoc_identification_Run)
#define mfoc_identification_GetStatus \
    MFOC_ABI_ENTRY(mfoc_identification_GetStatus)
#define mfoc_identification_GetResult \
    MFOC_ABI_ENTRY(mfoc_identification_GetResult)
#define mfoc_identification_Step MFOC_ABI_ENTRY(mfoc_identification_Step)
#define mfoc_identification_rs_Step \
    MFOC_ABI_ENTRY(mfoc_identification_rs_Step)
#define mfoc_identification_rs_Finish \
    MFOC_ABI_ENTRY(mfoc_identification_rs_Finish)
#define mfoc_identification_inductance_Step \
    MFOC_ABI_ENTRY(mfoc_identification_inductance_Step)
#define mfoc_identification_inductance_Finish \
    MFOC_ABI_ENTRY(mfoc_identification_inductance_Finish)
#define mfoc_identification_inductance_InitTiming \
    MFOC_ABI_ENTRY(mfoc_identification_inductance_InitTiming)
#define mfoc_identification_inductance_FinishPrepared \
    MFOC_ABI_ENTRY(mfoc_identification_inductance_FinishPrepared)
#define mfoc_identification_flux_Step \
    MFOC_ABI_ENTRY(mfoc_identification_flux_Step)
#define mfoc_identification_flux_Finish \
    MFOC_ABI_ENTRY(mfoc_identification_flux_Finish)
#define mfoc_identification_rotation_Init \
    MFOC_ABI_ENTRY(mfoc_identification_rotation_Init)
#define mfoc_identification_rotation_Step \
    MFOC_ABI_ENTRY(mfoc_identification_rotation_Step)
#define mfoc_identification_rotation_Finish \
    MFOC_ABI_ENTRY(mfoc_identification_rotation_Finish)
#define mfoc_identification_inertia_Block \
    MFOC_ABI_ENTRY(mfoc_identification_inertia_Block)
#define mfoc_identification_inertia_Finish \
    MFOC_ABI_ENTRY(mfoc_identification_inertia_Finish)
#endif
#endif
