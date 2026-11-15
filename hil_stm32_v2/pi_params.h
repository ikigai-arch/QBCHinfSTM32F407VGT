/* Auto-generated oleh gen_pi_params.m (29 Okt-1 Nov 2026, dari PI_QBC.mat & MPSOPI_QBC.mat user) -- JANGAN diedit manual. */
#ifndef PI_PARAMS_H
#define PI_PARAMS_H

#define CTRL_TS   0.0001
#define D_MIN     0.050000000000000003
#define D_MAX     0.84999999999999998

/* 0: PI respons frekuensi (Armaliany dkk., 2026) -- PI_QBC.mat */
#define PI_KP     0.00027135468755531397
#define PI_KI     0.0088029376660461282

/* 1: MPSO-PI (Guarnizo dkk., 2023) -- MPSOPI_QBC.mat */
#define MPSO_KP   0.0001557002546071472
#define MPSO_KI   0.020182990167171493

#define CTRL_SEL_DEFAULT 1   /* set awal setelah reset board; MATLAB bisa ganti (CMD 3) */

#endif
