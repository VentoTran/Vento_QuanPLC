
#ifndef _LOG_CODEC_
#define _LOG_CODEC_




#define R_CUR_mA    (0)
#define R_SPD_RPM   (1)
#define R_Duty      (2)
#define R_VCC_mV    (3)
#define R_ENA       (4)

#define R_Kp_R      (5)
#define R_Ki_R      (6)
#define R_Kp_C      (7)
#define R_Ki_C      (8)
#define R_REF_RPM   (9)
#define R_Ts        (10)
#define R_conv_mVpA (11)
#define R_conv_TpR  (12)

//---------------------------------------------------------------------------------------------

#define W_Kp_R      (0)
#define W_Ki_R      (1)
#define W_Kp_C      (2)
#define W_Ki_C      (3)
#define W_REF_RPM   (4)
#define W_Ts        (5)
#define W_conv_mVpA (6)
#define W_conv_TpR  (7)
#define W_ENA       (8)

#define W_APP_CON   (11)
#define W_APP_DIS   (12)
#define W_log       (13)
#define W_ack       (14)
#define W_req_retry (15)

//---------------------------------------------------------------------------------------------

#define offset_ID   (7)
#define offset_RW   (6)
#define offset_ER   (5)






#endif