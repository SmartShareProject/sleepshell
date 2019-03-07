

#include "rwip_config.h"     // SW configuration

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include <string.h>
#include "app.h"                     // Application Definitions
#include "app_task.h"                // application task definitions
#include "co_bt.h"
#include "prf_types.h"               // Profile common types definition
#include "arch.h"                    // Platform Definitions
#include "prf.h"
#include "rf.h"
#include "prf_utils.h"
#include "ke_timer.h"
#include "uart.h"
#include "gpio.h"

#include "BK3435_reg.h"
#include "lld_evt.h"
#include "lis3dh_drv.h"
#include "app_lis3dh.h"
#include "utc_clock.h"
#include "app_fff0.h"
#include "RomCallFlash.h"
#include "flash.h"

volatile uint32_t sys_flag = 0;
uint8_t szSensorBuf[cSensorBufLens];

uint8_t bSendIngData;
uint8_t bNeedSendPKTHead;

uint8_t bMSMeasure = 0x0;                   // 1: manual measure.
uint8_t bATMeasure = 0x0;
uint8_t uWaitNetDailyClk = 0x0;

uint8_t uWakeupCNT = 0x0;
uint8_t uUTCSetCount = 0x0;

uint8_t uLoseDataCNT = 0x0;
uint8_t uLoseDataThd = 0x50;    // 0x45;
uint8_t uLoseCNTThd = 0x03;
uint8_t uLoseTimeThd = 0x03;

uint16_t wBufInIndex;
uint16_t wBufOutIndex;

uint16_t wStartATMeaCNT = 0x0;
eUSEER_CHN eTXSensorDataCHN = cNone_CHN;
app_key_type app_key_state = ALL_KEY_IDLE;
//static unsigned char app_clear_bond_flag = 0;

//SENSOR_MSG sSensorLast;       
uint32_t DailyClock_Start = 22*3600UL;          // start time with timer measure
uint32_t DailyClock_End = 24*3600UL;            // end time with timer measure
uint32_t DailyClock_Start1 = 0x0;
uint32_t DailyClock_End1 = 7*3600UL;

//uint8_t uMSTimeRange;
uint32_t u32MS_UTCTime;                     // manual measure UTC Time.
UTCTime UTCTimeSec_Last;
UTCTimeStruct sUTCTime_LastSave;
UTCTime UTCTimeAdv = 0x0;
UTCTime UTCTimeATMea = 0x0;
UTCTimeStruct sUTCSetLast;
UTCTime UTCTimeBigDataDT = 0x0;
UTCTimeStruct sUTCTime_SaveFlash;
uint8_t *pSensorBuf;


static void app_gpio_int_cb(void);


/*
    Function: disable GPIO wakeup function.
*/
void GPIO_wakeup_clear(void)
{
    REG_APB5_GPIO_WUATOD_ENABLE = 0x00000000;       // 0: disable GPIO wakeup.
    REG_APB5_GPIO_WUATOD_STAT = 0xffffffff;
    REG_AHB0_ICU_INT_ENABLE &= (~(0x01 << 9));
}


/*
    Function: Configure key interrupt and hibernation
*/
void Lis3dhIRQ_wakeup_config(void)
{
    //Clear period timer
//  ke_timer_clear(APP_PERIOD_TIMER, TASK_APP);
    #ifdef  INT_ACTIVE_LOW
    REG_APB5_GPIO_WUATOD_TYPE |= 1<<(8*(P_LIS3DH_INT>>4)+(P_LIS3DH_INT&0x0f));      // 1:falling edge INT, 0: rising edge INT.
    #else
    REG_APB5_GPIO_WUATOD_TYPE &= ~(1<<(8*(P_LIS3DH_INT>>4)+(P_LIS3DH_INT&0x0f)));       // 1:falling edge INT, 0: rising edge INT.
    #endif
    REG_APB5_GPIO_WUATOD_STAT |= 1<<(8*(P_LIS3DH_INT>>4)+(P_LIS3DH_INT&0x0f));      // write 1: clear wakeup state
    Delay_ms(2);
    REG_APB5_GPIO_WUATOD_ENABLE |= 1<<(8*(P_LIS3DH_INT>>4)+(P_LIS3DH_INT&0x0f));    // 1: enable GPIO wakeup, 0: disable GPIO wakeup.
    REG_AHB0_ICU_DEEP_SLEEP0 |= 1<<(8*(P_LIS3DH_INT>>4)+(P_LIS3DH_INT&0x0f));
}

/*
    note: low power mode: output data 8bits.
            normal mode: output data 10bits.
            high resolution mode: output data 12bits

    return : 12bits data
*/
static void GetSensorData(SENSOR_DATA *pSensorData)
{
    uint16_t wTmp;
    uint8_t uTmp;
    uTmp = LIS3DH_GetXL();
    wTmp = LIS3DH_GetXH();
    wTmp <<= 8;
    wTmp |= uTmp;
//  UART_PRINTF("x=%x,",wTmp);
    pSensorData->x = (wTmp >> 6);   // left-aligned, (wTmp >> 2);
    
    uTmp = LIS3DH_GetYL();
    wTmp = LIS3DH_GetYH();
    wTmp <<= 8;
    wTmp |= uTmp;
//  UART_PRINTF("y=%x,",wTmp);
    pSensorData->y = (wTmp >> 6);   // (wTmp >> 2);

    uTmp = LIS3DH_GetZL();
    wTmp = LIS3DH_GetZH();
    wTmp <<= 8;
    wTmp |= uTmp;
//  UART_PRINTF("z=%x\r\n",wTmp);
    pSensorData->z = (wTmp >> 6);   // (wTmp >> 2);

    return;
}

void Lis3dh_process(void)
{
    SENSOR_DATA sensor;
    UTCTimeStruct time, timeTmp;
    uint16_t wTmp;
    UTCTime UTCTimeSec_Cur;
    uint8_t *pBufTmp, uTmp, uTmp1, uMonthTmp, uDayTmp;
    uint32_t dwCurTime, dwSensorACC;
    uint8_t bEnMeasure;
    
    Lis3dh_IRQ_Polling();
//  LIS3DH_Write_INT1_CFG();
    if(sys_flag & FLAG_KEY_ACTIVE)
    {
        GetSensorData(&sensor);
        utc_get_time(&time);
        UTCTimeSec_Cur= utc_get_clock();
//      if(sys_flag & FLAG_KEY_ACTIVE)
        {
            LIS3DH_ReadINT1_SRC();
//          UART_PRINTF("INTSrc=%x\r\n",uSrc);
        }
//      uSrc = LIS3DH_ReadREF();
//      LIS3DH_Write_INT1_CFG();
        //------------------------------------------------------------
        dwSensorACC = abs(sensor.x);
        dwSensorACC += abs(sensor.y);
        dwSensorACC += abs(sensor.z);
//      if(dwSensorACC == 0x0)
//          UART_PRINTF("SensorACC = %x\r\n", dwSensorACC);
        // wakeup advertising
        if(dwSensorACC > 0)
        {
            if(dwSensorACC >= uLoseDataThd) // 0x50 // 0x250)   // 0x1f0    // 0x600)               // >= 1.5G, +-2G range, resolution 12 bits. 
            {
                if(UTCTimeSec_Cur >= UTCTimeBigDataDT)
                {
                    if((UTCTimeSec_Cur - UTCTimeBigDataDT) <= 1)
                    {
                        if(++uLoseDataCNT >= uLoseCNTThd)               // 5
                        {
                            uLoseDataCNT = 0x0;
                            user_data.bDataLose = 1;
                            user_data.uLoseTime = uLoseTimeThd;
                            if(!ke_timer_active(APP_SER_DATALOSE_TIMER, TASK_APP))
                                ke_timer_set(APP_SER_DATALOSE_TIMER,TASK_APP,100);
                            UART_PRINTF("data lose\r\n");
                        }
                    }
                }
                else
                {
                    uLoseDataCNT = 0x1;
                    UTCTimeBigDataDT = UTCTimeSec_Cur;
                }
            }
                
            if ((ke_state_get(TASK_APP) != APPM_ADVERTISING) && (ke_state_get(TASK_APP) != APPM_CONNECTED))
            {
                #if 1
                if(dwSensorACC >= 0x70) // 0x70 0x50 // 0x250)  // 0x1f0    // 0x600)               // >= 1.5G, +-2G range, resolution 12 bits. 
                {
                    if(UTCTimeSec_Cur >= UTCTimeAdv)
                    {
                        if((UTCTimeSec_Cur - UTCTimeAdv) <= 2   ) // 1)             // 5
                        {
                            if(++uWakeupCNT >= 8)   // 5)                               // 4
                            {
                                appm_start_advertising();
                                if(!ke_timer_active(APP_SER_DATALOSE_TIMER, TASK_APP))
                                    ke_timer_set(APP_SER_DATALOSE_TIMER,TASK_APP,100);
                                UART_PRINTF("wakeup adv\r\n");
                            }
                        }
                        else
                        {
                            uWakeupCNT = 0x1;
                            UTCTimeAdv = UTCTimeSec_Cur;
                        }
                    }
                }
                #if 0
                else
                {
                    uWakeupCNT = 0x0;
                    UTCTimeAdv = UTCTimeSec_Cur;
                }
                #endif
                #else
                if((UTCTimeSec_Cur >= UTCTimeAdv) && ((UTCTimeSec_Cur - UTCTimeAdv) < 1))
                {
                    if(dwSensorACC >= 0x40) // 0x50
                    {
                        if(++uWakeupCNT >= 10)                              // 8
                        {
                            appm_start_advertising();
                            user_data.bDataLose = 1;
                            user_data.uLoseTime = 0x0a;
                            if(!ke_timer_active(APP_SER_DATALOSE_TIMER, TASK_APP))
                                ke_timer_set(APP_SER_DATALOSE_TIMER,TASK_APP,100);
                            UART_PRINTF("wakeup adv\r\n");
                        }
                    }
                }
                #endif
            }
            //------------------------------------------------------------
            dwCurTime = time.hour;
            dwCurTime *= 3600UL;
            dwCurTime += time.minutes*60UL;
            bEnMeasure = 0x0;
            if(bMSMeasure)      // manual measure
            {
                if((UTCTimeSec_Cur - u32MS_UTCTime) >= cMSOverTime)     // over 12 hour. manual measure stop.
                {
                    bMSMeasure = 0x0;
                    u32MS_UTCTime += cMSOverTime;                   // add 12 hour.
                    if((UTCTimeSec_Cur - u32MS_UTCTime) < 86400)        // < 24 hour
                    {
                        to_utc_time(&timeTmp, u32MS_UTCTime);
                        dwCurTime = timeTmp.hour;
                        dwCurTime *= 3600UL;
                        dwCurTime += timeTmp.minutes*60UL;
                        if((DailyClock_Start1 == 0x0) && (DailyClock_End1 == 0x0))  // end > start, same day
                        {
    //                      if((timeTmp.year == time.year) && (timeTmp.month == time.month) && (timeTmp.day == time.day))
                            {
                                if((dwCurTime >= DailyClock_Start) && (dwCurTime <= DailyClock_End))
                                {
                                    uWaitNetDailyClk = 0x1;
    //                              UART_PRINTF("MS over time\r\n");
                                }
                            }
                        }
                        else
                        {
                            if(dwCurTime <= DailyClock_End1)
                            {
                                uWaitNetDailyClk = 0x1;
                            }
                            else if((dwCurTime >= DailyClock_Start) && (dwCurTime <= DailyClock_End))
                            {
                                uWaitNetDailyClk = 0x2;
                            }
                        }
                        dwCurTime = time.hour;
                        dwCurTime *= 3600UL;
                        dwCurTime += time.minutes*60UL;
                    }
                    
    //              if(uWaitNetDailyClk)
    //                  UART_PRINTF("wait=%d\r\n", uWaitNetDailyClk);
                }
                else
                {
                    bEnMeasure = 0x01;
                }
            }
            else if(uWaitNetDailyClk)
            {
                if((UTCTimeSec_Cur - u32MS_UTCTime) < 86400)        // < 24 hour
                {
                    to_utc_time(&timeTmp, u32MS_UTCTime);
                    if((DailyClock_Start1 == 0x0) && (DailyClock_End1 == 0x0))
                    {
    //                  if((dwCurTime < DailyClock_Start) && (dwCurTime > DailyClock_End))
                        if((timeTmp.year != time.year) || (timeTmp.month != time.month) || (timeTmp.day != time.day))
                        {
                            uWaitNetDailyClk = 0x0;
                            UART_PRINTF("cancel diff day\r\n");
                        }
                        else
                        {
                            if(dwCurTime > DailyClock_End)
                            {
                                uWaitNetDailyClk = 0x0;
                                UART_PRINTF("cancel>end time\r\n");
                            }
                        }

                    }
                    else
                    {
                        if(uWaitNetDailyClk == 1)
                        {               
                            if((timeTmp.year == time.year) && (timeTmp.month == time.month) && (timeTmp.day == time.day))
                            {
                                if(dwCurTime > DailyClock_End1)
                                {
                                    uWaitNetDailyClk = 0x0;
                                    UART_PRINTF("cancel>end time1\r\n");
                                }
                            }
                            else
                            {
                                uWaitNetDailyClk = 0x0;
                                UART_PRINTF("cancel>end time1\r\n");
                            }
                        }
                        else
                        {           
                            if((timeTmp.year != time.year) || (timeTmp.month != time.month) || (timeTmp.day != time.day))
                            {
                                if(dwCurTime > DailyClock_End1)
                                {
                                    uWaitNetDailyClk = 0x0;
                                    UART_PRINTF("cancel>end time1\r\n");
                                }
                            }
                        }
                    }
                }
                else
                {
                    uWaitNetDailyClk = 0x0;
                }

            }
            else
            {
                if((DailyClock_Start1 == 0x0) && (DailyClock_End1 == 0x0))
                {
                    if((dwCurTime >= DailyClock_Start) && (dwCurTime <= DailyClock_End))
                    {
                        bEnMeasure = 0x01;

                        bATMeasure = 0x0;
                    }
                    else
                    {
                        // DailyClock_Start < DailyClock_End
                        if(bATMeasure == 0x0)
                        {
                            if(dwCurTime < DailyClock_Start)
                            {
                                if((DailyClock_Start <= 0x2A30) || \
                                    ((DailyClock_Start > 0x2A30) && (dwCurTime >= (DailyClock_Start - 0x2A30))))    // 0x2A30: 3hour.
                                {
                                    if(UTCTimeSec_Cur >= UTCTimeATMea)
                                    {
                                        if((UTCTimeSec_Cur - UTCTimeATMea) < 60)    // 60: 1 minute
                                        {
                                            if(dwSensorACC > 0x10)
                                            {
                                                if(++wStartATMeaCNT > 300)      // 300*20ms = 6s
                                                {
                                                    wStartATMeaCNT = 0x0;
                                                    bATMeasure = 0x01;
                                                }
                                            }
                                        }
                                        else
                                        {
                                            UTCTimeATMea = UTCTimeSec_Cur;
                                            wStartATMeaCNT = 0x0;
                                        }
                                    }
                                    else
                                    {
                                        UTCTimeATMea = UTCTimeSec_Cur;
                                        wStartATMeaCNT = 0x0;
                                    }
                                }
                            }
                            else if(dwCurTime > DailyClock_End)
                            {
                                if((DailyClock_End >= 0x12750) ||\
                                    ((DailyClock_End > 0x2A30) && (dwCurTime <= (DailyClock_End - 0x2A30))))    // 0x12750: 21 clock
                                {
                                    bATMeasure = 0x01;
                                }
                            }
                        }
                        else
                        {
                            if(dwCurTime < DailyClock_Start)    
                            {
                                if((DailyClock_Start > 0x2A30) && (dwCurTime < (DailyClock_Start - 0x2A30)))
                                    bATMeasure = 0x0;
                            }
                            else if(dwCurTime > DailyClock_End)
                            {
                                if((DailyClock_End < 0x12750) && (dwCurTime > (DailyClock_End - 0x2A30)))
                                    bATMeasure = 0x0;
                            }
                            else
                            {
                                bATMeasure = 0x0;
                            }
                        }
                        if(bATMeasure == 0x01)
                            bEnMeasure = 0x01;
                    }
                }
                else
                {
                    // DailyClock_Start > DailyClock_End1
                    if(((dwCurTime >= DailyClock_Start) && (dwCurTime <= DailyClock_End)) \
                    || ((dwCurTime >= DailyClock_Start1) && (dwCurTime <= DailyClock_End1)))
                    {
                        bEnMeasure = 0x01;

                        bATMeasure = 0x0;
                    }
                    else
                    {
                        if(bATMeasure == 0x0)
                        {
                            if((DailyClock_Start - DailyClock_End1) > 0x5460)   //6hour,  0x2A30)       // > 3hour
                            {
                                if(dwCurTime < DailyClock_Start)
                                {
                                    if(dwCurTime > DailyClock_End1)
                                    {
                                        if(dwCurTime < (DailyClock_End1 + 0x2A30))
                                            bATMeasure = 0x1;
                                        else if(dwCurTime > (DailyClock_Start - 0x2A30))    // (dwCurTime < (DailyClock_Start - 0x2A30))
                                        {
                                            if(UTCTimeSec_Cur >= UTCTimeATMea)
                                            {
                                                if((UTCTimeSec_Cur - UTCTimeATMea) < 60)    // 60: 1 minute
                                                {
                                                    if(dwSensorACC > 0x10)
                                                    {
                                                        if(++wStartATMeaCNT > 300)      // 300*20ms = 6s
                                                        {
                                                            wStartATMeaCNT = 0x0;
                                                            bATMeasure = 0x01;
                                                        }
                                                    }
                                                }
                                                else
                                                {
                                                    UTCTimeATMea = UTCTimeSec_Cur;
                                                    wStartATMeaCNT = 0x0;
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                            else
                                bATMeasure = 0x1;
                        }
                        else
                        {
                            if((DailyClock_Start - DailyClock_End1) > 0x5460)   //  0x2A30)
                            {
                                if((dwCurTime > (DailyClock_End1 + 0x2A30)) && (dwCurTime < (DailyClock_Start - 0x2A30)))
                                    bATMeasure = 0x0;
                            }
                        }
                        
                        if(bATMeasure == 0x01)
                            bEnMeasure = 0x01;
                    }
                }
                
            }
            
            if((bEnMeasure) && (user_data.bDataLose == 0x0))
            {
                if(((abs(UTCTimeSec_Cur - UTCTimeSec_Last)) >= 120) || (wBufInIndex == 0x0))    // 120 second, 2 minute.
                {
                    UTCTimeSec_Last = UTCTimeSec_Cur;
                    if(wBufInIndex == 0x0)
                        pSensorBuf = szSensorBuf;
                    else
                    {
                        if((cSensorBufLens - wBufInIndex) >= 300)   // 798)
                        {
                            pSensorBuf += cRawPktUnitNum;
                        }
                        else
                        {
                            // buffer too small, push one day data.
                            if(szSensorBuf[0] == 0xA0)
                            {
                                uTmp1 = szSensorBuf[1];     // month
                                uTmp = szSensorBuf[2];      // day
                                pBufTmp = szSensorBuf;
                                pBufTmp += cRawPktUnitNum;
                                for(wTmp = cRawPktUnitNum; wTmp < (wBufInIndex - cRawPktUnitNum); )
                                {
                                    if(*pBufTmp == 0xA0)
                                    {
//                                      UART_PRINTF("*pBufTmp=%x\r\n",pBufTmp);
                                        pBufTmp ++;
                                        uMonthTmp = *pBufTmp;
                                        pBufTmp ++;
                                        uDayTmp = *pBufTmp;
                                        if((uMonthTmp == uTmp1) && (uDayTmp == uTmp))
                                        {
                                            wTmp += cRawPktUnitNum;
                                            pBufTmp += (cRawPktUnitNum-2);
//                                          UART_PRINTF("dayTmp=%d\r\n",uDayTmp);
                                        }
                                        else
                                        {
                                            pBufTmp -= 2;
//                                          UART_PRINTF("*pBufTmp=%x\r\n",*pBufTmp);
                                            break;
                                        }
                                    }
                                    else
                                    {
                                        wTmp += cRawPktUnitNum;
                                        pBufTmp += cRawPktUnitNum;
                                    }
                                }

                                if((wTmp > cRawPktUnitNum) && (wTmp < wBufInIndex))
                                {
                                    wBufInIndex -= wTmp;
                                    for(wTmp = 0x0; wTmp < wBufInIndex; wTmp++)
                                    {
                                        szSensorBuf[wTmp] = *pBufTmp;
                                        pBufTmp++;
                                    }
                                    pSensorBuf = szSensorBuf;
                                    pSensorBuf += wBufInIndex;
                                    app_SaveSensorDataToFlash(wBufInIndex, szSensorBuf);
                                    UART_PRINTF("Delect one day data\r\n");
                                }
                                else
                                {
                                    app_ClearSaveSensorDataToFlash();
                                    UART_PRINTF("Buf error\r\n");
                                }
                            }
                            else
                            {
                                app_ClearSaveSensorDataToFlash();
                                UART_PRINTF("Buf error1\r\n");
                            }
                        }
                    }
                    if((sUTCTime_LastSave.month != time.month) || (sUTCTime_LastSave.day != time.day) \
                        || (sUTCTime_LastSave.hour != time.hour) || (wBufInIndex == 0x0))
                    {                   
                        *pSensorBuf = 0xA0;
                        pSensorBuf++;
                        *pSensorBuf = time.month;
                        pSensorBuf++;
                        *pSensorBuf = time.day;
                        pSensorBuf++;
                        *pSensorBuf = time.hour;
                        pSensorBuf++;

                        wTmp = time.year + 2000;
                        *pSensorBuf = (uint8_t)(wTmp >> 8); // 0x0;
                        pSensorBuf++;
                        *pSensorBuf = (uint8_t)wTmp;    // 0x0;
                        pSensorBuf++;
                        
                        sUTCTime_LastSave.month = time.month;
                        sUTCTime_LastSave.day = time.day;
                        sUTCTime_LastSave.hour = time.hour;
                        if((sUTCTime_SaveFlash.month != sUTCTime_LastSave.month) || (sUTCTime_SaveFlash.day != sUTCTime_LastSave.day))
                        {
                            if(wBufInIndex > 0)
                            {
                                app_SaveSensorDataToFlash(wBufInIndex, szSensorBuf);
                            }
                        }
                        wBufInIndex += 6;
    //                  if(wBufInIndex >= cSensorBufLens)
    //                      wBufInIndex = cSensorBufLens;

                        UART_PRINTF("year=%d,month=%d, day=%d, hour=%d\r\n",time.year, time.month, time.day, time.hour);
                    }
                    pBufTmp = pSensorBuf;
                    *pBufTmp = time.minutes;
                    pBufTmp++;

                    *pBufTmp = 0x0;         // count H.
                    pBufTmp++;
                    *pBufTmp = 0x01;            // count L.
                    pBufTmp++;

                    dwSensorACC = abs(sensor.x);
                    dwSensorACC += abs(sensor.y);
                    dwSensorACC += abs(sensor.z);

                    *pBufTmp = (uint8_t)(dwSensorACC >> 16);
                    pBufTmp++;

                    *pBufTmp = (uint8_t)(dwSensorACC >> 8);
                    pBufTmp++;

                    *pBufTmp = (uint8_t)(dwSensorACC);
                    wBufInIndex += 6;
    //              if(wBufInIndex >= cSensorBufLens)
    //                  wBufInIndex = cSensorBufLens;
                    //--------------------------------------uart debug
                    #if 1
                    pBufTmp = pSensorBuf;
                    pBufTmp -= 6;
                    if(*pBufTmp == 0xA0)
                    {
                        pBufTmp++;
                        UART_PRINTF("L_month=%d,",*pBufTmp);
                        pBufTmp++;
                        UART_PRINTF("L_day=%d,",*pBufTmp);
                        pBufTmp++;
                        UART_PRINTF("L_hour=%d\r\n",*pBufTmp);
                    }
                    else
                    {
                        
                        UART_PRINTF("minute=%d\r\n",*pBufTmp);
                        pBufTmp++;
                        wTmp = *pBufTmp;
                        wTmp <<= 8;
                        
                        pBufTmp++;
                        wTmp |= *pBufTmp;
                        UART_PRINTF("count=%x\r\n",wTmp);
                        pBufTmp++;
                        dwSensorACC = *pBufTmp;
                        dwSensorACC <<= 8;

                        pBufTmp++;
                        dwSensorACC |= *pBufTmp;
                        dwSensorACC <<= 8;
                    
                        pBufTmp++;
                        dwSensorACC |= *pBufTmp;
                        UART_PRINTF("SensorAcc=%x\r\n",dwSensorACC);
                    }
                    #endif
                }
                else
                {
                    pBufTmp = pSensorBuf;       // minute
                    
                    pBufTmp++;          
                    wTmp = *pBufTmp;            // count H
                    wTmp <<= 8;
                    
                    pBufTmp++;                  // count L
                    wTmp |= *pBufTmp;
                    wTmp++;                 // count++;
                    
                    pBufTmp = pSensorBuf;
                    
                    pBufTmp++;
                    *pBufTmp = (uint8_t)(wTmp >> 8);
                    
                    pBufTmp++;
                    *pBufTmp = (uint8_t)(wTmp);
                    //-----------------
                    pBufTmp++;
                    dwSensorACC = *pBufTmp;
                    dwSensorACC <<= 8;

                    pBufTmp++;
                    dwSensorACC |= *pBufTmp;
                    dwSensorACC <<= 8;
                    
                    pBufTmp++;
                    dwSensorACC |= *pBufTmp;

                    dwSensorACC += abs(sensor.x);
                    dwSensorACC += abs(sensor.y);
                    dwSensorACC += abs(sensor.z);
                    pBufTmp -= 2;

                    *pBufTmp = (uint8_t)(dwSensorACC >> 16);
                    pBufTmp++;

                    *pBufTmp = (uint8_t)(dwSensorACC >> 8);
                    pBufTmp++;

                    *pBufTmp = (uint8_t)(dwSensorACC);
                }
            }
    //      UART_PRINTF("d=%x, h=%x, m=%x, s=%x\r\n",sensorMsg.day, sensorMsg.hour, sensorMsg.min, sensorMsg.sec);
        }
    }
}


void app_SysUTCInit(void)
{
    UTCTimeStruct sUTC;
    sUTC.year = 18;

    sUTC.month = 1;
    sUTC.day = 1;
    sUTC.hour = 12;
    sUTC.minutes = 0;
    sUTC.seconds = 0;
    utc_set_time(&sUTC);

//  sUTCTime_SaveFlash.month = 0x0;
//  sUTCTime_SaveFlash.day = 0x0;
}
/*
    B0: Command, B1: Lens, B2~Bn: data, B[end]: CS
*/
void app_UserCommandParse(eUSEER_CHN uPrfType, uint8_t *pBuf, uint8_t uLens)
{
    uint8_t buf[20];
    UTCTimeStruct sUTC;
    uint32_t DailyClock_StartTmp;
    uint32_t DailyClock_EndTmp, dwCurTime;
    uint16_t wYearTmp, wInIndexBak;
    UTCTimeStruct time, time1;  // , timeTmp;
    memcpy(buf, pBuf, uLens);
    if(buf[0] == cSetUTC)           // 50 08 00 12 08 0e 08 00 58
    {
        if(bSendIngData == 0x0)
        {
            wYearTmp = buf[2];      // year high byte
            wYearTmp <<= 8;
            wYearTmp |= buf[3];     // year low byte
            
            if((wYearTmp >= 2000) && ((buf[4] <= 12) && (buf[4] > 0)) && ((buf[5] <= 31) && (buf[5] > 0)) \
                && (buf[6] <= 23) && (buf[7] <= 59) && (buf[8] <= 59))
            {
                sUTC.year = wYearTmp - 2000;

                sUTC.month = buf[4];
                sUTC.day = buf[5];
                sUTC.hour = buf[6];
                sUTC.minutes = buf[7];
                sUTC.seconds = buf[8];
                utc_set_time(&sUTC);

                sUTCSetLast.year = sUTC.year;
                sUTCSetLast.month = sUTC.month;
                sUTCSetLast.day = sUTC.day;
                sUTCSetLast.hour = sUTC.hour;
                sUTCSetLast.minutes = sUTC.minutes;
                sUTCSetLast.seconds = sUTC.seconds;

                bATMeasure = 0x0;
                buf[1] = 0x01;
                if(uUTCSetCount < 0xff)
                    uUTCSetCount++;

            }
            else
            {
                buf[1] = 0x00;
            }
            app_SendSensorData_fff1(buf, 2);
        }
    }
    else if(buf[0] == cSetClock)
    {
        if(bSendIngData == 0x0)
        {
            if((buf[2] <= 23) && (buf[5] <= 23) && (buf[3] <= 59) \
                && (buf[4] <= 59) && (buf[6] <= 59) && (buf[7] <= 59))
            {
                DailyClock_StartTmp = buf[2]*3600UL;
                DailyClock_StartTmp += buf[3]*60UL;
                DailyClock_StartTmp += buf[4];

                DailyClock_EndTmp = buf[5]*3600UL;
                DailyClock_EndTmp += buf[6]*60UL;
                DailyClock_EndTmp += buf[7];

                if(DailyClock_StartTmp != DailyClock_EndTmp)
                {
                    if(DailyClock_StartTmp > DailyClock_EndTmp)
                    {
                        DailyClock_Start = DailyClock_StartTmp;
                        DailyClock_End = 86400UL;
                        DailyClock_Start1 = 0x0;
                        DailyClock_End1 = DailyClock_EndTmp;                    // >= 24hour

                        bATMeasure = 0x0;
                        buf[1] = 0x01;
                    }
                    else if((DailyClock_EndTmp - DailyClock_StartTmp) >= 900UL) // 15min
                    {
                        DailyClock_Start = DailyClock_StartTmp;
                        DailyClock_End = DailyClock_EndTmp;
                        DailyClock_Start1 = 0x0;
                        DailyClock_End1 = 0x0;

                        bATMeasure = 0x0;
                        buf[1] = 0x01;
                    }
                    else
                    {
                        buf[1] = 0x00;
                    }
                        
                }
                else
                {
                    buf[1] = 0x00;
                }
            }
            else
            {
                buf[1] = 0x00;
            }

            if(buf[1] == 0x0)
                UART_PRINTF("clock set error\r\n");
            app_SendSensorData_fff1(buf, 2);
        }
    }
    else if(buf[0] == cStartMeasure)
    {
        if(bMSMeasure == 0)
        {
//          uMSTimeRange = 0x1;
            utc_get_time(&time);
            u32MS_UTCTime = utc_get_clock();
            dwCurTime = time.hour;
            dwCurTime *= 3600UL;
            dwCurTime += time.minutes*60UL;
            wInIndexBak = wBufInIndex;
            if((DailyClock_Start1 == 0x0) && (DailyClock_End1 == 0x0))      // 判断循环定时是否跨天
            {
                if((dwCurTime >= DailyClock_Start) && (dwCurTime <= DailyClock_End))
                 {          // current day
                    while(1)
                    {
                        if(wBufInIndex < 6)
                        {
//                          wBufInIndex = 0x0;
//                          pSensorBuf = szSensorBuf;
                            app_ClearSaveSensorDataToFlash();
                            app_SensorBufPopInit();
//                          app_DataTimeMarkInit();
                            app_SensorBufPushInit();
                            break;
                        }
                        wBufInIndex -= 6;
                        if(szSensorBuf[wBufInIndex] == 0xA0)
                        {
                            wYearTmp = szSensorBuf[wBufInIndex+4];      // year high byte
                            wYearTmp <<= 8;
                            wYearTmp |= szSensorBuf[wBufInIndex+5];     // year low byte
                            if(wYearTmp > 2000)
                                wYearTmp -= 2000;
                            else
                                wYearTmp = 0x0;
                            #if 0
                            if((wYearTmp < time.year) || (szSensorBuf[wBufInIndex+1] < time.month) || \
                                ((szSensorBuf[wBufInIndex+1] == time.month) && (szSensorBuf[wBufInIndex+2] < time.day)))
                            #endif
                            if((wYearTmp != time.year) || (szSensorBuf[wBufInIndex+1] != time.month) || \
                                (szSensorBuf[wBufInIndex+2] != time.day))
                            {
                                wBufInIndex = wInIndexBak;
                                pSensorBuf = szSensorBuf;
                                pSensorBuf += wBufInIndex;
                                pSensorBuf -= cRawPktUnitNum;
                                break;
                            }
                            else
                            {
                                wInIndexBak = wBufInIndex;
                                app_DataTimeMarkInit();
                            }
                        }
                    }
                 }
            }
            else
            {                   
                if((dwCurTime >= DailyClock_Start) && (dwCurTime <= DailyClock_End))
                {               
//                  uMSTimeRange = 0x2;
                    while(1)
                    {
                        if(wBufInIndex < 6)
                        {
                            app_ClearSaveSensorDataToFlash();
//                          wBufInIndex = 0x0;
//                          pSensorBuf = szSensorBuf;
                            app_SensorBufPopInit();
//                          app_DataTimeMarkInit();
                            app_SensorBufPushInit();
                            break;
                        }
                        wBufInIndex -= 6;
                        if(szSensorBuf[wBufInIndex] == 0xA0)
                        {
                            wYearTmp = szSensorBuf[wBufInIndex+4];      // year high byte
                            wYearTmp <<= 8;
                            wYearTmp |= szSensorBuf[wBufInIndex+5];     // year low byte
                            if(wYearTmp > 2000)
                                wYearTmp -= 2000;
                            else
                                wYearTmp = 0x0;
                            #if 0
                            if((wYearTmp < time.year) || (szSensorBuf[wBufInIndex+1] < time.month) || \
                                ((szSensorBuf[wBufInIndex+1] == time.month) && (szSensorBuf[wBufInIndex+2] < time.day)))
                            #endif
                            if((wYearTmp != time.year) || (szSensorBuf[wBufInIndex+1] != time.month) || \
                                (szSensorBuf[wBufInIndex+2] != time.day))
                            {
                                wBufInIndex = wInIndexBak;
                                pSensorBuf = szSensorBuf;
                                pSensorBuf += wBufInIndex;
                                pSensorBuf -= cRawPktUnitNum;
                                break;
                            }
                            else
                            {
                                wInIndexBak = wBufInIndex;
                                app_DataTimeMarkInit();
                            }
                        }
                    }
                }
                else if((dwCurTime >= DailyClock_Start1) && (dwCurTime <= DailyClock_End1))
                 {          
                    time1.year = time.year;
                    if(time.day > 1)
                    {
                        time1.day = time.day - 1;
                    }
                    else
                    {
                        if(time.month > 1)
                        {
                            time1.month = time.month - 1;
                            if(time1.month <= 7)
                            {
                                if(time1.month == 2)
                                {
                                    if(CheckLeapYear(time1.year))
                                    {
                                        time1.day = 29;
                                    }
                                    else
                                        time1.day = 28;
                                }
                                else
                                {
                                    if(time1.month & 0x01)
                                    {
                                        time1.day = 31;
                                    }
                                    else
                                    {
                                        time1.day = 30;
                                    }
                                }
                            }
                            else
                            {
                                if(time1.month & 0x01)
                                {
                                    time1.day = 30;
                                }
                                else
                                {
                                    time1.day = 31;
                                }
                            }
                        }
                        else
                        {
                            if(time1.year >= 1)
                            {
                                time1.year--;
                                time1.month = 12;
                                time1.day = 31;
                            }
                            else
                            {
                                time1.year=0x0;
                                time1.month = 1;
                                time1.day = 1;
                            }
                        }
                    }
                    
                    while(1)
                    {
                        if(wBufInIndex < 6)
                        {
//                          wBufInIndex = 0x0;
//                          pSensorBuf = szSensorBuf;
                            app_ClearSaveSensorDataToFlash();
                            app_SensorBufPushInit();
                            app_SensorBufPopInit();
//                          app_DataTimeMarkInit();
                            break;
                        }
                        wBufInIndex -= 6;
                        if(szSensorBuf[wBufInIndex] == 0xA0)
                        {
                            wYearTmp = szSensorBuf[wBufInIndex+4];      // year high byte
                            wYearTmp <<= 8;
                            wYearTmp |= szSensorBuf[wBufInIndex+5];     // year low byte
                            if(wYearTmp > 2000)
                                wYearTmp -= 2000;
                            else
                                wYearTmp = 0x0;
                            #if 0
                            if((wYearTmp < time.year) ||(szSensorBuf[wBufInIndex+1] < time.month) || \
                                ((szSensorBuf[wBufInIndex+1] == time.month) && (szSensorBuf[wBufInIndex+2] < time.day)))
                            #endif
//                          UART_PRINTF("y=%d, m=%d, d=%d; ms=%d, ds=%d\r\n", time1.year, time1.month, time1.day, szSensorBuf[wBufInIndex+1], szSensorBuf[wBufInIndex+2]);
                            if(((wYearTmp != time.year) && (wYearTmp != time1.year)) || \
                                ((szSensorBuf[wBufInIndex+1] != time.month) && (szSensorBuf[wBufInIndex+1] != time1.month)) || \
                                ((szSensorBuf[wBufInIndex+2] != time.day) && (szSensorBuf[wBufInIndex+2] != time1.day)))
                            {
                                wBufInIndex = wInIndexBak;
                                pSensorBuf = szSensorBuf;
                                pSensorBuf += wBufInIndex;
                                pSensorBuf -= cRawPktUnitNum;
                                break;
                            }
                            else
                            {
                                wInIndexBak = wBufInIndex;
                                app_DataTimeMarkInit();
                            }
                        }
                    }

                }
             
             }
            bMSMeasure = 1;
            uWaitNetDailyClk = 0x0;
            if(bSendIngData == 0x0)
            {
                buf[1] = 0x01;
                app_SendSensorData_fff1(buf, 2);
            }
        }
        else
        {
            if(bSendIngData == 0x0)
            {
                buf[1] = 0x0;
                app_SendSensorData_fff1(buf, 2);
            }
        }
    }
    else if(buf[0] == cStopMeasure)
    {
        if(bMSMeasure)
        {
            utc_get_time(&time);
            dwCurTime = time.hour;
            dwCurTime *= 3600UL;
            dwCurTime += time.minutes*60UL;
            to_utc_time(&time1, u32MS_UTCTime);
            if((DailyClock_Start1 == 0x0) && (DailyClock_End1 == 0x0))      
            {
//              if(time.day == time1.day)
                {
                    if((dwCurTime >= DailyClock_Start) && (dwCurTime <= DailyClock_End))
                    {
                        uWaitNetDailyClk = 0x1;
                    }
                }
            }
            else
            {
                if((dwCurTime >= DailyClock_Start) && (dwCurTime <= DailyClock_End))
                {
                    uWaitNetDailyClk = 0x2;
                }
                else if(dwCurTime <= DailyClock_End1)
                {
                    uWaitNetDailyClk = 0x1;
                }
            }
            if(uWaitNetDailyClk)
            {
                u32MS_UTCTime = utc_get_clock();            // stop time.
                UART_PRINTF("wait=%d\r\n", uWaitNetDailyClk);
            }
        }
        bMSMeasure = 0;
        if(bSendIngData == 0x0)
        {
            buf[1] = 0x01;
            app_SendSensorData_fff1(buf, 2);
        }
    }
    else if(buf[0] == cInitSensorBuf)
    {
        app_ClearSaveSensorDataToFlash();
        if(bSendIngData == 0x0)
        {
            buf[1] = 0x01;
            app_SendSensorData_fff1(buf, 2);
        }
    }
    else if(buf[0] == cRDUserData)
    {
        app_SetSensorDataPrf(uPrfType);
        if(bSendIngData == 0x0)
        {
            bSendIngData = 0x01;
            bNeedSendPKTHead = 0x01;
        }
        if(app_GetSensorDataPrf() == cFFF1_CHN)
            UART_PRINTF("prf = fff1\r\n");
        else if(app_GetSensorDataPrf() == cWechat_CHN)
            UART_PRINTF("prf = wechat\r\n");
        else
            UART_PRINTF("prf = error\r\n");
    }
    else if(buf[0] == cSetDataThd)
    {
        if(bSendIngData == 0x0)
        {   
            buf[1] = buf[2];
            buf[1] += buf[3];
            buf[1] += buf[4];
            buf[1] ^= 0x5a;
            if(buf[2] == buf[1])
            {
                uLoseDataThd = buf[2];
                uLoseCNTThd = buf[3];
                uLoseTimeThd = buf[4];
                buf[1] = 0x01;
            }
            else
                buf[1] = 0x00;
            app_SendSensorData_fff1(buf, 2);
        }
    }
    else if(buf[0] == cReadUTC)
    {
        if(bSendIngData == 0x0)
        {
            utc_get_time(&sUTC);
            buf[1] = 7;
            buf[2] = (uint8_t)(sUTC.year >> 8);
            buf[3] = (uint8_t)(sUTC.year);
            buf[4] = sUTC.month;
            buf[5] = sUTC.day;
            buf[6] = sUTC.hour;
            buf[7] = sUTC.minutes;
            buf[8] = sUTC.seconds;
            buf[9] = (uint8_t)(user_data.wBatADC >> 8);
            buf[10] = (uint8_t)(user_data.wBatADC);
            buf[11] = uLoseDataThd; // uUTCSetCount;
            buf[12] = (uint8_t)(sUTCSetLast.year >> 8);
            buf[13] = (uint8_t)(sUTCSetLast.year);
            buf[14] = sUTCSetLast.month;
            buf[15] = sUTCSetLast.day;
            buf[16] = sUTCSetLast.hour;
            buf[17] = sUTCSetLast.minutes;
            buf[18] = sUTCSetLast.seconds;
            app_SendSensorData_fff1(buf, 19);
        }
    }
    #if 0
    else if(buf[0] == cDEBUGPOP)
    {
        wInIndexBak = buf[2];
        wInIndexBak <<= 8;
        wInIndexBak |= buf[3];

        if(bSendIngData == 0x0)
        {
            buf[1] = 0x01;
            buf[2] = (uint8_t)(wBufInIndex >> 8);
            buf[3] = (uint8_t)(wBufInIndex);
            app_SendSensorData_fff1(buf, 4);
        }
        wBufInIndex = wInIndexBak;
    }
    #endif
}


void app_SendSensorData(void)
{
    uint16_t wLens, wMaxLens;   //, wOutIndex, wPakNum;
    uint8_t szBuf[cRawPktUnitNum*cAirPktGroupNum+1];
    uint8_t i;  //, uPakNum;    //, uPakIndex;
    if(bSendIngData == 0x01)
    {
        if(app_GetSensorDataPrf() == cFFF1_CHN)     // app send data
        {   
            if(bNeedSendPKTHead)
            {
                szBuf[0] = cMsgPacketHead;              // packet head.
                szBuf[1] = (uint8_t)(wBufInIndex >> 8);
                szBuf[2] = (uint8_t)(wBufInIndex);
                szBuf[3] = 0x0;
                szBuf[4] = 0x0;
                szBuf[5] = 0x0;
                for(i = 0; i < 4; i++)
                {
                    szBuf[5] += szBuf[1+i];
                }
                szBuf[5]  ^= 0x5a;
                if(app_SendSensorData_fff1(szBuf, 6))
                {
                    bNeedSendPKTHead = 0x0;
                    if(wBufInIndex == 0x0)
                    {
                        bSendIngData = 0x0;
                        app_SensorBufPopInit();
                        UART_PRINTF("buf size=0x0, end\r\n");
                    }
                    else
                    {
                        UART_PRINTF("buf size=0x%x, start tx\r\n",wBufInIndex);
                    }
                    
                }

            }
            else if(wBufInIndex > wBufOutIndex)
            {   
                if(user_data.uMTUSizeActual == 23)
                    wMaxLens = cRawPktUnitNum*cAirPktGroupNumDefault;
                else
                    wMaxLens = cRawPktUnitNum*cAirPktGroupNum;
                wLens = wBufInIndex -wBufOutIndex;
                if(wLens >= wMaxLens)
                    wLens = wMaxLens;
                szBuf[0] = cDataPacketHead; // packet head.
                memcpy(&szBuf[1], &szSensorBuf[wBufOutIndex], wLens);
                if(app_SendSensorData_fff1(szBuf, (uint8_t)(wLens+1)))
                {
                    wBufOutIndex += wLens;
//                  UART_PRINTF("IOutdex=%x\r\n",wBufOutIndex);
                }

            }
            else
            {
                app_SendDataInit();
                UART_PRINTF("tx finished\r\n");
            }
        }
        else    if(app_GetSensorDataPrf() == cWechat_CHN)   // wechat send data
        {
            if(bNeedSendPKTHead)
            {
            
            }
            else
            {
                if(wBufInIndex > wBufOutIndex)
                {

                }
                else //if(wBufInIndex != 0x0)
                {
                    app_SendDataInit();
                }
            }
        }
    }
}

void app_ReadSensorDataFromFlash(void)
{
    UTCTimeStruct sUTC;
    uint8_t szBuf[12], uTmp, bFirstPktType;
    uint32_t dwTmp, dwLens;

    app_SensorBufPushInit();

    sUTCTime_SaveFlash.month = 0x0;
    sUTCTime_SaveFlash.day = 0x0;
    
    bFirstPktType = 0x0;
    dwLens = 0x0;
    flash_read(0x0, cFlashAddr_MSG, 12, szBuf, NULL);
    if((szBuf[0] == 'S') && (szBuf[1] == 'D') && (szBuf[2] == 'A') && (szBuf[3] == 'T'))
    {
        dwTmp = 0x0;
        for(uTmp = 0x0; uTmp < 8; uTmp++)
        {
            dwTmp += szBuf[uTmp];
        }

        dwLens = szBuf[8];
        dwLens <<= 8;
        dwLens |= szBuf[9];
        
        dwLens <<= 8;
        dwLens |= szBuf[10];

        dwLens <<= 8;
        dwLens |= szBuf[11];

        if(dwLens == dwTmp)
        {
            dwLens = szBuf[4];
            dwLens <<= 8;
            dwLens |= szBuf[5];
            
            dwLens <<= 8;
            dwLens |= szBuf[6];

            dwLens <<= 8;
            dwLens |= szBuf[7];

            if((dwLens > 0x0) && (dwLens <= cSensorBufLens) && ((dwLens % 6) == 0x0))
            {
                flash_read(0x0, cFlashAddr_Data, dwLens, szSensorBuf, NULL);
            }
            else
                dwLens = 0x0;
        }
        else
            dwLens = 0x0;

        UART_PRINTF("sensor data lens=0x%x\r\n", dwLens);
    }
    else
    {
        UART_PRINTF("user flash empty\r\n");
    }
    wBufInIndex = (uint16_t)dwLens;
    pSensorBuf = szSensorBuf;
    pSensorBuf += wBufInIndex;
    
    sUTC.year = 0;
    sUTC.seconds = 0;

    if(dwLens)
    {
        pSensorBuf -= 6;
        if(((*pSensorBuf) != 0xa0) && ((*pSensorBuf) > 0) && ((*pSensorBuf) < 60))
        {
            bFirstPktType = 0x1;
        }
        pSensorBuf += 6;
    }
    while(dwLens)
    {
        pSensorBuf -= 6;
        dwLens -= 6;
//      UART_PRINTF("head=0x%x\r\n", *pSensorBuf);
        if((*pSensorBuf) == 0xa0)
        {
            pSensorBuf++;
            sUTC.month = *pSensorBuf;

            pSensorBuf++;
            sUTC.day = *pSensorBuf;

            pSensorBuf++;
            sUTC.hour = *pSensorBuf;

            pSensorBuf++;
            sUTC.year = (*pSensorBuf);      
            sUTC.year <<= 8;
            pSensorBuf++;
            sUTC.year |= (*pSensorBuf);     
            break;
        }
        else
        {
            sUTC.minutes = *pSensorBuf;
            if(sUTC.minutes > 59)
                break;              
        }
    }
    if((sUTC.year >= 2000) && ((sUTC.month > 0x0) && (sUTC.month <= 12))    \
        && ((sUTC.day > 0x0) && (sUTC.day <= 31)) && ((sUTC.hour > 0x0) && (sUTC.hour <= 23))   \
        && ((sUTC.minutes > 0x0) && (sUTC.minutes <= 59)))
    {
        sUTC.year -= 2000;

        sUTCTime_LastSave.month = sUTC.month;   // 0xff
        sUTCTime_LastSave.day = sUTC.day;
        sUTCTime_LastSave.hour = sUTC.hour;
        sUTCTime_LastSave.minutes = sUTC.minutes;
        sUTCTime_LastSave.seconds = sUTC.seconds;

        sUTCTime_SaveFlash.month = sUTC.month;
        sUTCTime_SaveFlash.day = sUTC.day;

        utc_set_time(&sUTC);

        UTCTimeSec_Last = utc_get_clock();
    }
    else
    {
        bFirstPktType = 0x0;
        
        sUTC.year = 18;
        sUTC.month = 1;
        sUTC.day = 1;
        sUTC.hour = 12;
        sUTC.minutes = 0;
        if(wBufInIndex)
        {
            wBufInIndex = 0x0;
            app_ClearSaveSensorDataToFlash();
        }

        utc_set_time(&sUTC);
    }
    
    pSensorBuf = szSensorBuf;
    pSensorBuf += wBufInIndex;
    if(bFirstPktType)
    {       
        pSensorBuf -= 6;
    }
    UART_PRINTF("year=%d, mon=%d, day=%d, hour=%d, min=%d, lens=0x%x\r\n", sUTC.year, sUTC.month, sUTC.day, sUTC.hour, sUTC.minutes, wBufInIndex);
}

void app_ClearSaveSensorDataToFlash(void)
{
    flash_erase(0x0, cFlashAddr_MSG, 12, NULL);
    sUTCTime_SaveFlash.month = 0x0;
    sUTCTime_SaveFlash.day = 0x0;
}

void app_SaveSensorDataToFlash(uint16_t wLens, uint8_t *pBuf)
{
    uint8_t szBuf[12], uTmp;
    uint32_t dwTmp;
    
    szBuf[0] = 'S';
    szBuf[1] = 'D';
    szBuf[2] = 'A';
    szBuf[3] = 'T';
    szBuf[4] = 0x0;
    szBuf[5] = 0x0;
    szBuf[6] = (uint8_t)(wLens >> 8);
    szBuf[7] = (uint8_t)(wLens);
    dwTmp = 0x0;
    for(uTmp = 0x0; uTmp < 8; uTmp++)
    {
        dwTmp += szBuf[uTmp];
    }
    szBuf[8] = (uint8_t)(dwTmp >> 24);
    szBuf[9] = (uint8_t)(dwTmp >> 16);
    szBuf[10] = (uint8_t)(dwTmp >> 8);
    szBuf[11] = (uint8_t)(dwTmp);
    flash_erase(0x0, cFlashAddr_MSG, 12, NULL);
    
    flash_erase(0x0, cFlashAddr_Data, wLens, NULL);
    flash_write(0x0, cFlashAddr_Data, wLens, pBuf, NULL);

    flash_write(0x0, cFlashAddr_MSG, 12, szBuf, NULL);

    UART_PRINTF("save data\r\n");
}

