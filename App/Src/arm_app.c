#include "arm_app.h"

#include "arm_buttons.h"
#include "arm_calibration.h"
#include "arm_comm.h"
#include "arm_motion.h"
#include "arm_puzzle.h"
#include "arm_safety.h"
#include "arm_servo.h"
#include "bsp_debug_uart_dma.h"
#include "bsp_magnet.h"
#include "bsp_puzzle_indicator.h"
#include "bsp_safety_io.h"
#include "bsp_uart_dma.h"

bool ArmApp_HardwareInit(void)
{
    BspMagnet_Init();
    BspPuzzleIndicator_Init();
    BspSafetyIo_Init();
    if (!ArmButtons_HardwareInit())
    {
        return false;
    }

    if (!ArmServo_Init())
    {
        return false;
    }

#if !ARM_ZERO_CALIBRATION_MODE
    if (!BspUartDma_HardwareInit())
    {
        return false;
    }
    if (!BspDebugUartDma_HardwareInit())
    {
        return false;
    }
#if ARM_DEBUG_UART_DIAGNOSTIC_ENABLED
    {
        static const uint8_t message[] = "UART2_HW_READY\r\n";
        (void)BspDebugUartDma_Transmit(message,
                                       (uint16_t)(sizeof(message) - 1U),
                                       50U);
    }
#endif
#endif

    return true;
}

bool ArmApp_RtosInit(void)
{
#if ARM_ZERO_CALIBRATION_MODE
    /*
     * 调零阶段只需要稳定的硬件 PWM。保留 CubeMX defaultTask，
     * 不启动串口、动作队列和安全任务，避免任何外部命令改变舵机位置。
     */
    return true;
#else
    /*
     * 先创建通信事件队列，再创建会向该队列投递消息的运动/安全任务。
     * 调度器尚未启动，因此任务只会在全部对象创建完成后开始运行。
     */
    if (!BspUartDma_RtosInit())
    {
        return false;
    }
    if (!BspDebugUartDma_RtosInit())
    {
        return false;
    }
    if (!ArmComm_RtosInit())
    {
        return false;
    }
    if (!ArmMotion_RtosInit())
    {
        return false;
    }
    if (!ArmCalibration_RtosInit())
    {
        return false;
    }
    if (!ArmPuzzle_RtosInit())
    {
        return false;
    }
    if (!ArmSafety_RtosInit())
    {
        return false;
    }
    if (!ArmButtons_RtosInit())
    {
        return false;
    }

    return true;
#endif
}
