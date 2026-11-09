#include "arm_comm.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include "arm_debug_cli.h"
#include "arm_puzzle.h"
#include "arm_puzzle_protocol.h"
#include "bsp_debug_uart_dma.h"
#include "bsp_uart_dma.h"

static QueueHandle_t s_event_queue;
static TaskHandle_t s_comm_task;
static volatile uint32_t s_last_valid_frame_tick;
static ArmPuzzleProtocolParser s_puzzle_parser;
static uint8_t s_active_vision_task;
static uint32_t s_injected_frame_counter;
static uint32_t s_injected_signature_salt;
static bool s_vision_parse_only;
static uint32_t s_vision_debug_start_tick;

static void ArmComm_HandlePuzzlePlan(const ArmPuzzlePlan *plan,
                                     void *context)
{
    ArmPuzzlePlan tagged_plan;
    ArmPuzzleSubmitResult submit_result;
    uint8_t ack = ARM_PUZZLE_ACK;

    (void)context;
    if (plan == NULL)
    {
        return;
    }

    /* 未经TASK1/TASK2/VREAD触发的残留或串扰帧禁止启动机械臂。 */
    if ((s_active_vision_task != 1U) &&
        (s_active_vision_task != 2U) &&
        (s_active_vision_task != 3U))
    {
        return;
    }

    tagged_plan = *plan;
    tagged_plan.task_id = s_active_vision_task;

    if (s_vision_parse_only)
    {
        /*
         * VREAD只验证协议并输出坐标。合法帧仍回复AC让K230停止重发，
         * 但绝不调用ArmPuzzle_SubmitPlan，因此舵机和电磁铁保持不动。
         */
        s_last_valid_frame_tick = HAL_GetTick();
        (void)BspUartDma_Transmit(&ack, 1U, 50U);
        ArmDebugCli_ReportVisionPlan(&tagged_plan);
        s_vision_parse_only = false;
        s_active_vision_task = 0U;
        return;
    }
    /*
     * 52字节结果帧本身不含题号。把触发来源混入签名，保证任务1和
     * 任务2即使偶然返回相同坐标，也能各执行一次；同一任务的重发
     * 仍会被识别为重复包，只重新回复AC而不重复执行。
     */
    tagged_plan.signature ^=
        (uint32_t)(0x9E3779B9UL * (uint32_t)tagged_plan.task_id);
    /*
     * 调试口主动注入的同一帧允许反复执行；正式USART1接收时salt为0，
     * 仍保留视觉重发包只ACK、不重复动作的原有安全语义。
     */
    tagged_plan.signature ^= s_injected_signature_salt;

    s_last_valid_frame_tick = HAL_GetTick();
    submit_result = ArmPuzzle_SubmitPlan(&tagged_plan);

    /*
     * 新计划已可靠存入队列，或只是ACK丢失造成的同包重发，均回复单
     * 字节0xAC。忙于另一套不同计划时不ACK，让K230按协议稍后重发。
     */
    if ((submit_result == ARM_PUZZLE_SUBMIT_ACCEPTED) ||
        (submit_result == ARM_PUZZLE_SUBMIT_DUPLICATE))
    {
        (void)BspUartDma_Transmit(&ack, 1U, 50U);
    }
}

static void ArmComm_HandleEvent(const ArmEvent *event)
{
    if (event == NULL)
    {
        return;
    }

    /*
     * USART1已经冻结为第一问52字节视觉协议，其唯一回复是0xAC，
     * 不再混发旧版CRC控制帧。USART2继续输出可读的动作/安全结果。
     */
    if ((event->response_port == ARM_COMM_PORT_DEBUG) ||
        (event->response_port == ARM_COMM_PORT_ALL))
    {
        ArmDebugCli_HandleEvent(event);
    }
}

static void ArmComm_Task(void *argument)
{
    BspUartRxChunk rx_chunk;
    ArmEvent event;
    bool vision_started = false;
    bool debug_started = false;

    (void)argument;
    ArmPuzzleProtocol_Init(&s_puzzle_parser);
    s_last_valid_frame_tick = HAL_GetTick();
    s_active_vision_task = 0U;
    s_injected_frame_counter = 0U;
    s_injected_signature_salt = 0U;
    s_vision_parse_only = false;
    s_vision_debug_start_tick = 0U;

    /*
     * 两路串口独立启动。已经启动成功的串口不会重复启动；
     * 启动失败的一路留在任务中重试，不阻塞其他安全任务。
     */
    while (!vision_started || !debug_started)
    {
        if (!vision_started)
        {
            vision_started = BspUartDma_Start();
        }
        if (!debug_started)
        {
            debug_started = BspDebugUartDma_Start();
        }

        if (!vision_started || !debug_started)
        {
            vTaskDelay(pdMS_TO_TICKS(100U));
        }
    }

    ArmDebugCli_Init();

    for (;;)
    {
        if (BspUartDma_Receive(&rx_chunk, pdMS_TO_TICKS(5U)))
        {
            uint32_t invalid_before = s_puzzle_parser.invalid_frames;
            bool parse_only_before = s_vision_parse_only;

            ArmPuzzleProtocol_Feed(&s_puzzle_parser,
                                   rx_chunk.data,
                                   rx_chunk.length,
                                   ArmComm_HandlePuzzlePlan,
                                   NULL);

            if (parse_only_before && s_vision_parse_only &&
                (s_puzzle_parser.invalid_frames > invalid_before))
            {
                uint8_t failed_task = s_active_vision_task;

                s_vision_parse_only = false;
                s_active_vision_task = 0U;
                ArmDebugCli_ReportVisionReadError(
                    failed_task,
                    "FRAME_HEADER_CHECKSUM_OR_DATA");
            }
        }

        if (BspDebugUartDma_Receive(&rx_chunk, pdMS_TO_TICKS(5U)))
        {
            if (ArmDebugCli_HandleChunk(rx_chunk.data, rx_chunk.length))
            {
                s_last_valid_frame_tick = HAL_GetTick();
            }
        }

        while (xQueueReceive(s_event_queue, &event, 0U) == pdPASS)
        {
            ArmComm_HandleEvent(&event);
        }

        if (s_vision_parse_only &&
            ((HAL_GetTick() - s_vision_debug_start_tick) >
             ARM_VISION_DEBUG_TIMEOUT_MS))
        {
            uint8_t timeout_task = s_active_vision_task;

            s_vision_parse_only = false;
            s_active_vision_task = 0U;
            ArmPuzzleProtocol_Init(&s_puzzle_parser);
            ArmDebugCli_ReportVisionReadError(timeout_task, "TIMEOUT");
        }
    }
}

bool ArmComm_RtosInit(void)
{
    BaseType_t task_result;

    s_event_queue = xQueueCreate(ARM_EVENT_QUEUE_DEPTH, sizeof(ArmEvent));
    if (s_event_queue == NULL)
    {
        return false;
    }

    task_result = xTaskCreate(ArmComm_Task,
                              "ArmComm",
                              ARM_COMM_TASK_STACK_WORDS,
                              NULL,
                              ARM_COMM_TASK_PRIORITY,
                              &s_comm_task);
    return (task_result == pdPASS);
}

bool ArmComm_PublishEvent(const ArmEvent *event)
{
    if ((event == NULL) || (s_event_queue == NULL))
    {
        return false;
    }

    return (xQueueSend(s_event_queue, event, 0U) == pdPASS);
}

static bool ArmComm_StartVisionTask(uint8_t task_id,
                                    uint8_t trigger,
                                    bool parse_only)
{
    /*
     * TASK1/2由ArmComm任务内的USART2 CLI调用，不会与视觉解析并发。
     * 发送前清空可能残留的半帧，视觉返回AA 55开头的新52字节帧后，
     * 由同一解析器校验、保存、回复AC并提交机械臂抓放任务。
     */
    ArmPuzzleProtocol_Init(&s_puzzle_parser);
    s_active_vision_task = task_id;
    s_vision_parse_only = parse_only;
    s_vision_debug_start_tick = HAL_GetTick();

    if (!BspUartDma_Transmit(&trigger, 1U, 50U))
    {
        s_active_vision_task = 0U;
        s_vision_parse_only = false;
        return false;
    }

    return true;
}

bool ArmComm_StartVisionTask1(void)
{
    return ArmComm_StartVisionTask(1U,
                                   ARM_PUZZLE_TASK1_TRIGGER,
                                   false);
}

bool ArmComm_StartVisionTask2(void)
{
    return ArmComm_StartVisionTask(2U,
                                   ARM_PUZZLE_TASK2_TRIGGER,
                                   false);
}

bool ArmComm_StartVisionTask3(void)
{
    return ArmComm_StartVisionTask(3U,
                                   ARM_PUZZLE_TASK3_TRIGGER,
                                   false);
}

bool ArmComm_SendVisionCommand(uint8_t command)
{
    /*
     * 用于视觉端内部校准等不返回机械臂坐标帧的单字节命令。
     * 不设置active_task，因此即使视觉端误回52字节帧，也不会启动机械臂。
     */
    return BspUartDma_Transmit(&command, 1U, 50U);
}

bool ArmComm_StartVisionDebug(uint8_t task_id)
{
    if (task_id == 1U)
    {
        return ArmComm_StartVisionTask(1U,
                                       ARM_PUZZLE_TASK1_TRIGGER,
                                       true);
    }
    if (task_id == 2U)
    {
        return ArmComm_StartVisionTask(2U,
                                       ARM_PUZZLE_TASK2_TRIGGER,
                                       true);
    }
    if (task_id == 3U)
    {
        return ArmComm_StartVisionTask(3U,
                                       ARM_PUZZLE_TASK3_TRIGGER,
                                       true);
    }

    return false;
}

bool ArmComm_InjectVisionFrame(uint8_t task_id,
                               const uint8_t *frame,
                               uint16_t length)
{
    if (((task_id != 1U) && (task_id != 2U) && (task_id != 3U)) ||
        (frame == NULL) ||
        (length != ARM_PUZZLE_PACKET_SIZE))
    {
        return false;
    }

    /*
     * USART2 CLI与USART1视觉接收都在ArmComm任务内运行，可安全复用
     * 正式解析器。注入帧仍执行帧头、版本、COUNT、坐标、校验和及帧尾
     * 的全部检查，只有合法计划才会进入机械臂任务队列。
     */
    ArmPuzzleProtocol_Init(&s_puzzle_parser);
    s_active_vision_task = task_id;
    s_vision_parse_only = false;
    ++s_injected_frame_counter;
    if (s_injected_frame_counter == 0U)
    {
        ++s_injected_frame_counter;
    }
    s_injected_signature_salt =
        0x85EBCA6BUL * s_injected_frame_counter;
    ArmPuzzleProtocol_Feed(&s_puzzle_parser,
                           frame,
                           length,
                           ArmComm_HandlePuzzlePlan,
                           NULL);
    s_injected_signature_salt = 0U;

    return
        ((s_puzzle_parser.valid_frames == 1U) &&
         (s_puzzle_parser.invalid_frames == 0U));
}

bool ArmComm_SimulateVisionFrame(uint8_t task_id)
{
    /*
     * 内置两片合法测试帧：
     *   0: pick=(80,-60),  place=(80,60),  rotate=0
     *   1: pick=(130,-60), place=(130,60), rotate=0
     * COUNT=2，未使用的两个记录槽按最终协议填0，校验和=0x7B。
     *
     * 该函数由USART2 CLI在ArmComm任务上下文中调用，因此可安全复用
     * 正式USART1解析器及处理回调，完整验证“解析→保存→AC→执行”。
     */
    static const uint8_t test_frame[ARM_PUZZLE_PACKET_SIZE] =
    {
        0xAAU, 0x55U, 0x01U, 0x01U, 0x02U,
        0x00U, 0x03U, 0x20U, 0xFDU, 0xA8U,
        0x03U, 0x20U, 0x02U, 0x58U, 0x00U, 0x00U,
        0x01U, 0x05U, 0x14U, 0xFDU, 0xA8U,
        0x05U, 0x14U, 0x02U, 0x58U, 0x00U, 0x00U,
        0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U,
        0x00U, 0x00U, 0x00U, 0x00U, 0x00U,
        0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U,
        0x00U, 0x00U, 0x00U, 0x00U, 0x00U,
        0x7BU, 0x55U, 0xCCU
    };

    return ArmComm_InjectVisionFrame(task_id,
                                     test_frame,
                                     (uint16_t)sizeof(test_frame));
}

void ArmComm_GetVisionProtocolStats(uint32_t *valid_frames,
                                    uint32_t *invalid_frames,
                                    uint8_t *active_task)
{
    if (valid_frames != NULL)
    {
        *valid_frames = s_puzzle_parser.valid_frames;
    }
    if (invalid_frames != NULL)
    {
        *invalid_frames = s_puzzle_parser.invalid_frames;
    }
    if (active_task != NULL)
    {
        *active_task = s_active_vision_task;
    }
}

uint32_t ArmComm_GetLastValidFrameTick(void)
{
    return s_last_valid_frame_tick;
}
