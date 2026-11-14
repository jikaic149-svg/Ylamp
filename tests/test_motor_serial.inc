/* Real UART/BSP/RPC integration with the shared HAL fixture; no hardware. */
static uint16_t serial_crc(const uint8_t *bytes, unsigned size)
{
    uint16_t crc = 0xFFFFU;
    for (unsigned i = 0U; i < size; ++i) {
        crc ^= (uint16_t)((uint16_t)bytes[i] << 8);
        for (unsigned j = 0U; j < 8U; ++j) {
            crc = (uint16_t)((crc << 1) ^ ((crc & 0x8000U) ? 0x1021U : 0U));
        }
    }
    return crc;
}

static uint32_t serial_u32(const uint8_t *bytes)
{
    return (uint32_t)bytes[0] | ((uint32_t)bytes[1] << 8) |
        ((uint32_t)bytes[2] << 16) | ((uint32_t)bytes[3] << 24);
}

static void serial_request(uint8_t frame[15], uint8_t op, uint8_t field)
{
    const uint8_t hello[15] = {
        0xA5U, 0x5AU, 1U, 0x10U, 8U, 1U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0xFCU, 0xF6U};
    memcpy(frame, hello, sizeof(hello));
    frame[6] = op; frame[8] = field;
    if (op == 7U) { frame[9] = 1U; } /* Current revision for an explicit START. */
    const uint16_t crc = serial_crc(&frame[2], 11U);
    frame[13] = (uint8_t)crc; frame[14] = (uint8_t)(crc >> 8);
    if (op == 0U) { assert(memcmp(frame, hello, sizeof(hello)) == 0); }
}

static void serial_receive(const uint8_t *data, uint16_t size, uint32_t now)
{
    assert(size <= BSP_UART_RX_CAPACITY && uart_rx_data != NULL);
    test_tick = now;
    uint16_t at = (uint16_t)((BSP_UART_DMA_RX_CAPACITY -
        huart1.hdmarx->remaining) % BSP_UART_DMA_RX_CAPACITY);
    for (uint16_t i = 0U; i < size; ++i) {
        uart_rx_data[at] = data[i];
        at = (uint16_t)((at + 1U) % BSP_UART_DMA_RX_CAPACITY);
    }
    huart1.hdmarx->remaining = BSP_UART_DMA_RX_CAPACITY - at;
    test_ipsr = 16U;
    HAL_UARTEx_RxEventCallback(&huart1, at == 0U ? BSP_UART_DMA_RX_CAPACITY : at);
    test_ipsr = 0U;
}

static void serial_complete(void)
{
    huart1.gState = HAL_UART_STATE_READY;
    test_ipsr = 16U;
    HAL_UART_TxCpltCallback(&huart1);
    test_ipsr = 0U;
}

static HAL_StatusTypeDef serial_run(uint32_t now)
{
    test_tick = now;
    return MotorSerial_Run(now);
}

static const uint8_t *serial_output(uint8_t kind, uint16_t length)
{
    const unsigned header = kind == 1U ? 6U : 5U;
    const unsigned total = length + header + 2U;
    assert(uart_tx_len == total);
    assert(uart_tx_data[0] == 0xA5U && uart_tx_data[1] == 0x5AU);
    assert(uart_tx_data[2] == (kind == 1U ? 2U : 1U) && uart_tx_data[3] == kind);
    assert(uart_tx_data[4] == (uint8_t)length);
    if (kind == 1U) { assert(uart_tx_data[5] == (uint8_t)(length >> 8)); }
    const uint16_t expected = (uint16_t)(uart_tx_data[total - 2U] |
        ((uint16_t)uart_tx_data[total - 1U] << 8));
    assert(serial_crc(&uart_tx_data[2], total - 4U) == expected);
    return &uart_tx_data[header];
}

static void serial_overflow_irq(void)
{
    const uint8_t noise[128] = {0};
    for (unsigned i = 0U; i < 17U; ++i) {
        serial_receive(noise, sizeof(noise), test_tick);
    }
}

static void serial_wire(uint8_t *frame, uint8_t kind,
                        const uint8_t *payload, uint8_t length)
{
    frame[0] = 0xA5U; frame[1] = 0x5AU; frame[2] = 1U;
    frame[3] = kind; frame[4] = length;
    memcpy(&frame[5], payload, length);
    const uint16_t crc = serial_crc(&frame[2], (unsigned)length + 3U);
    frame[5U + length] = (uint8_t)crc;
    frame[6U + length] = (uint8_t)(crc >> 8);
}

static void serial_reliable(uint8_t frame[31], const uint8_t session[8],
    uint32_t id, uint32_t issued, uint8_t command)
{
    uint8_t payload[24], legacy[15];
    memcpy(payload, session, 8U);
    for (unsigned i = 0U; i < 4U; ++i) {
        payload[8U + i] = (uint8_t)(id >> (8U * i));
        payload[12U + i] = (uint8_t)(issued >> (8U * i));
    }
    serial_request(legacy, 7U, command);
    memcpy(&payload[16], &legacy[5], 8U);
    serial_wire(frame, 0x11U, payload, sizeof(payload));
}

static void test_serial_reliable(void)
{
    setup_task(true, 995U);
    task_run(1000U);
    assert(MotorTuning_Init(NULL) == BSP_CAN_OK);
    assert(JointCalibration_Init() == BSP_CAN_OK);
    huart1.Init.BaudRate = YLAMP_SERIAL_BAUD_RATE;
    huart1.hdmarx = &uart_receive_dma;
    huart1.hdmatx = &uart_transmit_dma;
    huart1.hdmarx->Init.Mode = DMA_NORMAL;
    assert(MotorSerial_Init(&huart1) == HAL_ERROR);
    huart1.hdmarx->Init.Mode = DMA_CIRCULAR;
    assert(MotorSerial_Init(&huart1) == HAL_OK);

    const uint8_t session[8] = {1U, 2U, 3U, 4U, 5U, 6U, 7U, 8U};
    uint8_t hello[15], start[31], disable[31], expired[31];
    serial_reliable(start, session, 1U, 1000U, MOTOR_COMMAND_START);
    serial_receive(start, sizeof(start), 1000U);
    assert(serial_run(1000U) == HAL_OK);
    assert(serial_output(0x91U, 20U)[13] == BSP_CAN_NOT_READY);
    assert(task_status().state == MOTOR_TASK_IDLE); /* No session after reboot. */
    serial_complete();

    serial_wire(hello, 0x12U, session, 8U);
    serial_receive(hello, sizeof(hello), 1000U);
    assert(serial_run(1000U) == HAL_OK);
    assert(memcmp(serial_output(0x92U, 8U), session, 8U) == 0);
    serial_complete();
    const unsigned sent = uart_tx_calls;
    HAL_UARTEx_RxEventCallback(&huart1, BSP_UART_DMA_RX_CAPACITY / 2U);
    assert(serial_run(1000U) == HAL_OK && uart_tx_calls == sent);
    /* A delayed HT/duplicate IDLE callback cannot redeliver old bytes. */

    serial_receive(start, sizeof(start), 1001U);
    tick_hook = serial_overflow_irq;
    assert(serial_run(1001U) == HAL_OK && uart_tx_calls == sent);
    assert(task_status().state == MOTOR_TASK_IDLE); /* Lost before acceptance. */

    serial_receive(start, sizeof(start), 1002U); /* Identical retry. */
    uart_tx_result = HAL_ERROR; /* Action accepted, but its reply was lost. */
    assert(serial_run(1002U) == HAL_ERROR);
    assert(task_status().state == MOTOR_TASK_ARMING);
    uart_tx_result = HAL_OK;
    serial_receive(hello, sizeof(hello), 1003U);
    assert(serial_run(1003U) == HAL_OK); /* Same-session HELLO preserves cache. */
    serial_complete();
    serial_receive(start, sizeof(start), 1601U);
    assert(serial_run(1601U) == HAL_OK);
    assert(serial_output(0x91U, 20U)[13] == BSP_CAN_OK);
    assert(task_status().state == MOTOR_TASK_ARMING); /* No second START. */
    serial_complete();

    serial_reliable(disable, session, 1U, 1602U, MOTOR_COMMAND_DISABLE);
    serial_receive(disable, sizeof(disable), 1602U);
    assert(serial_run(1602U) == HAL_OK);
    assert(serial_output(0x91U, 20U)[13] == BSP_CAN_INVALID_ARG);
    assert(motor_disable_calls[1] == 0U); /* Same ID, changed intent rejected. */
    serial_complete();
    serial_reliable(disable, session, 2U, 1603U, MOTOR_COMMAND_DISABLE);
    serial_receive(disable, sizeof(disable), 1603U);
    assert(serial_run(1603U) == HAL_OK);
    assert(serial_output(0x91U, 20U)[13] == BSP_CAN_OK);
    assert(motor_disable_calls[1] == 1U && motor_disable_calls[2] == 1U);
    serial_complete();
    serial_receive(disable, sizeof(disable), 1604U);
    assert(serial_run(1604U) == HAL_OK);
    assert(serial_output(0x91U, 20U)[13] == BSP_CAN_OK);
    assert(motor_disable_calls[1] == 1U && motor_disable_calls[2] == 1U);
    serial_complete();
    serial_receive(start, sizeof(start), 1605U);
    assert(serial_run(1605U) == HAL_OK);
    assert(serial_output(0x91U, 20U)[13] == BSP_CAN_NOT_READY); /* Old ID. */
    serial_complete();
    serial_reliable(expired, session, 3U, 1000U, MOTOR_COMMAND_START);
    serial_receive(expired, sizeof(expired), 1606U);
    assert(serial_run(1606U) == HAL_OK);
    assert(serial_output(0x91U, 20U)[13] == BSP_CAN_NOT_READY);
    serial_complete();
    /* A damaged long frame can contain two shorter valid frames. Keep the
     * trailing bytes when accepting the first, even with no new RX afterward. */
    uint8_t mixed[35] = {0xA5U, 0x5AU, 1U, 0x11U, 24U};
    memcpy(&mixed[5], hello, sizeof(hello));
    memcpy(&mixed[20], hello, sizeof(hello));
    assert(serial_crc(&mixed[2], 27U) !=
        (uint16_t)(mixed[29] | ((uint16_t)mixed[30] << 8)));
    serial_receive(mixed, sizeof(mixed), 1607U);
    assert(serial_run(1607U) == HAL_OK);
    assert(memcmp(serial_output(0x92U, 8U), session, 8U) == 0);
    serial_complete();
    assert(serial_run(1608U) == HAL_OK);
    assert(memcmp(serial_output(0x92U, 8U), session, 8U) == 0);
    serial_complete();
    assert(task_status().state == MOTOR_TASK_FAULT && motor_enable_calls[1] == 1U);
    assert(uart_rx_calls == 1U && test_primask == 0U); /* Circular RX never rearms. */
}

static void test_serial_link(void)
{
    uint32_t now = 1000U;
    setup_task(true, now - 5U);
    task_run(now);
    assert(MotorTuning_Init(NULL) == BSP_CAN_OK);
    assert(JointCalibration_Init() == BSP_CAN_OK);
    assert(MotorSerial_Init(NULL) == HAL_ERROR);
    huart1.Init.BaudRate = 9600U;
    assert(MotorSerial_Init(&huart1) == HAL_ERROR);
    huart1.Init.BaudRate = 115200U; /* Reject a stale host/board baud setting. */
    assert(MotorSerial_Init(&huart1) == HAL_ERROR);
    huart1.Init.BaudRate = YLAMP_SERIAL_BAUD_RATE;
    assert(MotorSerial_Init(&huart1) == HAL_ERROR); /* No implicit IT fallback. */
    huart1.hdmarx = &uart_receive_dma;
    huart1.hdmatx = &uart_transmit_dma;
    assert(MotorSerial_Init(&huart1) == HAL_OK);
    assert(MotorSerial_Init(&huart1) == HAL_BUSY);
    test_ipsr = 16U;
    assert(serial_run(now) == HAL_ERROR);
    test_ipsr = 0U;
    assert(serial_run(now) == HAL_OK && uart_tx_calls == 0U);

    assert(serial_run(1020U) == HAL_OK);
    const uint8_t *p = serial_output(1U, 192U);
    assert(serial_u32(p) == 1020U && serial_u32(&p[4]) == 1U);
    assert(p[8] == MOTOR_TASK_IDLE && p[11] == 2U && p[14] == 64U);
    assert(p[64] == 1U && (p[65] & 1U) && p[128] == 2U && (p[129] & 1U));
    assert(serial_u32(&p[72]) == now - 5U && serial_u32(&p[76]) == 1U);
    assert(serial_run(1040U) == HAL_BUSY && uart_tx_calls == 1U);
    assert(serial_u32(&p[4]) == 1U); /* Busy must not overwrite in-flight data. */
    serial_complete();
    assert(serial_run(1060U) == HAL_OK);
    p = serial_output(1U, 192U);
    assert(serial_u32(&p[4]) == 3U && serial_u32(&p[48]) == 1U);
    assert(serial_u32(&p[72]) == now - 5U && serial_u32(&p[76]) == 1U);
    serial_complete(); /* UART traffic did not refresh CAN timestamps/counts. */

    uint8_t hello[15], start[15], noise[128];
    serial_request(hello, 0U, 0U);
    serial_request(start, 7U, MOTOR_COMMAND_START);
    unsigned sent = can_tx_calls, uart_sent = uart_tx_calls;
    serial_receive(hello, 6U, 1061U);
    assert(serial_run(1061U) == HAL_OK && uart_tx_calls == uart_sent);
    serial_receive(&hello[6], 9U, 1062U);
    assert(serial_run(1062U) == HAL_OK);
    p = serial_output(0x90U, 8U);
    assert(p[0] == 1U && p[1] == BSP_CAN_OK);
    assert(serial_u32(&p[4]) == MOTOR_TUNING_VERSION && can_tx_calls == sent);
    serial_complete();

    start[14] ^= 1U; /* Bad CRC must not execute START, then resynchronize. */
    memcpy(noise, start, 15U);
    memcpy(&noise[15], hello, 15U); noise[19] = 9U; /* Invalid declared length. */
    memcpy(&noise[30], hello, 15U);
    serial_receive(noise, 45U, 1063U);
    assert(serial_run(1063U) == HAL_OK);
    p = serial_output(0x90U, 8U);
    assert(p[1] == BSP_CAN_OK && serial_u32(&p[4]) == MOTOR_TUNING_VERSION);
    assert(motor_enable_calls[1] == 0U && motor_enable_calls[2] == 0U);
    serial_complete();

    serial_receive(hello, sizeof(hello), 1070U);
    /* UART IRQ completed after the control loop captured its timestamp. */
    assert(MotorSerial_Run(1069U) == HAL_OK);
    assert(serial_output(0x90U, 8U)[1] == BSP_CAN_OK);
    serial_complete();

    assert(serial_run(1080U) == HAL_OK); /* Leave telemetry in flight. */
    serial_request(start, 7U, MOTOR_COMMAND_START);
    serial_receive(start, sizeof(start), 1081U);
    assert(serial_run(1081U) == HAL_OK);
    assert(serial_run(1700U) == HAL_BUSY);
    serial_complete();
    assert(serial_run(1701U) == HAL_OK);
    p = serial_output(0x90U, 8U);
    assert(p[1] == BSP_CAN_NOT_READY && motor_enable_calls[1] == 0U);
    serial_complete(); /* Queued START expired, never retried. */

    memset(noise, 0, sizeof(noise));
    for (unsigned i = 0U; i < 17U; ++i) { serial_receive(noise, sizeof(noise), 1702U); }
    serial_receive(start, sizeof(start), 1702U);
    uart_sent = uart_tx_calls;
    assert(serial_run(1702U) == HAL_OK && uart_tx_calls == uart_sent);
    serial_receive(hello, sizeof(hello), 1703U);
    assert(serial_run(1703U) == HAL_OK);
    assert(serial_output(0x90U, 8U)[1] == BSP_CAN_OK);
    serial_complete();
    assert(serial_run(1721U) == HAL_OK);
    p = serial_output(1U, 192U);
    assert(serial_u32(&p[40]) > 0U && serial_u32(&p[44]) == 1U);
    serial_complete();

    now = 1790U;
    feedback_pair(now, 0U);
    task_run(now);
    serial_request(start, 7U, MOTOR_COMMAND_START);
    serial_receive(start, sizeof(start), now);
    /* Overflow after parsing, before dispatch, must invalidate pending START. */
    tick_hook = serial_overflow_irq;
    assert(serial_run(now) == HAL_OK);
    assert(task_status().state == MOTOR_TASK_IDLE && motor_enable_calls[1] == 0U);
    serial_complete(); /* The telemetry period was due, but no RPC was accepted. */
    serial_receive(hello, sizeof(hello), now + 1U);
    assert(serial_run(now + 1U) == HAL_OK);
    assert(serial_output(0x90U, 8U)[1] == BSP_CAN_OK);
    serial_complete();

    now = 1800U;
    feedback_pair(now, 0U);
    task_run(now);
    serial_request(start, 7U, MOTOR_COMMAND_START);
    serial_receive(start, sizeof(start), now);
    assert(serial_run(now) == HAL_OK);
    assert(serial_output(0x90U, 8U)[1] == BSP_CAN_OK);
    assert(task_status().state == MOTOR_TASK_ARMING);
    serial_complete();
    task_steps(&now, 2U, 1U);
    assert(task_status().state == MOTOR_TASK_RUNNING);
    can_protocol[1].BusOff = 1U;
    hfdcan2.Instance->CCCR = FDCAN_CCCR_INIT;
    can_protocol[1].LastErrorCode = 5U;
    can_counters[1].TxErrorCnt = 248U;
    hfdcan2.Instance->TXBRP = 0x3CU;
    now += 5U;
    feedback_pair(now, 1U);
    task_run(now);
    assert(task_status().state == MOTOR_TASK_RECOVERING);
    assert(task_status().reason == MOTOR_REASON_TRANSPORT);
    assert(serial_run(now + 20U) == HAL_OK);
    p = serial_output(1U, 192U);
    assert(p[8] == MOTOR_TASK_RECOVERING && p[9] == MOTOR_REASON_TRANSPORT);
    assert(!(p[65] & 2U) && !(p[129] & 2U) &&
           (p[20] & 2U) && p[32] == 5U && p[33] == 248U);
    assert(serial_u32(&p[36]) == 0x3CU);
    serial_complete();
    const unsigned psr_reads = can_protocol_reads, ecr_reads = can_counter_reads;
    sent = can_tx_calls;
    serial_request(hello, 6U, 0U);
    serial_receive(hello, sizeof(hello), now + 21U);
    assert(serial_run(now + 21U) == HAL_OK);
    p = serial_output(0x90U, 8U);
    assert(p[1] == BSP_CAN_OK && p[4] == MOTOR_TASK_RECOVERING);
    assert(can_tx_calls == sent); /* CAN is down, UART RPC still answers. */
    serial_complete();
    assert(can_protocol_reads == psr_reads && can_counter_reads == ecr_reads);

    const unsigned rx_calls = uart_rx_calls;
    huart1.ErrorCode = HAL_UART_ERROR_ORE;
    huart1.RxState = HAL_UART_STATE_READY;
    HAL_UART_ErrorCallback(&huart1);
    assert(serial_run(now + 40U) == HAL_OK);
    p = serial_output(1U, 192U);
    assert(serial_u32(&p[52]) & HAL_UART_ERROR_ORE);
    assert(!(p[20] & 16U) && (p[20] & 32U));
    assert(uart_rx_calls == rx_calls && uart_abort_calls == 0U); /* No auto restart. */
    assert(can_start_calls == 1U && can_stop_calls == 1U &&
           motor_enable_calls[1] == 1U && test_primask == 0U);
}
