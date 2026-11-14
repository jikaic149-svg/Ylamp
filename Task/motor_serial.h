#ifndef YLAMP_MOTOR_SERIAL_H
#define YLAMP_MOTOR_SERIAL_H

#include "bsp_uart.h"

#ifdef __cplusplus
extern "C" {
#endif

/* RPC wire: A5 5A / version=1 / type / payload-length(u8) / payload /
 * CRC16-CCITT-FALSE little endian over version..payload (poly1021, initFFFF).
 * type10: existing eight-byte MCU RPC request;
 * type90: eight-byte RPC response. All numeric fields explicitly little endian.
 * Reliable RPC: type12/92 = 8-byte host session HELLO/ACK;
 * type11 = session[8], id(u32), issued MCU tick(u32), RPC[8] (24 bytes);
 * type91 = session[8], id(u32), response[8] (20 bytes).
 * IDs start at 1, strictly increasing, single outstanding request; identical
 * retries resend the cached response, never re-execute. A reset loses session,
 * so old actions are rejected until a new explicit connection handshake.
 * Telemetry type01 is v2, u16 length, count + 64-byte joint records:
 * motor_telemetry.h and README.md, "串口遥测 v2". Firmware only emits v2.
 * PC accepts legacy v1 telemetry read-only for historical firmware.
 * No text may be sent on this UART while the binary link owns it.
 *
 * Init after UART init and MotorTuning_Init. Requires serial_config.h baud,
 * 8N1,
 * TX/RX, configured circular RX / normal TX DMA. UART is an explicit alternate
 * PC link; never run CAN and UART configuration writers simultaneously.
 * Owns BSP_UART callback/TX
 * for the lifetime; no dynamic allocation.
 * Run in the existing control task at 1ms, <=64 RX bytes and one RPC per pass.
 * ISR copies bytes/timestamps only into a bounded 2048-byte ring. Overflow
 * invalidates partial input and parsed requests not yet accepted for dispatch.
 * Acceptance is atomic with the overflow check; RPC execution is outside IRQ
 * masking. No automatic RX restart; CAN recovery/enable
 * policy belongs to MotorTask, never to this transport.
 * Legacy type10 has no retry guarantee. Reliable retries keep the original
 * issued tick; the existing 500ms lifetime cannot be refreshed by retrying.
 * Lost replies after the retry budget still leave the remote result unknown.
 * Telemetry sequence counts due slots, including skipped ones. tx_drops counts
 * RPC-preempted/busy telemetry slots AND failed submissions, not wire errors. */
HAL_StatusTypeDef MotorSerial_Init(UART_HandleTypeDef *uart);
HAL_StatusTypeDef MotorSerial_Run(uint32_t now_ms);

#ifdef __cplusplus
}
#endif
#endif
