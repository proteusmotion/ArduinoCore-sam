/*
  Copyright (c) 2011 Arduino.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "UARTClass.h"

#include "Arduino.h"
#include "config.h"

/*
From FTDI:

FTxxx RTS# pin is an output. It should be connected to the CTS# input pin of the device at the other end of the UART link.
If RTS# is logic 0 it is indicating the FTxxx device can accept more data on the RXD pin.
If RTS# is logic 1 it is indicating the FTxxx device cannot accept more data.
RTS# changes state when the chip buffer reaches its last 32 bytes of space to allow time for the external device to stop sending data to the FTxxx device.
FTxxx CTS# pin is an input. It should be connected to the RTS# output pin of the device at the other end of the UART link.
If CTS# is logic 0 it is indicating the external device can accept more data, and the FTxxx will transmit on the TXD pin.
If CTS# is logic 1 it is indicating the external device cannot accept more data. the FTxxx will stop transmitting within 0~3 characters, depending on what is in the buffer.
This potential 3 character overrun does occasionally present problems. Customers shoud be made aware the FTxxx is a USB device and not a "normal" RS232 device as seen on a PC. As such the device operates on a packet basis as opposed to a byte basis.
Word to the wise. Not only do RS232 level shifting devices (e.g. MAX232) level shift, but they also invert the signal.
*/

// * IMPORTANT *: 
// ctsPin must be connected to the RTS output from the FTDI device
static int ctsPin = FLOW_CONTROL_CTS_PIN;
// likewise, rtsPin must be connected to the CTS input of the FTDI
static int rtsPin = FLOW_CONTROL_RTS_PIN;

static volatile bool recvPaused = false;
static volatile bool sendPaused = false;
static volatile int ctsStatus = 0;

void _cts_irq(void) {
  ctsStatus = digitalRead(ctsPin);
  if (ctsStatus == 1) {
    sendPaused = true;
  } else {
    sendPaused = false;
  }
}

// Constructors ////////////////////////////////////////////////////////////////

UARTClass::UARTClass( Uart *pUart, IRQn_Type dwIrq, uint32_t dwId, RingBuffer *pRx_buffer, RingBuffer *pTx_buffer )
{
  _rx_buffer = pRx_buffer;
  _tx_buffer = pTx_buffer;

  _pUart=pUart;
  _dwIrq=dwIrq;
  _dwId=dwId;
}

// Public Methods //////////////////////////////////////////////////////////////

int UARTClass::manageRTS(void)
{
  unsigned int avail = ((unsigned int)(SERIAL_BUFFER_SIZE + _rx_buffer->_iHead - _rx_buffer->_iTail)) % SERIAL_BUFFER_SIZE;

  if (!recvPaused && (avail >= SERIAL_BUFFER_SIZE)) {
    recvPaused = true;
    digitalWrite(rtsPin, 1);
  } else {      
    if (recvPaused && (avail <= 0)) {
      recvPaused = false;
      digitalWrite(rtsPin, 0);
    }
  }
  
  return avail;
}

void UARTClass::initFlowControl(void) {
  // monitor CTS
  pinMode(ctsPin, INPUT_PULLUP);
  _cts_irq();  // initialize ctsStatus
  attachInterrupt(digitalPinToInterrupt(ctsPin), _cts_irq, CHANGE);

  // assign RTS
  pinMode(rtsPin, OUTPUT);
  digitalWrite(rtsPin, 0);  // ready to receive
}


void UARTClass::begin(const uint32_t dwBaudRate)
{
  begin(dwBaudRate, Mode_8N1);
}

void UARTClass::begin(const uint32_t dwBaudRate, const UARTModes config)
{
  uint32_t modeReg = static_cast<uint32_t>(config) & 0x00000E00;
  init(dwBaudRate, modeReg | UART_MR_CHMODE_NORMAL);
}

void UARTClass::init(const uint32_t dwBaudRate, const uint32_t modeReg)
{
  // Configure PMC
  pmc_enable_periph_clk( _dwId );

  // Disable PDC channel
  _pUart->UART_PTCR = UART_PTCR_RXTDIS | UART_PTCR_TXTDIS;

  // Reset and disable receiver and transmitter
  _pUart->UART_CR = UART_CR_RSTRX | UART_CR_RSTTX | UART_CR_RXDIS | UART_CR_TXDIS;

  // Configure mode
  _pUart->UART_MR = modeReg;

  // Configure baudrate (asynchronous, no oversampling)
  _pUart->UART_BRGR = (SystemCoreClock / dwBaudRate) >> 4;	

  // Configure interrupts
  _pUart->UART_IDR = 0xFFFFFFFF;
  _pUart->UART_IER = UART_IER_RXRDY | UART_IER_OVRE | UART_IER_FRAME;

  // Enable UART interrupt in NVIC
  NVIC_EnableIRQ(_dwIrq);

  // Make sure both ring buffers are initialized back to empty.
  _rx_buffer->_iHead = _rx_buffer->_iTail = 0;
  _tx_buffer->_iHead = _tx_buffer->_iTail = 0;

  // Enable receiver and transmitter
  _pUart->UART_CR = UART_CR_RXEN | UART_CR_TXEN;

  if (FLOW_CONTROL_ENABLED) {
    initFlowControl();
  }
}

void UARTClass::end( void )
{
  // Clear any received data
  _rx_buffer->_iHead = _rx_buffer->_iTail;

  // Wait for any outstanding data to be sent
  flush();

  // Disable UART interrupt in NVIC
  NVIC_DisableIRQ( _dwIrq );

  pmc_disable_periph_clk( _dwId );
}

void UARTClass::setInterruptPriority(uint32_t priority)
{
  NVIC_SetPriority(_dwIrq, priority & 0x0F);
}

uint32_t UARTClass::getInterruptPriority()
{
  return NVIC_GetPriority(_dwIrq);
}

int UARTClass::available( void )
{
  if (FLOW_CONTROL_ENABLED) {
    return manageRTS();  // manage_rts returns the available space in the buffer
  } else {
    return (uint32_t)(SERIAL_BUFFER_SIZE + _rx_buffer->_iHead - _rx_buffer->_iTail) % SERIAL_BUFFER_SIZE;
  }
}

int UARTClass::availableForWrite(void)
{
  int head = _tx_buffer->_iHead;
  int tail = _tx_buffer->_iTail;
  if (head >= tail) return SERIAL_BUFFER_SIZE - 1 - head + tail;
  return tail - head - 1;
}

int UARTClass::peek( void )
{
  if ( _rx_buffer->_iHead == _rx_buffer->_iTail )
    return -1;

  if (FLOW_CONTROL_ENABLED) {
    manageRTS();
  }

  return _rx_buffer->_aucBuffer[_rx_buffer->_iTail];
}

int UARTClass::read( void )
{
  // if the head isn't ahead of the tail, we don't have any characters
  if ( _rx_buffer->_iHead == _rx_buffer->_iTail )
    return -1;

  if (FLOW_CONTROL_ENABLED) {
    manageRTS();
  }

  uint8_t uc = _rx_buffer->_aucBuffer[_rx_buffer->_iTail];
  _rx_buffer->_iTail = (unsigned int)(_rx_buffer->_iTail + 1) % SERIAL_BUFFER_SIZE;
  return uc;
}

void UARTClass::flush( void )
{
  while (_tx_buffer->_iHead != _tx_buffer->_iTail); // wait for transmit data to be sent

  // Wait for transmission to complete
  while ((_pUart->UART_SR & UART_SR_TXEMPTY) != UART_SR_TXEMPTY)
   ;
}


size_t UARTClass::write( const uint8_t uc_data )
{
  int nextWrite = (_tx_buffer->_iHead + 1) % SERIAL_BUFFER_SIZE;
  while (_tx_buffer->_iTail == nextWrite || sendPaused) {
    ; // Spin locks if we're about to overwrite the buffer. This continues once the data is sent
  }

  _tx_buffer->_aucBuffer[_tx_buffer->_iHead] = uc_data;
  _tx_buffer->_iHead = nextWrite;

  // Make sure TX interrupt is enabled
  _pUart->UART_IER = UART_IER_TXRDY;

  return 1;
}

void UARTClass::IrqHandler( void )
{
  uint32_t status = _pUart->UART_SR;

  // Did we receive data?
  if ((status & UART_SR_RXRDY) == UART_SR_RXRDY)
    _rx_buffer->store_char(_pUart->UART_RHR);

  // Do we need to keep sending data?
  if ((status & UART_SR_TXRDY) == UART_SR_TXRDY) 
  {
    if (_tx_buffer->_iTail != _tx_buffer->_iHead) {
      _pUart->UART_THR = _tx_buffer->_aucBuffer[_tx_buffer->_iTail];
      _tx_buffer->_iTail = (unsigned int)(_tx_buffer->_iTail + 1) % SERIAL_BUFFER_SIZE;
    }
    else
    {
      // Mask off transmit interrupt so we don't get it anymore
      _pUart->UART_IDR = UART_IDR_TXRDY;
    }
  }

  // Acknowledge errors
  if ((status & UART_SR_OVRE) == UART_SR_OVRE || (status & UART_SR_FRAME) == UART_SR_FRAME)
  {
    // TODO: error reporting outside ISR
    _pUart->UART_CR |= UART_CR_RSTSTA;
  }
}